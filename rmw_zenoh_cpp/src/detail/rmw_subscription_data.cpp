// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmw_subscription_data.hpp"

#include <fastcdr/FastBuffer.h>

#include <cinttypes>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

#include "attachment_helpers.hpp"
#include "cdr.hpp"
#include "identifier.hpp"
#include "rmw_context_impl_s.hpp"
#include "message_type_support.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/impl/cpp/macros.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
SubscriptionData::Message::Message(
  const zenoh::Bytes & p,
  uint64_t recv_ts,
  AttachmentData && attachment_)
: payload(p), recv_timestamp(recv_ts), attachment(std::move(attachment_))
{
}

///=============================================================================
std::shared_ptr<SubscriptionData> SubscriptionData::make(
  std::shared_ptr<zenoh::Session> session,
  std::shared_ptr<GraphCache> graph_cache,
  const rmw_node_t * const node,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t subscription_id,
  const std::string & topic_name,
  const rosidl_message_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    node, topic_name.c_str(), &adapted_qos_profile, rmw_get_publishers_info_by_topic);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  auto message_type_support = std::make_unique<MessageTypeSupport>(callbacks);

  // Convert the type hash to a string so that it can be included in the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    // rosidl_stringify_type_hash already set the error
    return nullptr;
  }
  auto free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  // Everything above succeeded and is setup properly. Now declare a subscriber
  // with Zenoh; after this, callbacks may come in at any time.
  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    session->get_zid(),
    std::to_string(node_id),
    std::to_string(subscription_id),
    liveliness::EntityType::Subscription,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      topic_name,
      message_type_support->get_name(),
      type_hash_c_str,
      adapted_qos_profile}
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the subscription %s.",
      topic_name.c_str());
    return nullptr;
  }

  auto sub_data = std::shared_ptr<SubscriptionData>(
    new SubscriptionData{
      node,
      graph_cache,
      std::move(entity),
      std::move(session),
      type_support->data,
      std::move(message_type_support)
    });

  if (!sub_data->init()) {
    // init() already set the error
    return nullptr;
  }

  return sub_data;
}

///=============================================================================
SubscriptionData::SubscriptionData(
  const rmw_node_t * rmw_node,
  std::shared_ptr<GraphCache> graph_cache,
  std::shared_ptr<liveliness::Entity> entity,
  std::shared_ptr<zenoh::Session> session,
  const void * type_support_impl,
  std::unique_ptr<MessageTypeSupport> type_support)
: rmw_node_(rmw_node),
  graph_cache_(std::move(graph_cache)),
  entity_(std::move(entity)),
  sess_(std::move(session)),
  type_support_impl_(type_support_impl),
  type_support_(std::move(type_support)),
  last_known_published_msg_({}),
  wait_set_data_(nullptr),
  is_shutdown_(false),
  initialized_(false)
{
  events_mgr_ = std::make_shared<EventsManager>();
}

// We have to use an "init" function here, rather than do this in the constructor, because we use
// enable_shared_from_this, which is not available in constructors.
bool SubscriptionData::init()
{
  zenoh::ZResult result;
  zenoh::KeyExpr sub_ke(entity_->topic_info()->topic_keyexpr_, true, &result);
  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh keyexpr.");
    return false;
  }

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(rmw_node_->context->impl);

  sess_ = context_impl->session();

  auto adv_sub_opts = zenoh::ext::SessionExt::AdvancedSubscriberOptions::create_default();

  // Instantiate the subscription with suitable options depending on the
  // adapted_qos_profile.
  // TODO(Yadunund): Rely on a separate function to return the sub
  // as we start supporting more qos settings.
  if (entity_->topic_info()->qos_.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    // Allow this subscriber to be detected through liveliness.
    adv_sub_opts.subscriber_detection = true;
    adv_sub_opts.query_timeout_ms = std::numeric_limits<uint64_t>::max();
    // History can only be retransmitted by Publishers that enable caching.
    adv_sub_opts.history =
      zenoh::ext::SessionExt::AdvancedSubscriberOptions::HistoryOptions::create_default();
    adv_sub_opts.history->detect_late_publishers = true;
    adv_sub_opts.history->max_samples = entity_->topic_info()->qos_.depth;
    adv_sub_opts.recovery = zenoh::ext::SessionExt::AdvancedSubscriberOptions::RecoveryOptions{};
    adv_sub_opts.recovery->periodic_queries_period_ms = 1000;
  }

  std::weak_ptr<SubscriptionData> data_wp = shared_from_this();
  auto on_sample = [data_wp](const zenoh::Sample & sample) {
      auto sub_data = data_wp.lock();
      if (sub_data == nullptr) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "SubscriberCallback triggered over %s.",
          std::string(sample.get_keyexpr().as_string_view()).c_str()
        );
        return;
      }
      auto attachment = sample.get_attachment();
      if (!attachment.has_value()) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Unable to obtain attachment")
        return;
      }
      auto attachment_value = attachment.value();

      AttachmentData attachment_data(attachment_value);
      sub_data->add_new_message(
        std::make_unique<SubscriptionData::Message>(
          sample.get_payload(),
          get_system_time_in_ns(),
          std::move(attachment_data)),
        std::string(sample.get_keyexpr().as_string_view()));
    };
  sub_ = context_impl->session()->ext().declare_advanced_subscriber(
    sub_ke,
    on_sample,
    zenoh::closures::none,
    std::move(adv_sub_opts),
    &result);
  if (result != Z_OK) {
    RMW_SET_ERROR_MSG("unable to create zenoh subscription");
    return false;
  }

  // Publish to the graph that a new subscription is in town.
  std::string liveliness_keyexpr = entity_->liveliness_keyexpr();
  token_ = context_impl->session()->liveliness_declare_token(
    zenoh::KeyExpr(liveliness_keyexpr),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the subscription.");
    return false;
  }

  initialized_ = true;

  return true;
}

///=============================================================================
std::size_t SubscriptionData::keyexpr_hash() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->keyexpr_hash();
}

///=============================================================================
liveliness::TopicInfo SubscriptionData::topic_info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entity_->topic_info().value();
}

///=============================================================================
bool SubscriptionData::liveliness_is_valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // The z_check function is now internal in zenoh-1.0.0 so we assume
  // the liveliness token is still initialized as long as this entity has
  // not been shutdown.
  return !is_shutdown_;
}

///=============================================================================
std::shared_ptr<EventsManager> SubscriptionData::events_mgr() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_mgr_;
}

///=============================================================================
SubscriptionData::~SubscriptionData()
{
  const rmw_ret_t ret = this->shutdown();
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing publisher /%s.",
      entity_->topic_info().value().name_.c_str()
    );
  }
}

///=============================================================================
rmw_ret_t SubscriptionData::shutdown()
{
  rmw_ret_t ret = RMW_RET_OK;
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || !initialized_) {
    return ret;
  }

  // Remove any event callbacks registered to this subscription.
  graph_cache_->remove_qos_event_callbacks(entity_->keyexpr_hash());

  // Unregister this subscription from the ROS graph.
  zenoh::ZResult result;
  std::move(token_).value().undeclare(&result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to undeclare liveliness token");
    return RMW_RET_ERROR;
  }

  if (sub_.has_value()) {
    std::move(sub_.value()).undeclare(&result);
    if (result != Z_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "failed to undeclare sub.");
      return RMW_RET_ERROR;
    }
  }

  sess_.reset();
  is_shutdown_ = true;
  initialized_ = false;
  return ret;
}

///=============================================================================
bool SubscriptionData::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_shutdown_;
}

///=============================================================================
bool SubscriptionData::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!message_queue_.empty()) {
    return true;
  }

  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool SubscriptionData::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(mutex_);
  wait_set_data_ = nullptr;

  return message_queue_.empty();
}

///=============================================================================
rmw_ret_t SubscriptionData::take_one_message(
  void * ros_message,
  rmw_message_info_t * message_info,
  bool * taken)
{
  *taken = false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  auto & payload_data = msg_data->payload;

  if (payload_data.empty()) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData not able to get slice data");
    return RMW_RET_ERROR;
  }
  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(payload_data.data())),
    payload_data.size());

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);
  if (!type_support_->deserialize_ros_message(
      deser.get_cdr(),
      ros_message,
      type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not deserialize ROS message");
    return RMW_RET_ERROR;
  }

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->attachment.source_timestamp();
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->attachment.sequence_number();
    // TODO(clalancette): fill in reception_sequence_number
    message_info->reception_sequence_number = 0;
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(
      message_info->publisher_gid.data,
      msg_data->attachment.copy_gid().data(),
      RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }
  *taken = true;

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t SubscriptionData::take_serialized_message(
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  *taken = false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_ || message_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<Message> msg_data = std::move(message_queue_.front());
  message_queue_.pop_front();

  auto & payload_data = msg_data->payload;

  if (payload_data.empty()) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "SubscriptionData not able to get slice data");
    return RMW_RET_ERROR;
  }
  if (serialized_message->buffer_capacity < payload_data.size()) {
    rmw_ret_t ret =
      rmw_serialized_message_resize(serialized_message, payload_data.size());
    if (ret != RMW_RET_OK) {
      return ret;  // Error message already set
    }
  }
  serialized_message->buffer_length = payload_data.size();
  memcpy(
    serialized_message->buffer,
    reinterpret_cast<char *>(const_cast<uint8_t *>(payload_data.data())),
    payload_data.size());

  *taken = true;

  if (message_info != nullptr) {
    message_info->source_timestamp = msg_data->attachment.source_timestamp();
    message_info->received_timestamp = msg_data->recv_timestamp;
    message_info->publication_sequence_number = msg_data->attachment.sequence_number();
    // TODO(clalancette): fill in reception_sequence_number
    message_info->reception_sequence_number = 0;
    message_info->publisher_gid.implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    memcpy(
      message_info->publisher_gid.data,
      msg_data->attachment.copy_gid().data(),
      RMW_GID_STORAGE_SIZE);
    message_info->from_intra_process = false;
  }

  return RMW_RET_OK;
}

///=============================================================================
void SubscriptionData::add_new_message(
  std::unique_ptr<SubscriptionData::Message> msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return;
  }
  const rmw_qos_profile_t adapted_qos_profile = entity_->topic_info().value().qos_;
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    message_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Message queue depth of %ld reached, discarding oldest message "
      "for subscription for %s",
      adapted_qos_profile.depth,
      topic_name.c_str());

    // If the adapted_qos_profile.depth is 0, the std::move command below will result
    // in UB and the z_drop will segfault. We explicitly set the depth to a minimum of 1
    // in rmw_create_subscription() but to be safe, we only attempt to discard from the
    // queue if it is non-empty.
    if (!message_queue_.empty()) {
      std::unique_ptr<Message> old = std::move(message_queue_.front());
      message_queue_.pop_front();
    }
  }

  // Check for messages lost if the new sequence number is not monotonically increasing.
  const size_t gid_hash = hash_gid(msg->attachment.copy_gid());
  auto last_known_pub_it = last_known_published_msg_.find(gid_hash);
  if (last_known_pub_it != last_known_published_msg_.end()) {
    const int64_t seq_increment = std::abs(
      msg->attachment.sequence_number() -
      last_known_pub_it->second);
    if (seq_increment > 1) {
      const size_t num_msg_lost = seq_increment - 1;
      events_mgr_->update_event_status(
        ZENOH_EVENT_MESSAGE_LOST,
        num_msg_lost);
    }
  }
  // Always update the last known sequence number for the publisher.
  last_known_published_msg_[gid_hash] = msg->attachment.sequence_number();

  message_queue_.emplace_back(std::move(msg));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr_.trigger_callback();
  if (wait_set_data_ != nullptr) {
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

//==============================================================================
void SubscriptionData::set_on_new_message_callback(
  rmw_event_callback_t callback,
  const void * user_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_callback_mgr_.set_callback(user_data, std::move(callback));
}

//==============================================================================
std::shared_ptr<GraphCache> SubscriptionData::graph_cache() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_cache_;
}

}  // namespace rmw_zenoh_cpp
