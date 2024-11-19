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

#include "rmw_client_data.hpp"

#include <fastcdr/FastBuffer.h>
#include <zenoh.h>

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "cdr.hpp"
#include "liveliness_utils.hpp"
#include "logging_macros.hpp"
#include "qos.hpp"
#include "rmw_context_impl_s.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"

namespace
{

///=============================================================================
void client_data_handler(z_loaned_reply_t * reply, void * data)
{
  auto client_data = static_cast<rmw_zenoh_cpp::ClientData *>(data);
  if (client_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t from data in client_data_handler."
    );
    return;
  }

  if (client_data->is_shutdown()) {
    return;
  }

  if (!z_reply_is_ok(reply)) {
    const z_loaned_reply_err_t * err = z_reply_err(reply);
    const z_loaned_bytes_t * err_payload = z_reply_err_payload(err);

    z_owned_string_t err_str;
    z_bytes_to_string(err_payload, &err_str);

    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_is_ok returned False for keyexpr %s. Reason: %.*s",
      client_data->topic_info().topic_keyexpr_.c_str(),
      static_cast<int>(z_string_len(z_loan(err_str))),
      z_string_data(z_loan(err_str)));
    z_drop(z_move(err_str));

    return;
  }

  std::chrono::nanoseconds::rep received_timestamp =
    std::chrono::system_clock::now().time_since_epoch().count();

  z_owned_reply_t owned_reply;
  z_reply_clone(&owned_reply, reply);
  client_data->add_new_reply(
    std::make_unique<rmw_zenoh_cpp::ZenohReply>(owned_reply, received_timestamp));
}

///=============================================================================
void client_data_drop(void * data)
{
  auto client_data = static_cast<rmw_zenoh_cpp::ClientData *>(data);
  if (client_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t from data in client_data_drop."
    );
    return;
  }

  client_data->decrement_in_flight_and_conditionally_remove();
}

}  // namespace

namespace rmw_zenoh_cpp
{
///=============================================================================
std::shared_ptr<ClientData> ClientData::make(
  const std::shared_ptr<zenoh::Session> & session,
  const rmw_node_t * const node,
  const rmw_client_t * client,
  liveliness::NodeInfo node_info,
  std::size_t node_id,
  std::size_t service_id,
  const std::string & service_name,
  const rosidl_service_type_support_t * type_support,
  const rmw_qos_profile_t * qos_profile)
{
  // Adapt any 'best available' QoS options
  rmw_qos_profile_t adapted_qos_profile = *qos_profile;
  rmw_ret_t ret = QoS::get().best_available_qos(
    nullptr, nullptr, &adapted_qos_profile, nullptr);
  if (RMW_RET_OK != ret) {
    RMW_SET_ERROR_MSG("Failed to obtain adapted_qos_profile.");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  const rosidl_type_hash_t * type_hash = type_support->get_type_hash_func(type_support);
  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data);
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data);
  auto request_type_support = std::make_shared<RequestTypeSupport>(service_members);
  auto response_type_support = std::make_shared<ResponseTypeSupport>(service_members);

  // Note: Service request/response types will contain a suffix Request_ or Response_.
  // We remove the suffix when appending the type to the liveliness tokens for
  // better reusability within GraphCache.
  std::string service_type = request_type_support->get_name();
  size_t suffix_substring_position = service_type.find("Request_");
  if (std::string::npos != suffix_substring_position) {
    service_type = service_type.substr(0, suffix_substring_position);
  } else {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unexpected type %s for client %s. Report this bug",
      service_type.c_str(), service_name.c_str());
    return nullptr;
  }

  // Convert the type hash to a string so that it can be included in the keyexpr.
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(
    type_hash,
    *allocator,
    &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    RMW_SET_ERROR_MSG("Failed to allocate type_hash_c_str.");
    return nullptr;
  }
  auto free_type_hash_c_str = rcpputils::make_scope_exit(
    [&allocator, &type_hash_c_str]() {
      allocator->deallocate(type_hash_c_str, allocator->state);
    });

  std::size_t domain_id = node_info.domain_id_;
  auto entity = liveliness::Entity::make(
    session->get_zid(),
    std::to_string(node_id),
    std::to_string(service_id),
    liveliness::EntityType::Client,
    std::move(node_info),
    liveliness::TopicInfo{
      std::move(domain_id),
      service_name,
      std::move(service_type),
      type_hash_c_str,
      std::move(adapted_qos_profile)}
  );
  if (entity == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to generate keyexpr for liveliness token for the client %s.",
      service_name.c_str());
    return nullptr;
  }

  std::shared_ptr<ClientData> client_data = std::shared_ptr<ClientData>(
    new ClientData{
      node,
      client,
      entity,
      request_members,
      response_members,
      request_type_support,
      response_type_support
    });

  if (!client_data->init(session)) {
    // init() already set the error.
    return nullptr;
  }

  return client_data;
}

///=============================================================================
ClientData::ClientData(
  const rmw_node_t * rmw_node,
  const rmw_client_t * rmw_client,
  std::shared_ptr<liveliness::Entity> entity,
  const void * request_type_support_impl,
  const void * response_type_support_impl,
  std::shared_ptr<RequestTypeSupport> request_type_support,
  std::shared_ptr<ResponseTypeSupport> response_type_support)
: rmw_node_(rmw_node),
  rmw_client_(rmw_client),
  entity_(std::move(entity)),
  request_type_support_impl_(request_type_support_impl),
  response_type_support_impl_(response_type_support_impl),
  request_type_support_(request_type_support),
  response_type_support_(response_type_support),
  wait_set_data_(nullptr),
  sequence_number_(1),
  is_shutdown_(false),
  num_in_flight_(0)
{
  // Do nothing.
}

///=============================================================================
bool ClientData::init(const std::shared_ptr<zenoh::Session> & session)
{
  std::string topic_keyexpr = this->entity_->topic_info().value().topic_keyexpr_;
  keyexpr_ = zenoh::KeyExpr(topic_keyexpr);

  std::string liveliness_keyexpr = this->entity_->liveliness_keyexpr();
  zenoh::ZResult err;
  this->token_ = session->liveliness_declare_token(
    zenoh::KeyExpr(liveliness_keyexpr),
    zenoh::Session::LivelinessDeclarationOptions::create_default(),
    &err);
  if (err != Z_OK)
  {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to create liveliness token for the client.");
    return false;
  }

  return true;
}

///=============================================================================
liveliness::TopicInfo ClientData::topic_info() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return entity_->topic_info().value();
}

///=============================================================================
void ClientData::copy_gid(uint8_t out_gid[RMW_GID_STORAGE_SIZE]) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  entity_->copy_gid(out_gid);
}

///=============================================================================
void ClientData::add_new_reply(std::unique_ptr<ZenohReply> reply)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const rmw_qos_profile_t adapted_qos_profile =
    entity_->topic_info().value().qos_;
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    reply_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_loan(keyexpr_.value()._0), &keystr);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Query queue depth of %ld reached, discarding oldest Query "
      "for client for %.*s",
      adapted_qos_profile.depth,
      static_cast<int>(z_string_len(z_loan(keystr))),
      z_string_data(z_loan(keystr)));
    reply_queue_.pop_front();
  }
  reply_queue_.emplace_back(std::move(reply));

  // Since we added new data, trigger user callback and guard condition if they are available
  data_callback_mgr_.trigger_callback();
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
rmw_ret_t ClientData::take_response(
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  *taken = false;

  if (is_shutdown_ || reply_queue_.empty()) {
    // This tells rcl that the check for a new message was done, but no messages have come in yet.
    return RMW_RET_OK;
  }
  std::unique_ptr<ZenohReply> latest_reply = std::move(reply_queue_.front());
  reply_queue_.pop_front();

  if (!latest_reply->get_sample().has_value()) {
    RMW_SET_ERROR_MSG("invalid reply sample");
    return RMW_RET_ERROR;
  }
  const z_loaned_sample_t * sample = latest_reply->get_sample().value();

  // Object that manages the raw buffer
  z_owned_slice_t payload;
  z_bytes_to_slice(z_sample_payload(sample), &payload);
  eprosima::fastcdr::FastBuffer fastbuffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(z_slice_data(z_loan(payload)))),
    z_slice_len(z_loan(payload)));

  // Object that serializes the data
  rmw_zenoh_cpp::Cdr deser(fastbuffer);
  if (!response_type_support_->deserialize_ros_message(
      deser.get_cdr(),
      ros_response,
      response_type_support_impl_))
  {
    RMW_SET_ERROR_MSG("could not deserialize ROS response");
    return RMW_RET_ERROR;
  }

  // Fill in the request_header
  rmw_zenoh_cpp::attachement_data_t attachment(z_sample_attachment(sample));
  request_header->request_id.sequence_number = attachment.sequence_number;
  if (request_header->request_id.sequence_number < 0) {
    RMW_SET_ERROR_MSG("Failed to get sequence_number from client call attachment");
    return RMW_RET_ERROR;
  }
  request_header->source_timestamp = attachment.source_timestamp;
  if (request_header->source_timestamp < 0) {
    RMW_SET_ERROR_MSG("Failed to get source_timestamp from client call attachment");
    return RMW_RET_ERROR;
  }
  memcpy(request_header->request_id.writer_guid, attachment.source_gid, RMW_GID_STORAGE_SIZE);
  request_header->received_timestamp = latest_reply->get_received_timestamp();

  z_drop(z_move(payload));
  *taken = true;

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t ClientData::send_request(
  const void * ros_request,
  int64_t * sequence_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (is_shutdown_) {
    return RMW_RET_OK;
  }

  rcutils_allocator_t * allocator = &rmw_node_->context->options.allocator;
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(rmw_node_->context->impl);
  if (context_impl == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  size_t max_data_length = (
    request_type_support_->get_estimated_serialized_size(
      ros_request, request_type_support_impl_));

  // Init serialized message byte array
  char * request_bytes = static_cast<char *>(allocator->allocate(
      max_data_length,
      allocator->state));
  if (!request_bytes) {
    RMW_SET_ERROR_MSG("failed allocate request message bytes");
    return RMW_RET_ERROR;
  }
  auto always_free_request_bytes = rcpputils::make_scope_exit(
    [request_bytes, allocator]() {
      allocator->deallocate(request_bytes, allocator->state);
    });

  // Object that manages the raw buffer
  eprosima::fastcdr::FastBuffer fastbuffer(request_bytes, max_data_length);
  // Object that serializes the data
  Cdr ser(fastbuffer);
  if (!request_type_support_->serialize_ros_message(
      ros_request,
      ser.get_cdr(),
      request_type_support_impl_))
  {
    return RMW_RET_ERROR;
  }
  size_t data_length = ser.get_serialized_data_length();
  *sequence_id = sequence_number_++;

  // Send request
  z_get_options_t opts;
  z_get_options_default(&opts);
  z_owned_bytes_t attachment;
  uint8_t local_gid[RMW_GID_STORAGE_SIZE];
  entity_->copy_gid(local_gid);
  rmw_zenoh_cpp::create_map_and_set_sequence_num(
    &attachment, *sequence_id,
    local_gid);
  opts.attachment = z_move(attachment);

  opts.target = Z_QUERY_TARGET_ALL_COMPLETE;
  // The default timeout for a z_get query is 10 seconds and if a response is not received within
  // this window, the queryable will return an invalid reply. However, it is common for actions,
  // which are implemented using services, to take an extended duration to complete. Hence, we set
  // the timeout_ms to the largest supported value to account for most realistic scenarios.
  opts.timeout_ms = std::numeric_limits<uint64_t>::max();
  // Latest consolidation guarantees unicity of replies for the same key expression,
  // which optimizes bandwidth. The default is "None", which imples replies may come in any order
  // and any number.
  opts.consolidation = z_query_consolidation_latest();

  z_owned_bytes_t payload;
  z_bytes_copy_from_buf(
    &payload, reinterpret_cast<const uint8_t *>(request_bytes), data_length);
  opts.payload = z_move(payload);

  // TODO(Yadunund): Once we switch to zenoh-cpp with lambda closures,
  // capture shared_from_this() instead of this.
  z_owned_closure_reply_t callback;
  z_closure(&callback, client_data_handler, client_data_drop, this);
  z_get(
    context_impl->session(),
    z_loan(keyexpr_.value()._0), "",
    z_move(callback),
    &opts);

  return RMW_RET_OK;
}

///=============================================================================
ClientData::~ClientData()
{
  const rmw_ret_t ret = this->shutdown();
  if (ret != RMW_RET_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Error destructing client /%s.",
      entity_->topic_info().value().name_.c_str()
    );
  }
}

//==============================================================================
void ClientData::set_on_new_response_callback(
  rmw_event_callback_t callback,
  const void * user_data)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  data_callback_mgr_.set_callback(user_data, std::move(callback));
}

///=============================================================================
bool ClientData::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!reply_queue_.empty()) {
    return true;
  }
  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool ClientData::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  wait_set_data_ = nullptr;

  return reply_queue_.empty();
}

///=============================================================================
void ClientData::_shutdown()
{
  if (is_shutdown_) {
    return;
  }

  // Unregister this node from the ROS graph.
  zenoh::ZResult err;
  std::move(token_).value().undeclare(&err);
  if (err != Z_OK)
  {
    RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to undeclare liveliness token");
    return;
  }

  is_shutdown_ = true;
}

///=============================================================================
rmw_ret_t ClientData::shutdown()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  _shutdown();
  return RMW_RET_OK;
}

///=============================================================================
bool ClientData::shutdown_and_query_in_flight()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  _shutdown();
  return num_in_flight_ > 0;
}

///=============================================================================
void ClientData::decrement_in_flight_and_conditionally_remove()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  --num_in_flight_;

  if (is_shutdown_ && num_in_flight_ == 0) {
    rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(rmw_node_->data);
    if (context_impl == nullptr) {
      return;
    }
    std::shared_ptr<rmw_zenoh_cpp::NodeData> node_data = context_impl->get_node_data(rmw_node_);
    if (node_data == nullptr) {
      return;
    }
    node_data->delete_client_data(rmw_client_);
  }
}

///=============================================================================
bool ClientData::is_shutdown() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return is_shutdown_;
}
}  // namespace rmw_zenoh_cpp