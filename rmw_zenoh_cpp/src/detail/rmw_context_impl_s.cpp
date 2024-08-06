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

#include "rmw_context_impl_s.hpp"

#include <utility>

#include "guard_condition.hpp"
#include "liveliness_utils.hpp"
#include "logging_macros.hpp"
#include "zenoh_config.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"

///=============================================================================
void rmw_context_impl_s::graph_sub_data_handler(const z_sample_t * sample, void * data)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  auto free_keystr = rcpputils::make_scope_exit(
    [&keystr]() {
      z_drop(z_move(keystr));
    });

  auto data_ptr = static_cast<Data *>(data);
  if (data_ptr == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Unable to lock data_wp."
    );
    return;
  }

  // Update the graph cache.
  std::lock_guard<std::mutex> lock(data_ptr->mutex_);
  if (data_ptr->is_shutdown_) {
    return;
  }
  switch (sample->kind) {
    case z_sample_kind_t::Z_SAMPLE_KIND_PUT:
      data_ptr->graph_cache_->parse_put(keystr._cstr);
      break;
    case z_sample_kind_t::Z_SAMPLE_KIND_DELETE:
      data_ptr->graph_cache_->parse_del(keystr._cstr);
      break;
    default:
      return;
  }

  // Trigger the ROS graph guard condition.
  rmw_ret_t rmw_ret = rmw_trigger_guard_condition(data_ptr->graph_guard_condition_);
  if (RMW_RET_OK != rmw_ret) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "[graph_sub_data_handler] Unable to trigger graph guard condition."
    );
  }
}

///=============================================================================
rmw_context_impl_s::Data::Data(
  const rcutils_allocator_t * allocator,
  const std::size_t domain_id,
  const std::string & enclave,
  z_owned_session_t session,
  std::optional<zc_owned_shm_manager_t> shm_manager,
  rmw_guard_condition_t * graph_guard_condition)
: allocator_(allocator),
  enclave_(std::move(enclave)),
  session_(std::move(session)),
  shm_manager_(std::move(shm_manager)),
  graph_guard_condition_(graph_guard_condition),
  is_shutdown_(false),
  next_entity_id_(0),
  is_initialized_(false)
{
  z_id_t zid = z_info_zid(z_loan(session_));
  graph_cache_ = std::make_unique<rmw_zenoh_cpp::GraphCache>(std::move(zid));
  // Setup liveliness subscriptions for discovery.
  liveliness_str_ = rmw_zenoh_cpp::liveliness::subscription_token(
    domain_id);

  // Query router/liveliness participants to get graph information before this session was started.
  // We create a blocking channel that is unbounded, ie. `bound` = 0, to receive
  // replies for the zc_liveliness_get() call. This is necessary as if the `bound`
  // is too low, the channel may starve the zenoh executor of its threads which
  // would lead to deadlocks when trying to receive replies and block the
  // execution here.
  // The blocking channel will return when the sender end is closed which is
  // the moment the query finishes.
  // The non-blocking fifo exists only for the use case where we don't want to
  // block the thread between responses (including the request termination response).
  // In general, unless we want to cooperatively schedule other tasks on the same
  // thread as reading the fifo, the blocking fifo will be more appropriate as
  // the code will be simpler, and if we're just going to spin over the non-blocking
  // reads until we obtain responses, we'll just be hogging CPU time by convincing
  // the OS that we're doing actual work when it could instead park the thread.
  z_owned_reply_channel_t channel = zc_reply_fifo_new(0);
  zc_liveliness_get(
    z_loan(session_), z_keyexpr(liveliness_str_.c_str()),
    z_move(channel.send), NULL);
  z_owned_reply_t reply = z_reply_null();
  for (bool call_success = z_call(channel.recv, &reply); !call_success || z_check(reply);
    call_success = z_call(channel.recv, &reply))
  {
    if (!call_success) {
      continue;
    }
    if (z_reply_is_ok(&reply)) {
      z_sample_t sample = z_reply_ok(&reply);
      z_owned_str_t keystr = z_keyexpr_to_string(sample.keyexpr);
      // Ignore tokens from the same session to avoid race conditions from this
      // query and the liveliness subscription.
      graph_cache_->parse_put(z_loan(keystr), true);
      z_drop(z_move(keystr));
    } else {
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp", "[rmw_context_impl_s] z_call received an invalid reply\n");
    }
  }
  z_drop(z_move(reply));
  z_drop(z_move(channel));
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::Data::subscribe()
{
  if (is_initialized_) {
    return RMW_RET_OK;
  }
  // Setup the liveliness subscriber to receives updates from the ROS graph
  // and update the graph cache.
  auto sub_options = zc_liveliness_subscriber_options_null();
  z_owned_closure_sample_t callback = z_closure(
    rmw_context_impl_s::graph_sub_data_handler, nullptr,
    this->shared_from_this().get());
  graph_subscriber_ = zc_liveliness_declare_subscriber(
    z_loan(session_),
    z_keyexpr(liveliness_str_.c_str()),
    z_move(callback),
    &sub_options);
  zc_liveliness_subscriber_options_drop(z_move(sub_options));
  auto undeclare_z_sub = rcpputils::make_scope_exit(
    [this]() {
      z_undeclare_subscriber(z_move(this->graph_subscriber_));
    });
  if (!z_check(graph_subscriber_)) {
    RMW_SET_ERROR_MSG("unable to create zenoh subscription");
    return RMW_RET_ERROR;
  }

  undeclare_z_sub.cancel();
  is_initialized_ = true;
  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::Data::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_shutdown_) {
    return RMW_RET_OK;
  }

  z_undeclare_subscriber(z_move(graph_subscriber_));
  if (shm_manager_.has_value()) {
    z_drop(z_move(shm_manager_.value()));
  }
  // Close the zenoh session
  if (z_close(z_move(session_)) < 0) {
    RMW_SET_ERROR_MSG("Error while closing zenoh session");
    return RMW_RET_ERROR;
  }
  is_shutdown_ = true;
  return RMW_RET_OK;
}

///=============================================================================
rmw_context_impl_s::Data::~Data()
{
  RMW_TRY_DESTRUCTOR(
    static_cast<rmw_zenoh_cpp::GuardCondition *>(
      graph_guard_condition_->data)->~GuardCondition(),
    rmw_zenoh_cpp::GuardCondition, );
  if (rcutils_allocator_is_valid(allocator_)) {
    allocator_->deallocate(graph_guard_condition_->data, allocator_->state);
    allocator_->deallocate(graph_guard_condition_, allocator_->state);
    graph_guard_condition_ = nullptr;
  }

  auto ret = this->shutdown();
  static_cast<void>(ret);
}

///=============================================================================
rmw_context_impl_s::rmw_context_impl_s(
  const rcutils_allocator_t * allocator,
  const std::size_t domain_id,
  const std::string & enclave,
  z_owned_session_t session,
  std::optional<zc_owned_shm_manager_t> shm_manager,
  rmw_guard_condition_t * graph_guard_condition)
{
  data_ = std::make_shared<Data>(
    allocator,
    std::move(domain_id),
    std::move(enclave),
    std::move(session),
    std::move(shm_manager),
    graph_guard_condition);

  // TODO(Yadunund): Consider switching to a make() pattern to avoid throwing
  // errors.
  auto ret = data_->subscribe();
  if (ret != RMW_RET_OK) {
    throw std::runtime_error("Unable to subscribe to ROS Graph updates.");
  }
}

///=============================================================================
std::string rmw_context_impl_s::enclave() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->enclave_;
}

///=============================================================================
z_session_t rmw_context_impl_s::session() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return z_loan(data_->session_);
}

///=============================================================================
std::optional<zc_owned_shm_manager_t> & rmw_context_impl_s::shm_manager()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->shm_manager_;
}

///=============================================================================
rmw_guard_condition_t * rmw_context_impl_s::graph_guard_condition()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_guard_condition_;
}

///=============================================================================
size_t rmw_context_impl_s::get_next_entity_id()
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->next_entity_id_++;
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::shutdown()
{
  return data_->shutdown();
}

///=============================================================================
bool rmw_context_impl_s::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->is_shutdown_;
}

///=============================================================================
bool rmw_context_impl_s::session_is_valid() const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return z_check(data_->session_);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::get_node_names(
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves,
  rcutils_allocator_t * allocator) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->get_node_names(
    node_names,
    node_namespaces,
    enclaves,
    allocator);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::get_topic_names_and_types(
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->get_topic_names_and_types(
    allocator,
    no_demangle,
    topic_names_and_types);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->publisher_count_matched_subscriptions(
    publisher,
    subscription_count);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->subscription_count_matched_publishers(
    subscription,
    publisher_count);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::get_service_names_and_types(
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->get_service_names_and_types(
    allocator,
    service_names_and_types);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::count_publishers(
  const char * topic_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->count_publishers(
    topic_name,
    count);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::count_subscriptions(
  const char * topic_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->count_subscriptions(
    topic_name,
    count);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::count_services(
  const char * service_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->count_services(
    service_name,
    count);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::count_clients(
  const char * service_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->count_clients(
    service_name,
    count);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::get_entity_names_and_types_by_node(
  rmw_zenoh_cpp::liveliness::EntityType entity_type,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * names_and_types) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->get_entity_names_and_types_by_node(
    entity_type,
    allocator,
    node_name,
    node_namespace,
    no_demangle,
    names_and_types);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::get_entities_info_by_topic(
  rmw_zenoh_cpp::liveliness::EntityType entity_type,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_demangle,
  rmw_topic_endpoint_info_array_t * endpoints_info) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->get_entities_info_by_topic(
    entity_type,
    allocator,
    topic_name,
    no_demangle,
    endpoints_info);
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::service_server_is_available(
  const char * service_name,
  const char * service_type,
  bool * is_available) const
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->service_server_is_available(
    service_name,
    service_type,
    is_available);
}

///=============================================================================
void rmw_context_impl_s::set_qos_event_callback(
  rmw_zenoh_cpp::liveliness::ConstEntityPtr entity,
  const rmw_zenoh_cpp::rmw_zenoh_event_type_t & event_type,
  GraphCacheEventCallback callback)
{
  std::lock_guard<std::mutex> lock(data_->mutex_);
  return data_->graph_cache_->set_qos_event_callback(
    std::move(entity),
    event_type,
    std::move(callback));
}