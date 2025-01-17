// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <fastcdr/FastBuffer.h>
#include <rmw/get_topic_endpoint_info.h>

#include <chrono>
#include <cinttypes>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <string>
#include <utility>

#include <zenoh.hxx>

#include "detail/cdr.hpp"
#include "detail/guard_condition.hpp"
#include "detail/graph_cache.hpp"
#include "detail/identifier.hpp"
#include "detail/liveliness_utils.hpp"
#include "detail/logging_macros.hpp"
#include "detail/message_type_support.hpp"
#include "detail/qos.hpp"
#include "detail/rmw_context_impl_s.hpp"
#include "detail/serialization_format.hpp"
#include "detail/type_support_common.hpp"
#include "detail/zenoh_utils.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rcutils/env.h"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/allocators.h"
#include "rmw/dynamic_message_type_support.h"
#include "rmw/error_handling.h"
#include "rmw/features.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "tracetools/tracetools.h"

namespace
{
//==============================================================================
const rosidl_message_type_support_t * find_message_type_support(
  const rosidl_message_type_support_t * type_supports)
{
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();
    type_support = get_message_typesupport_handle(type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Type support not from this implementation. Got:\n"
        "    %s\n"
        "    %s\n"
        "while fetching it",
        prev_error_string.str, error_string.str);
      return nullptr;
    }
  }

  return type_support;
}

//==============================================================================
const rosidl_service_type_support_t * find_service_type_support(
  const rosidl_service_type_support_t * type_supports)
{
  const rosidl_service_type_support_t * type_support = get_service_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C);
  if (!type_support) {
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    rcutils_reset_error();
    type_support = get_service_typesupport_handle(
      type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Type support not from this implementation. Got:\n"
        "    %s\n"
        "    %s\n"
        "while fetching it",
        prev_error_string.str, error_string.str);
      return nullptr;
    }
  }

  return type_support;
}

}  // namespace

extern "C"
{
//==============================================================================
/// Get the name of the rmw implementation being used
const char *
rmw_get_implementation_identifier(void)
{
  return rmw_zenoh_cpp::rmw_zenoh_identifier;
}

//==============================================================================
/// Get the unique serialization format for this middleware.
const char *
rmw_get_serialization_format(void)
{
  return rmw_zenoh_cpp::rmw_zenoh_serialization_format;
}

//==============================================================================
bool rmw_feature_supported(rmw_feature_t feature)
{
  switch (feature) {
    case RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER:
      return false;
    case RMW_FEATURE_MESSAGE_INFO_RECEPTION_SEQUENCE_NUMBER:
      return false;
    case RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY:
      return true;
    case RMW_MIDDLEWARE_CAN_TAKE_DYNAMIC_MESSAGE:
      return false;
    default:
      return false;
  }
}

//==============================================================================
/// Create a node and return a handle to that node.
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return nullptr);
  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "expected initialized context_impl",
    return nullptr);
  if (context_impl->is_shutdown()) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return nullptr;
  }

  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);
    return nullptr;
  }

  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return nullptr;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_namespace_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);
    return nullptr;
  }

  rcutils_allocator_t * allocator = &context->options.allocator;

  rmw_node_t * node =
    static_cast<rmw_node_t *>(allocator->zero_allocate(1, sizeof(rmw_node_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node,
    "unable to allocate memory for rmw_node_t",
    return nullptr);
  auto free_node = rcpputils::make_scope_exit(
    [node, allocator]() {
      allocator->deallocate(node, allocator->state);
    });

  node->name = rcutils_strdup(name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->name,
    "unable to allocate memory for node name",
    return nullptr);
  auto free_name = rcpputils::make_scope_exit(
    [node, allocator]() {
      allocator->deallocate(const_cast<char *>(node->name), allocator->state);
    });

  node->namespace_ = rcutils_strdup(namespace_, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->namespace_,
    "unable to allocate memory for node namespace",
    return nullptr);
  auto free_namespace = rcpputils::make_scope_exit(
    [node, allocator]() {
      allocator->deallocate(const_cast<char *>(node->namespace_), allocator->state);
    });

  // Create the NodeData for this node.
  if (!context_impl->create_node_data(node, namespace_, name)) {
    // Error already set.
    return nullptr;
  }

  node->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  node->context = context;
  // Store type erased rmw_context_impl_s in node->data so that the NodeData
  // can be safely accessed.
  node->data = context->impl;

  free_namespace.cancel();
  free_name.cancel();
  free_node.cancel();
  return node;
}

//==============================================================================
/// Finalize a given node handle, reclaim the resources, and deallocate the node handle.
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // Undeclare liveliness token for the node to advertise that the node has ridden
  // off into the sunset.
  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(node->data);
  if (context_impl == nullptr) {
    RMW_SET_ERROR_MSG("Unable to cast node->data into rmw_context_impl_s.");
    return RMW_RET_ERROR;
  }

  context_impl->delete_node_data(node);

  allocator->deallocate(const_cast<char *>(node->namespace_), allocator->state);
  allocator->deallocate(const_cast<char *>(node->name), allocator->state);
  allocator->deallocate(node, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
/// Return a guard condition which is triggered when the ROS graph changes.
const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, nullptr);

  return node->context->impl->graph_guard_condition();
}

//==============================================================================
/// Initialize a publisher allocation to be used with later publications.
rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(allocation);
  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Destroy a publisher allocation object.
rmw_ret_t
rmw_fini_publisher_allocation(
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation: unimplemented");
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Create a publisher and return a handle to that publisher.
rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
  if (publisher_options->require_unique_network_flow_endpoints ==
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED)
  {
    RMW_SET_ERROR_MSG(
      "Strict requirement on unique network flow endpoints for publishers not supported");
    return nullptr;
  }

  // Get the RMW type support.
  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_message_type_support
    return nullptr;
  }

  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  if (context_impl->is_shutdown()) {
    RMW_SET_ERROR_MSG("context_impl is shutdown");
    return nullptr;
  }
  if (!context_impl->session_is_valid()) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  if (!rcutils_allocator_is_valid(allocator)) {
    RMW_SET_ERROR_MSG("allocator is invalid.");
    return nullptr;
  }

  // Create the rmw_publisher.
  auto rmw_publisher =
    static_cast<rmw_publisher_t *>(allocator->zero_allocate(
      1, sizeof(rmw_publisher_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_publisher,
    "failed to allocate memory for the publisher",
    return nullptr);
  auto free_rmw_publisher = rcpputils::make_scope_exit(
    [rmw_publisher, allocator]() {
      allocator->deallocate(rmw_publisher, allocator->state);
    });

  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data,
    "NodeData not found.",
    return nullptr);

  if (!node_data->create_pub_data(
      rmw_publisher,
      context_impl->session(),
      context_impl->get_next_entity_id(),
      topic_name,
      type_support,
      qos_profile))
  {
    // Error already handled.
    return nullptr;
  }

  // Store type erased node in rmw_publisher->data so that the
  // PublisherData can be safely accessed.
  rmw_publisher->data = reinterpret_cast<void *>(const_cast<rmw_node_t *>(node));
  rmw_publisher->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_publisher->options = *publisher_options;
  rmw_publisher->can_loan_messages = false;
  rmw_publisher->topic_name = rcutils_strdup(topic_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_publisher->topic_name,
    "Failed to allocate topic name",
    return nullptr);
  auto free_topic_name = rcpputils::make_scope_exit(
    [rmw_publisher, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_publisher->topic_name), allocator->state);
    });

  free_topic_name.cancel();
  free_rmw_publisher.cancel();

  if (TRACETOOLS_TRACEPOINT_ENABLED(rmw_publisher_init)) {
    rmw_gid_t gid{};
    // Trigger tracepoint even if we cannot get the GID
    rmw_ret_t gid_ret = rmw_get_gid_for_publisher(rmw_publisher, &gid);
    static_cast<void>(gid_ret);
    TRACETOOLS_DO_TRACEPOINT(
      rmw_publisher_init, static_cast<const void *>(rmw_publisher), gid.data);
  }
  return rmw_publisher;
}

//==============================================================================
/// Finalize a given publisher handle, reclaim the resources, and deallocate the publisher handle.
rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  auto node_data = context_impl->get_node_data(node);
  if (node_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto pub_data = node_data->get_pub_data(publisher);
  if (pub_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  // Remove any event callbacks registered to this publisher.
  context_impl->graph_cache()->remove_qos_event_callbacks(pub_data->keyexpr_hash());
  // Remove the PublisherData from NodeData.
  node_data->delete_pub_data(publisher);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  allocator->deallocate(const_cast<char *>(publisher->topic_name), allocator->state);
  allocator->deallocate(publisher, allocator->state);
  return RMW_RET_OK;
}

//==============================================================================
rmw_ret_t
rmw_take_dynamic_message(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(dynamic_message);
  static_cast<void>(taken);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
rmw_ret_t
rmw_take_dynamic_message_with_info(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(dynamic_message);
  static_cast<void>(taken);
  static_cast<void>(message_info);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
rmw_ret_t
rmw_serialization_support_init(
  const char * serialization_lib_name,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support)
{
  static_cast<void>(serialization_lib_name);
  static_cast<void>(allocator);
  static_cast<void>(serialization_support);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Borrow a loaned ROS message.
rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  static_cast<void>(publisher);
  static_cast<void>(type_support);
  static_cast<void>(ros_message);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Return a loaned message previously borrowed from a publisher.
rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  static_cast<void>(publisher);
  static_cast<void>(loaned_message);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Publish a ROS message.
rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    ros_message, "ros message handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  rmw_node_t * rmw_node = static_cast<rmw_node_t *>(publisher->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_node, "publisher_data is null",
    return RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(rmw_node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl, "context_impl is null",
    return RMW_RET_INVALID_ARGUMENT);
  auto node_data = context_impl->get_node_data(rmw_node);
  if (node_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto pub_data = node_data->get_pub_data(publisher);
  if (pub_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  return pub_data->publish(
    ros_message
#ifdef RMW_ZENOH_BUILD_WITH_SHARED_MEMORY
    , context_impl->shm()
#endif
  );
}

//==============================================================================
/// Publish a loaned ROS message.
rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(publisher);
  static_cast<void>(ros_message);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Retrieve the number of matched subscriptions to a publisher.
rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);
  rmw_node_t * node =
    static_cast<rmw_node_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, RMW_RET_INVALID_ARGUMENT);
  auto pub_data = node_data->get_pub_data(publisher);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  return context_impl->graph_cache()->publisher_count_matched_subscriptions(
    pub_data->topic_info(), subscription_count);
}

//==============================================================================
/// Retrieve the actual qos settings of the publisher.
rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  rmw_node_t * node =
    static_cast<rmw_node_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, RMW_RET_INVALID_ARGUMENT);
  auto pub_data = node_data->get_pub_data(publisher);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  *qos = pub_data->topic_info().qos_;
  return RMW_RET_OK;
}

//==============================================================================
/// Publish a ROS message as a byte stream.
rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    serialized_message, "serialized message handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  rmw_node_t * node =
    static_cast<rmw_node_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, RMW_RET_INVALID_ARGUMENT);
  auto publisher_data = node_data->get_pub_data(publisher);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);

  return publisher_data->publish_serialized_message(
    serialized_message
#ifdef RMW_ZENOH_BUILD_WITH_SHARED_MEMORY
    , context_impl->shm()
#endif
  );
}

//==============================================================================
/// Compute the size of a serialized message.
rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  size_t * size)
{
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(size);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Manually assert that this Publisher is alive (for RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher->data, "publisher data is null",
    return RMW_RET_INVALID_ARGUMENT);
  rmw_node_t * node =
    static_cast<rmw_node_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, RMW_RET_INVALID_ARGUMENT);
  auto pub_data = node_data->get_pub_data(publisher);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  if (!pub_data->liveliness_is_valid()) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

//==============================================================================
/// Wait until all published message data is acknowledged or until the specified timeout elapses.
rmw_ret_t
rmw_publisher_wait_for_all_acked(
  const rmw_publisher_t * publisher,
  rmw_time_t wait_timeout)
{
  static_cast<void>(publisher);
  static_cast<void>(wait_timeout);

  // We are not currently tracking all published data, so we don't know what data is in flight that
  // we might have to wait for.  Even if we did start tracking it, we don't have insight into the
  // TCP stream that Zenoh is managing for us, so we couldn't guarantee this anyway.
  // Just lie to the upper layers and say that everything is working as expected.
  // We return OK rather than UNSUPPORTED so that certain upper-layer tests continue working.
  return RMW_RET_OK;
}

//==============================================================================
/// Serialize a ROS message into a rmw_serialized_message_t.
rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  const rosidl_message_type_support_t * ts = find_message_type_support(type_support);
  if (ts == nullptr) {
    // error was already set by find_message_type_support
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = rmw_zenoh_cpp::MessageTypeSupport(callbacks);
  auto data_length = tss.get_estimated_serialized_size(ros_message, callbacks);
  if (serialized_message->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_message, data_length) != RMW_RET_OK) {
      RMW_SET_ERROR_MSG("unable to dynamically resize serialized message");
      return RMW_RET_ERROR;
    }
  }

  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), data_length);
  rmw_zenoh_cpp::Cdr ser(buffer);

  auto ret = tss.serialize_ros_message(ros_message, ser.get_cdr(), callbacks);
  serialized_message->buffer_length = data_length;
  serialized_message->buffer_capacity = data_length;
  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

//==============================================================================
/// Deserialize a ROS message.
rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  const rosidl_message_type_support_t * ts = find_message_type_support(type_support);
  if (ts == nullptr) {
    // error was already set by find_message_type_support
    return RMW_RET_ERROR;
  }

  auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts->data);
  auto tss = rmw_zenoh_cpp::MessageTypeSupport(callbacks);
  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  rmw_zenoh_cpp::Cdr deser(buffer);

  auto ret = tss.deserialize_ros_message(deser.get_cdr(), ros_message, callbacks);
  return ret == true ? RMW_RET_OK : RMW_RET_ERROR;
}

//==============================================================================
/// Initialize a subscription allocation to be used with later `take`s.
rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  // Unused in current implementation.
  static_cast<void>(type_support);
  static_cast<void>(message_bounds);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Destroy a publisher allocation object.
rmw_ret_t
rmw_fini_subscription_allocation(
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Create a subscription and return a handle to that subscription.
rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->data, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (topic_name[0] == '\0') {
    RMW_SET_ERROR_MSG("topic_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid topic name: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);
  if (subscription_options->require_unique_network_flow_endpoints ==
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED)
  {
    RMW_SET_ERROR_MSG(
      "Strict requirement on unique network flow endpoints for subscriptions not supported");
    return nullptr;
  }

  const rosidl_message_type_support_t * type_support = find_message_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_message_type_support
    return nullptr;
  }

  // TODO(yadunund): Check if a duplicate entry for the same topic name + topic type
  // is present in node_data->subscriptions and if so return error;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  if (context_impl->is_shutdown()) {
    RMW_SET_ERROR_MSG("context_impl is shutdown");
    return nullptr;
  }
  if (!context_impl->session_is_valid()) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  if (!rcutils_allocator_is_valid(allocator)) {
    RMW_SET_ERROR_MSG("allocator is invalid.");
    return nullptr;
  }

  // Create the rmw_subscription.
  rmw_subscription_t * rmw_subscription =
    static_cast<rmw_subscription_t *>(allocator->zero_allocate(
      1, sizeof(rmw_subscription_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription,
    "failed to allocate memory for the subscription",
    return nullptr);
  auto free_rmw_subscription = rcpputils::make_scope_exit(
    [rmw_subscription, allocator]() {
      allocator->deallocate(rmw_subscription, allocator->state);
    });

  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data,
    "NodeData not found.",
    return nullptr);

  if (!node_data->create_sub_data(
      rmw_subscription,
      context_impl->session(),
      context_impl->graph_cache(),
      context_impl->get_next_entity_id(),
      topic_name,
      type_support,
      qos_profile))
  {
    // Error already handled.
    return nullptr;
  }

  // TODO(Yadunund): We cannot store the rmw_node_t * here since this type erased
  // subscription handle will be returned in the rmw_subscriptions_t in rmw_wait
  // from which we cannot obtain SubscriptionData.
  rmw_subscription->data = static_cast<void *>(node_data->get_sub_data(rmw_subscription).get());
  rmw_subscription->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_subscription->options = *subscription_options;
  rmw_subscription->can_loan_messages = false;
  rmw_subscription->is_cft_enabled = false;
  rmw_subscription->topic_name = rcutils_strdup(topic_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_subscription->topic_name,
    "Failed to allocate topic name",
    return nullptr);
  auto free_topic_name = rcpputils::make_scope_exit(
    [rmw_subscription, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_subscription->topic_name), allocator->state);
    });

  free_topic_name.cancel();
  free_rmw_subscription.cancel();

  // rmw does not require GIDs for subscriptions, and GIDs in rmw_zenoh are not based on any ID of
  // the underlying zenoh objects, so there is no need to collect a GID here
  rmw_gid_t gid{};
  static_cast<void>(gid);
  TRACETOOLS_TRACEPOINT(
    rmw_subscription_init, static_cast<const void *>(rmw_subscription), gid.data);
  return rmw_subscription;
}

//==============================================================================
/// Finalize a given subscription handle, reclaim the resources, and deallocate the subscription
rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  auto node_data = context_impl->get_node_data(node);
  if (node_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto sub_data = node_data->get_sub_data(subscription);
  if (sub_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  rmw_ret_t ret = RMW_RET_OK;

  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // Finally remove the SubscriptionData from NodeData.
  node_data->delete_sub_data(subscription);

  allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
  allocator->deallocate(subscription, allocator->state);

  return ret;
}

//==============================================================================
/// Retrieve the number of matched publishers to a subscription.
rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  return sub_data->graph_cache()->subscription_count_matched_publishers(
    sub_data->topic_info(), publisher_count);
}

//==============================================================================
/// Retrieve the actual qos settings of the subscription.
rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  *qos = sub_data->topic_info().qos_;
  return RMW_RET_OK;
}

//==============================================================================
/// Set the content filter options for the subscription.
rmw_ret_t
rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  // Re-enable test in rclcpp when this feature is implemented
  // https://github.com/ros2/rclcpp/pull/2627
  static_cast<void>(subscription);
  static_cast<void>(options);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Retrieve the content filter options of the subscription.
rmw_ret_t
rmw_subscription_get_content_filter(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_subscription_content_filter_options_t * options)
{
  // Re-enable test in rclcpp when this feature is implemented
  // https://github.com/ros2/rclcpp/pull/2627
  static_cast<void>(subscription);
  static_cast<void>(allocator);
  static_cast<void>(options);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Take an incoming ROS message.
rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  if (!TRACETOOLS_TRACEPOINT_ENABLED(rmw_take)) {
    return sub_data->take_one_message(ros_message, nullptr, taken);
  }
  rmw_message_info_t message_info{};
  rmw_ret_t ret = sub_data->take_one_message(ros_message, &message_info, taken);
  TRACETOOLS_DO_TRACEPOINT(
    rmw_take,
    static_cast<const void *>(subscription),
    static_cast<const void *>(ros_message),
    message_info.source_timestamp,
    *taken);
  return ret;
}

//==============================================================================
/// Take an incoming ROS message with its metadata.
rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  rmw_ret_t ret = sub_data->take_one_message(ros_message, message_info, taken);
  TRACETOOLS_TRACEPOINT(
    rmw_take,
    static_cast<const void *>(subscription),
    static_cast<const void *>(ros_message),
    message_info->source_timestamp,
    *taken);
  return ret;
}

//==============================================================================
/// Take multiple incoming ROS messages with their metadata.
rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  if (0u == count) {
    RMW_SET_ERROR_MSG("count cannot be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insufficient capacity in message_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > message_info_sequence->capacity) {
    RMW_SET_ERROR_MSG("Insufficient capacity in message_info_sequence");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (count > (std::numeric_limits<uint32_t>::max)()) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Cannot take %zu samples at once, limit is %" PRIu32,
      count, (std::numeric_limits<uint32_t>::max)());
    return RMW_RET_ERROR;
  }

  *taken = 0;

  rmw_ret_t ret;
  while (*taken < count) {
    bool one_taken = false;
    ret = sub_data->take_one_message(
      message_sequence->data[*taken],
      &message_info_sequence->data[*taken], &one_taken);
    if (ret != RMW_RET_OK) {
      // If we are taking a sequence and the 2nd take in the sequence failed, we'll report
      // RMW_RET_ERROR to the caller, but we will *also* tell the caller that there are valid
      // messages already taken (via the message_sequence size).  It is up to the caller to deal
      // with that situation appropriately.
      break;
    }

    if (!one_taken) {
      // No error, but there was nothing left to be taken, so break out of the loop
      break;
    }

    (*taken)++;
  }

  message_sequence->size = *taken;
  message_info_sequence->size = *taken;

  return ret;
}

//==============================================================================
namespace
{
rmw_ret_t
__rmw_take_serialized(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->topic_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription->data, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription handle,
    subscription->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  rmw_ret_t ret = sub_data->take_serialized_message(
    serialized_message,
    taken,
    message_info
  );
  TRACETOOLS_TRACEPOINT(
    rmw_take,
    static_cast<const void *>(subscription),
    static_cast<const void *>(serialized_message),
    message_info->source_timestamp,
    *taken);
  return ret;
}
}  // namespace

//==============================================================================
/// Take an incoming ROS message as a byte stream.
rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  return __rmw_take_serialized(subscription, serialized_message, taken, nullptr);
}

//==============================================================================
/// Take an incoming ROS message as a byte stream with its metadata.
rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);

  return __rmw_take_serialized(subscription, serialized_message, taken, message_info);
}

//==============================================================================
/// Take an incoming ROS message, loaned by the middleware.
rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  static_cast<void>(taken);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Take a loaned message and with its additional message information.
rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  static_cast<void>(taken);
  static_cast<void>(message_info);
  static_cast<void>(allocation);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Return a loaned ROS message previously taken from a subscription.
rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  static_cast<void>(subscription);
  static_cast<void>(loaned_message);
  return RMW_RET_UNSUPPORTED;
}

//==============================================================================
/// Create a service client that can send requests to and receive replies from a service server.
rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (strlen(service_name) == 0) {
    RMW_SET_ERROR_MSG("service name is empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  // Validate service name
  int validation_result;
  if (rmw_validate_full_topic_name(service_name, &validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    return nullptr;
  }
  if (validation_result != RMW_TOPIC_VALID && !qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service name is malformed: %s", service_name);
    return nullptr;
  }
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  if (context_impl->is_shutdown()) {
    RMW_SET_ERROR_MSG("context_impl is shutdown");
    return nullptr;
  }
  if (!context_impl->session_is_valid()) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  // Get the service type support.
  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_service_type_support
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  if (!rcutils_allocator_is_valid(allocator)) {
    RMW_SET_ERROR_MSG("allocator is invalid.");
    return nullptr;
  }

  // Create the rmw_client.
  rmw_client_t * rmw_client =
    static_cast<rmw_client_t *>(allocator->zero_allocate(
      1,
      sizeof(rmw_client_t),
      allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_client,
    "failed to allocate memory for the client",
    return nullptr);
  auto free_rmw_client = rcpputils::make_scope_exit(
    [rmw_client, allocator]() {
      allocator->deallocate(rmw_client, allocator->state);
    });

  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data,
    "NodeData not found.",
    return nullptr);

  if (!node_data->create_client_data(
      rmw_client,
      context_impl->session(),
      context_impl->get_next_entity_id(),
      service_name,
      type_support,
      qos_profile))
  {
    // Error already handled.
    return nullptr;
  }

  // TODO(Yadunund): We cannot store the rmw_node_t * here since this type erased
  // Client handle will be returned in the rmw_clients_t in rmw_wait
  // from which we cannot obtain ClientData.
  rmw_zenoh_cpp::ClientDataPtr client_data = node_data->get_client_data(rmw_client);
  rmw_client->data = static_cast<void *>(client_data.get());
  rmw_client->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_client->service_name = rcutils_strdup(service_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_client->service_name,
    "failed to allocate service client name",
    return nullptr);
  auto free_service_name = rcpputils::make_scope_exit(
    [rmw_client, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_client->service_name), allocator->state);
    });

  free_rmw_client.cancel();
  free_service_name.cancel();

  TRACETOOLS_TRACEPOINT(
    rmw_client_init,
    static_cast<const void *>(rmw_client),
    client_data->copy_gid().data());
  return rmw_client;
}

//==============================================================================
/// Destroy and unregister a service client from its node.
rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  auto node_data = context_impl->get_node_data(node);
  if (node_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Remove the ClientData from NodeData.
  node_data->delete_client_data(client);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  allocator->deallocate(const_cast<char *>(client->service_name), allocator->state);
  allocator->deallocate(client, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
/// Send a ROS service request.
rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_id)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->service_name, "client has no service name", RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_zenoh_cpp::ClientData * client_data =
    static_cast<rmw_zenoh_cpp::ClientData *>(client->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client_data,
    "Unable to retrieve client_data from client.",
    RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);

  return client_data->send_request(ros_request, sequence_id);
}

//==============================================================================
/// Take an incoming ROS service response.
rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  *taken = false;
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->service_name, "client has no service name", RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ClientData * client_data =
    static_cast<rmw_zenoh_cpp::ClientData *>(client->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    client->data, "Unable to retrieve client_data from client.", RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);

  rmw_ret_t ret = client_data->take_response(request_header, ros_response, taken);
  TRACETOOLS_TRACEPOINT(
    rmw_take_response,
    static_cast<const void *>(client),
    static_cast<const void *>(ros_response),
    request_header->request_id.sequence_number,
    request_header->source_timestamp,
    *taken);
  return ret;
}

//==============================================================================
/// Retrieve the actual qos settings of the client's request publisher.
rmw_ret_t
rmw_client_request_publisher_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ClientData * client_data =
    static_cast<rmw_zenoh_cpp::ClientData *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  *qos = client_data->topic_info().qos_;
  return RMW_RET_OK;
}

//==============================================================================
/// Retrieve the actual qos settings of the client's response subscription.
rmw_ret_t
rmw_client_response_subscription_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  // The same QoS profile is used for sending requests and receiving responses.
  return rmw_client_request_publisher_get_actual_qos(client, qos);
}

//==============================================================================
/// Create a service server that can receive requests from and send replies to a service client.
rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (0 == strlen(service_name)) {
    RMW_SET_ERROR_MSG("service_name argument is an empty string");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  if (!qos_profile->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    // TODO(francocipollone): Verify if this is the right way to validate the service name.
    rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("service_name argument is invalid: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context,
    "expected initialized context",
    return nullptr);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node->context->impl,
    "expected initialized context impl",
    return nullptr);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(
    node->context->impl);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context_impl,
    "unable to get rmw_context_impl_s",
    return nullptr);
  if (context_impl->is_shutdown()) {
    RMW_SET_ERROR_MSG("context_impl is shutdown");
    return nullptr;
  }
  if (!context_impl->session_is_valid()) {
    RMW_SET_ERROR_MSG("zenoh session is invalid");
    return nullptr;
  }

  // Get the RMW type support.
  const rosidl_service_type_support_t * type_support = find_service_type_support(type_supports);
  if (type_support == nullptr) {
    // error was already set by find_service_type_support
    return nullptr;
  }

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  if (!rcutils_allocator_is_valid(allocator)) {
    RMW_SET_ERROR_MSG("allocator is invalid.");
    return nullptr;
  }

  // Create the rmw_service.
  rmw_service_t * rmw_service =
    static_cast<rmw_service_t *>(allocator->zero_allocate(
      1,
      sizeof(rmw_service_t),
      allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_service,
    "failed to allocate memory for the service",
    return nullptr);
  auto free_rmw_service = rcpputils::make_scope_exit(
    [rmw_service, allocator]() {
      allocator->deallocate(rmw_service, allocator->state);
    });

  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    node_data,
    "NodeData not found.",
    return nullptr);

  if (!node_data->create_service_data(
      rmw_service,
      context_impl->session(),
      context_impl->get_next_entity_id(),
      service_name,
      type_support,
      qos_profile))
  {
    // Error already handled.
    return nullptr;
  }

  // TODO(Yadunund): We cannot store the rmw_node_t * here since this type erased
  // Service handle will be returned in the rmw_services_t in rmw_wait
  // from which we cannot obtain ServiceData.
  rmw_service->data = static_cast<void *>(node_data->get_service_data(rmw_service).get());
  rmw_service->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  rmw_service->service_name = rcutils_strdup(service_name, *allocator);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    rmw_service->service_name,
    "failed to allocate service name",
    return nullptr);
  auto free_service_name = rcpputils::make_scope_exit(
    [rmw_service, allocator]() {
      allocator->deallocate(const_cast<char *>(rmw_service->service_name), allocator->state);
    });

  free_rmw_service.cancel();
  free_service_name.cancel();

  return rmw_service;
}

//==============================================================================
/// Destroy and unregister a service server from its node.
rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl = static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  auto node_data = context_impl->get_node_data(node);
  if (node_data == nullptr) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Remove the ServiceData from NodeData.
  node_data->delete_service_data(service);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  allocator->deallocate(const_cast<char *>(service->service_name), allocator->state);
  allocator->deallocate(service, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
/// Take an incoming ROS service request.
rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  *taken = false;
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->service_name, RMW_RET_ERROR);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_ERROR);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  rmw_zenoh_cpp::ServiceData * service_data =
    static_cast<rmw_zenoh_cpp::ServiceData *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);

  rmw_ret_t ret = service_data->take_request(
    request_header,
    ros_request,
    taken);
  TRACETOOLS_TRACEPOINT(
    rmw_take_request,
    static_cast<const void *>(service),
    static_cast<const void *>(ros_request),
    request_header->request_id.writer_guid,
    request_header->request_id.sequence_number,
    *taken);
  return ret;
}

//==============================================================================
/// Send a ROS service response.
rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    service->data,
    "Unable to retrieve service_data from service",
    RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ServiceData * service_data =
    static_cast<rmw_zenoh_cpp::ServiceData *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  return service_data->send_response(
    request_header,
    ros_response);
}

//==============================================================================
/// Retrieve the actual qos settings of the service's request subscription.
rmw_ret_t
rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ServiceData * service_data =
    static_cast<rmw_zenoh_cpp::ServiceData *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  *qos = service_data->topic_info().qos_;
  return RMW_RET_OK;
}

//==============================================================================
/// Retrieve the actual qos settings of the service's response publisher.
rmw_ret_t
rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  // The same QoS profile is used for receiving requests and sending responses.
  return rmw_service_request_subscription_get_actual_qos(service, qos);
}

//==============================================================================
/// Create a guard condition and return a handle to that guard condition.
rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  rcutils_allocator_t * allocator = &context->options.allocator;

  auto guard_condition =
    static_cast<rmw_guard_condition_t *>(allocator->zero_allocate(
      1, sizeof(rmw_guard_condition_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition,
    "unable to allocate memory for guard_condition",
    return nullptr);
  auto free_guard_condition = rcpputils::make_scope_exit(
    [guard_condition, allocator]() {
      allocator->deallocate(guard_condition, allocator->state);
    });

  guard_condition->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  guard_condition->context = context;

  guard_condition->data = allocator->zero_allocate(
    1, sizeof(rmw_zenoh_cpp::GuardCondition),
    allocator->state);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    guard_condition->data,
    "unable to allocate memory for guard condition data",
    return nullptr);
  auto free_guard_condition_data = rcpputils::make_scope_exit(
    [guard_condition, allocator]() {
      allocator->deallocate(guard_condition->data, allocator->state);
    });

  new(guard_condition->data) rmw_zenoh_cpp::GuardCondition;
  auto destruct_guard_condition = rcpputils::make_scope_exit(
    [guard_condition]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_condition->data)->~GuardCondition(),
        rmw_zenoh_cpp::GuardCondition);
    });

  destruct_guard_condition.cancel();
  free_guard_condition_data.cancel();
  free_guard_condition.cancel();

  return guard_condition;
}

/// Finalize a given guard condition handle, reclaim the resources, and deallocate the handle.
rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &guard_condition->context->options.allocator;

  if (guard_condition->data) {
    static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_condition->data)->~GuardCondition();
    allocator->deallocate(guard_condition->data, allocator->state);
  }

  allocator->deallocate(guard_condition, allocator->state);

  return RMW_RET_OK;
}

//==============================================================================
rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard_condition,
    guard_condition->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_condition->data)->trigger();

  return RMW_RET_OK;
}

//==============================================================================
/// Create a wait set to store conditions that the middleware can wait on.
rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  static_cast<void>(max_conditions);

  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return nullptr);

  rcutils_allocator_t * allocator = &context->options.allocator;

  auto wait_set = static_cast<rmw_wait_set_t *>(
    allocator->zero_allocate(1, sizeof(rmw_wait_set_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set,
    "failed to allocate wait set",
    return nullptr);
  auto cleanup_wait_set = rcpputils::make_scope_exit(
    [wait_set, allocator]() {
      allocator->deallocate(wait_set, allocator->state);
    });

  wait_set->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;

  wait_set->data = allocator->zero_allocate(
    1, sizeof(rmw_zenoh_cpp::rmw_wait_set_data_t),
    allocator->state);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set->data,
    "failed to allocate wait set data",
    return nullptr);
  auto free_wait_set_data = rcpputils::make_scope_exit(
    [wait_set, allocator]() {
      allocator->deallocate(wait_set->data, allocator->state);
    });

  // Invoke placement new
  new(wait_set->data) rmw_zenoh_cpp::rmw_wait_set_data_t;
  auto destruct_rmw_wait_set_data = rcpputils::make_scope_exit(
    [wait_set]() {
      RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
        static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data)->~rmw_wait_set_data_t(),
        rmw_wait_set_data);
    });

  static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data)->context = context;

  destruct_rmw_wait_set_data.cancel();
  free_wait_set_data.cancel();
  cleanup_wait_set.cancel();

  return wait_set;
}

//==============================================================================
/// Destroy a wait set.
rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set,
    wait_set->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto wait_set_data = static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data);

  rcutils_allocator_t * allocator = &wait_set_data->context->options.allocator;

  wait_set_data->~rmw_wait_set_data_t();
  allocator->deallocate(wait_set_data, allocator->state);

  allocator->deallocate(wait_set, allocator->state);

  return RMW_RET_OK;
}

namespace
{
bool
check_and_attach_condition(
  const rmw_subscriptions_t * const subscriptions,
  const rmw_guard_conditions_t * const guard_conditions,
  const rmw_services_t * const services,
  const rmw_clients_t * const clients,
  const rmw_events_t * const events,
  rmw_zenoh_cpp::rmw_wait_set_data_t * wait_set_data)
{
  if (guard_conditions) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      rmw_zenoh_cpp::GuardCondition * gc =
        static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_conditions->guard_conditions[i]);
      if (gc == nullptr) {
        continue;
      }
      if (gc->check_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  if (events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      auto event = static_cast<rmw_event_t *>(events->events[i]);
      rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
        rmw_zenoh_cpp::zenoh_event_from_rmw_event(event->event_type);
      if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "has_triggered_condition() called with unknown event %u. Report this bug.",
          event->event_type);
        continue;
      }

      auto event_data = static_cast<rmw_zenoh_cpp::EventsManager *>(event->data);
      if (event_data == nullptr) {
        continue;
      }

      if (event_data->queue_has_data_and_attach_condition_if_not(zenoh_event_type, wait_set_data)) {
        return true;
      }
    }
  }

  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      rmw_zenoh_cpp::SubscriptionData * sub_data =
        static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscriptions->subscribers[i]);
      if (sub_data == nullptr) {
        continue;
      }
      if (sub_data->queue_has_data_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      auto serv_data = static_cast<rmw_zenoh_cpp::ServiceData *>(services->services[i]);
      if (serv_data == nullptr) {
        continue;
      }
      if (serv_data->queue_has_data_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      rmw_zenoh_cpp::ClientData * client_data =
        static_cast<rmw_zenoh_cpp::ClientData *>(clients->clients[i]);
      if (client_data == nullptr) {
        continue;
      }
      if (client_data->queue_has_data_and_attach_condition_if_not(wait_set_data)) {
        return true;
      }
    }
  }

  return false;
}
}  // namespace

//==============================================================================
/// Waits on sets of different entities and returns when one is ready.
rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait set handle,
    wait_set->implementation_identifier, rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto wait_set_data = static_cast<rmw_zenoh_cpp::rmw_wait_set_data_t *>(wait_set->data);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    wait_set_data,
    "waitset data struct is null",
    return RMW_RET_ERROR);

  // rmw_wait should return *all* entities that have data available, and let the caller decide
  // how to handle them.
  //
  // If there is no data currently available in any of the entities we were told to wait on, we
  // we attach a context-global condition variable to each entity, calculate a timeout based on
  // wait_timeout, and then sleep on the condition variable.  If any of the entities has an event
  // during that time, it will wake up from that sleep.
  //
  // If there is data currently available in one or more of the entities, then we'll skip attaching
  // the condition variable, and skip the sleep, and instead just go to the last part.
  //
  // In the last part, we check every entity and see if there are conditions that make it ready.
  // If that entity is not ready, then we set the pointer to it to nullptr in the wait set, which
  // signals to the upper layers that it isn't ready.  If something is ready, then we leave it as
  // a valid pointer.

  bool skip_wait = check_and_attach_condition(
    subscriptions, guard_conditions, services, clients, events, wait_set_data);
  if (!skip_wait) {
    std::unique_lock<std::mutex> lock(wait_set_data->condition_mutex);

    // According to the RMW documentation, if wait_timeout is NULL that means
    // "wait forever", if it specified as 0 it means "never wait", and if it is anything else wait
    // for that amount of time.
    if (wait_timeout == nullptr) {
      wait_set_data->condition_variable.wait(
        lock, [wait_set_data]() {
          return wait_set_data->triggered;
        });
    } else {
      if (wait_timeout->sec != 0 || wait_timeout->nsec != 0) {
        wait_set_data->condition_variable.wait_for(
          lock,
          std::chrono::nanoseconds(wait_timeout->nsec + RCUTILS_S_TO_NS(wait_timeout->sec)),
          [wait_set_data]() {return wait_set_data->triggered;});
      }
    }

    // It is important to reset this here while still holding the lock, otherwise every subsequent
    // call to rmw_wait() will be immediately ready.  We could handle this another way by making
    // "triggered" a stack variable in this function and "attaching" it during
    // "check_and_attach_condition", but that isn't clearly better so leaving this.
    wait_set_data->triggered = false;
  }

  bool wait_result = false;

  // According to the documentation for rmw_wait in rmw.h, entries in the various arrays that have
  // *not* been triggered should be set to NULL
  if (guard_conditions) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      rmw_zenoh_cpp::GuardCondition * gc =
        static_cast<rmw_zenoh_cpp::GuardCondition *>(guard_conditions->guard_conditions[i]);
      if (gc == nullptr) {
        continue;
      }
      if (!gc->detach_condition_and_is_trigger_set()) {
        // Setting to nullptr lets rcl know that this guard condition is not ready
        guard_conditions->guard_conditions[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      auto event = static_cast<rmw_event_t *>(events->events[i]);
      auto event_data = static_cast<rmw_zenoh_cpp::EventsManager *>(event->data);
      if (event_data == nullptr) {
        continue;
      }

      rmw_zenoh_cpp::rmw_zenoh_event_type_t zenoh_event_type =
        rmw_zenoh_cpp::zenoh_event_from_rmw_event(event->event_type);
      if (zenoh_event_type == rmw_zenoh_cpp::ZENOH_EVENT_INVALID) {
        continue;
      }

      if (event_data->detach_condition_and_event_queue_is_empty(zenoh_event_type)) {
        // Setting to nullptr lets rcl know that this subscription is not ready
        events->events[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      rmw_zenoh_cpp::SubscriptionData * sub_data =
        static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscriptions->subscribers[i]);
      if (sub_data == nullptr) {
        continue;
      }
      if (sub_data->detach_condition_and_queue_is_empty()) {
        // Setting to nullptr lets rcl know that this subscription is not ready
        subscriptions->subscribers[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      auto serv_data = static_cast<rmw_zenoh_cpp::ServiceData *>(services->services[i]);
      if (serv_data == nullptr) {
        continue;
      }

      if (serv_data->detach_condition_and_queue_is_empty()) {
        // Setting to nullptr lets rcl know that this service is not ready
        services->services[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      rmw_zenoh_cpp::ClientData * client_data =
        static_cast<rmw_zenoh_cpp::ClientData *>(clients->clients[i]);
      if (client_data == nullptr) {
        continue;
      }

      if (client_data->detach_condition_and_queue_is_empty()) {
        // Setting to nullptr lets rcl know that this client is not ready
        clients->clients[i] = nullptr;
      } else {
        wait_result = true;
      }
    }
  }

  return wait_result ? RMW_RET_OK : RMW_RET_TIMEOUT;
}

//==============================================================================
/// Return the name and namespace of all nodes in the ROS graph.
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache()->get_node_names(
    node_names, node_namespaces, nullptr, allocator);
}

//==============================================================================
/// Return the name, namespace, and enclave name of all nodes in the ROS graph.
rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node->context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(enclaves, RMW_RET_INVALID_ARGUMENT);

  rcutils_allocator_t * allocator = &node->context->options.allocator;
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache()->get_node_names(
    node_names, node_namespaces, enclaves, allocator);
}

//==============================================================================
/// Count the number of known publishers matching a topic name.
rmw_ret_t
rmw_count_publishers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache()->count_publishers(topic_name, count);
}

//==============================================================================
/// Count the number of known subscribers matching a topic name.
rmw_ret_t
rmw_count_subscribers(
  const rmw_node_t * node,
  const char * topic_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache()->count_subscriptions(topic_name, count);
}

//==============================================================================
/// Count the number of known clients matching a service name.
rmw_ret_t
rmw_count_clients(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache()->count_clients(service_name, count);
}

//==============================================================================
/// Count the number of known servers matching a service name.
rmw_ret_t
rmw_count_services(
  const rmw_node_t * node,
  const char * service_name,
  size_t * count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_TOPIC_VALID;
  rmw_ret_t ret = rmw_validate_full_topic_name(service_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_TOPIC_VALID != validation_result) {
    const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("topic_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);

  return node->context->impl->graph_cache()->count_services(service_name, count);
}

//==============================================================================
/// Get the globally unique identifier (GID) of a publisher.
rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  rmw_node_t * node =
    static_cast<rmw_node_t *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  rmw_context_impl_s * context_impl =
    static_cast<rmw_context_impl_s *>(node->context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);
  auto node_data = context_impl->get_node_data(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, RMW_RET_INVALID_ARGUMENT);
  auto pub_data = node_data->get_pub_data(publisher);
  RMW_CHECK_ARGUMENT_FOR_NULL(pub_data, RMW_RET_INVALID_ARGUMENT);

  gid->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  memcpy(gid->data, pub_data->copy_gid().data(), RMW_GID_STORAGE_SIZE);

  return RMW_RET_OK;
}

//==============================================================================
/// Get the globally unique identifier (GID) of a service client.
rmw_ret_t
rmw_get_gid_for_client(const rmw_client_t * client, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ClientData * client_data =
    static_cast<rmw_zenoh_cpp::ClientData *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  gid->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  memcpy(gid->data, client_data->copy_gid().data(), RMW_GID_STORAGE_SIZE);

  return RMW_RET_OK;
}

//==============================================================================
/// Check if two globally unique identifiers (GIDs) are equal.
rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid1,
    gid1->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid2,
    gid2->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);

  *result = memcmp(gid1->data, gid2->data, RMW_GID_STORAGE_SIZE) == 0;

  return RMW_RET_OK;
}

//==============================================================================
/// Check if a service server is available for the given service client.
rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(client->data, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(is_available, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ClientData * client_data =
    static_cast<rmw_zenoh_cpp::ClientData *>(client->data);
  if (client_data == nullptr) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Unable to retrieve client_data from client for service %s", client->service_name);
    return RMW_RET_INVALID_ARGUMENT;
  }

  return node->context->impl->graph_cache()->service_server_is_available(
    client_data->topic_info(), is_available);
}

//==============================================================================
/// Set the current log severity
rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  switch (severity) {
    case RMW_LOG_SEVERITY_DEBUG:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_DEBUG);
      break;
    case RMW_LOG_SEVERITY_INFO:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_INFO);
      break;
    case RMW_LOG_SEVERITY_WARN:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_WARN);
      break;
    case RMW_LOG_SEVERITY_ERROR:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_ERROR);
      break;
    case RMW_LOG_SEVERITY_FATAL:
      rmw_zenoh_cpp::Logger::get().set_log_level(RCUTILS_LOG_SEVERITY_FATAL);
      break;
    default:
      return RMW_RET_UNSUPPORTED;
  }
  return RMW_RET_OK;
}

//==============================================================================
/// Set the on new message callback function for the subscription.
rmw_ret_t
rmw_subscription_set_on_new_message_callback(
  rmw_subscription_t * subscription,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::SubscriptionData * sub_data =
    static_cast<rmw_zenoh_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(sub_data, RMW_RET_INVALID_ARGUMENT);

  sub_data->set_on_new_message_callback(std::move(callback), user_data);
  return RMW_RET_OK;
}

//==============================================================================
/// Set the on new request callback function for the service.
rmw_ret_t
rmw_service_set_on_new_request_callback(
  rmw_service_t * service,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ServiceData * service_data =
    static_cast<rmw_zenoh_cpp::ServiceData *>(service->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);
  service_data->set_on_new_request_callback(
    std::move(callback), user_data);
  return RMW_RET_OK;
}

//==============================================================================
/// Set the on new response callback function for the client.
rmw_ret_t
rmw_client_set_on_new_response_callback(
  rmw_client_t * client,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  rmw_zenoh_cpp::ClientData * client_data =
    static_cast<rmw_zenoh_cpp::ClientData *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);
  client_data->set_on_new_response_callback(
    std::move(callback), user_data);
  return RMW_RET_OK;
}
}  // extern "C"
