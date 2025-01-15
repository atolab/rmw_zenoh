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

#ifndef DETAIL__RMW_SERVICE_DATA_HPP_
#define DETAIL__RMW_SERVICE_DATA_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <zenoh.hxx>

#include "event.hpp"
#include "liveliness_utils.hpp"
#include "message_type_support.hpp"
#include "service_type_support.hpp"
#include "type_support_common.hpp"
#include "zenoh_utils.hpp"

#include "rcutils/allocator.h"

#include "rmw/rmw.h"
#include "rmw/ret_types.h"

namespace rmw_zenoh_cpp
{

///=============================================================================
class ServiceData final : public std::enable_shared_from_this<ServiceData>
{
public:
  // Make a shared_ptr of ServiceData.
  static std::shared_ptr<ServiceData> make(
    std::shared_ptr<zenoh::Session> session,
    const rmw_node_t * const node,
    const rmw_service_t * rmw_service,
    liveliness::NodeInfo node_info,
    std::size_t node_id,
    std::size_t service_id,
    const std::string & service_name,
    const rosidl_service_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  // Get a copy of the TopicInfo of this ServiceData.
  liveliness::TopicInfo topic_info() const;

  // Returns true if liveliness token is still valid.
  bool liveliness_is_valid() const;

  // Add a new ZenohQuery to the queue.
  void add_new_query(std::unique_ptr<ZenohQuery> query);

  // Take a ROS service request.
  rmw_ret_t take_request(
    rmw_service_info_t * request_header,
    void * ros_request,
    bool * taken);

  // Send a service response.
  rmw_ret_t send_response(
    rmw_request_id_t * request_id,
    void * ros_response);

  void set_on_new_request_callback(
    rmw_event_callback_t callback,
    const void * user_data);

  bool queue_has_data_and_attach_condition_if_not(
    rmw_wait_set_data_t * wait_set_data);

  bool detach_condition_and_queue_is_empty();

  // Shutdown this ServiceData.
  rmw_ret_t shutdown();

  // Check if this ServiceData is shutdown.
  bool is_shutdown() const;

  // Destructor.
  ~ServiceData();

private:
  // Constructor.
  ServiceData(
    const rmw_node_t * rmw_node,
    const rmw_service_t * rmw_service,
    std::shared_ptr<liveliness::Entity> entity,
    std::shared_ptr<zenoh::Session> session,
    const void * request_type_support_impl,
    const void * response_type_support_impl,
    std::unique_ptr<RequestTypeSupport> request_type_support,
    std::unique_ptr<ResponseTypeSupport> response_type_support);

  // Internal mutex.
  mutable std::mutex mutex_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // The rmw service.
  const rmw_service_t * rmw_service_;
  // The Entity generated for the service.
  std::shared_ptr<liveliness::Entity> entity_;

  // A shared session
  std::shared_ptr<zenoh::Session> sess_;
  // The keyexpr string.
  std::string keyexpr_;
  // An owned queryable.
  // The Queryable *must* exist in order for anything in this ServiceData class,
  // and hence rmw_zenoh_cpp, to work.
  // However, zenoh::Queryable does not have an empty constructor,
  // so just declaring this as a zenoh::Queryable fails to compile.
  // We work around that by wrapping it in a std::optional, so the std::optional
  // gets constructed at ServiceData constructor time,
  // and then we initialize qable_ later. Note that the zenoh-cpp API declare_queryable() throws an
  // exception if it fails, so this should all be safe to do.
  std::optional<zenoh::Queryable<void>> qable_;
  // Liveliness token for the service.
  // The token_ *must* exist in order for anything in this ServiceData class,
  // and hence rmw_zenoh_cpp, to work.
  // However, zenoh::LivelinessToken does not have an empty constructor,
  // so just declaring this as a zenoh::LivelinessToken fails to compile.
  // We work around that by wrapping it in a std::optional, so the std::optional
  // gets constructed at ServiceData constructor time,
  // and then we initialize token_ later. Note that the zenoh-cpp API
  // liveliness_declare_token() throws an exception if it fails, so this should all be safe to do.
  std::optional<zenoh::LivelinessToken> token_;
  // Type support fields.
  const void * request_type_support_impl_;
  const void * response_type_support_impl_;
  std::unique_ptr<RequestTypeSupport> request_type_support_;
  std::unique_ptr<ResponseTypeSupport> response_type_support_;
  // Deque to store the queries in the order they arrive.
  std::deque<std::unique_ptr<ZenohQuery>> query_queue_;
  // Map to store the sequence_number (as given by the client) -> ZenohQuery
  using SequenceToQuery = std::unordered_map<int64_t, std::unique_ptr<ZenohQuery>>;
  std::unordered_map<Gid, SequenceToQuery> sequence_to_query_map_;
  // Wait set data.
  rmw_wait_set_data_t * wait_set_data_;
  // Data callback manager.
  DataCallbackManager data_callback_mgr_;
  // Shutdown flag.
  bool is_shutdown_;
  // Whether the object has ever successfully been initialized.
  bool initialized_;
};
using ServiceDataPtr = std::shared_ptr<ServiceData>;
using ServiceDataConstPtr = std::shared_ptr<const ServiceData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_SERVICE_DATA_HPP_
