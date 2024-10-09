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

#ifndef DETAIL__RMW_NODE_DATA_HPP_
#define DETAIL__RMW_NODE_DATA_HPP_

#include <zenoh.h>

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "graph_cache.hpp"
#include "liveliness_utils.hpp"
#include "rmw_publisher_data.hpp"
#include "rmw_subscription_data.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
// The NodeData can only be created via rmw_context_impl_s::create_node_data().
class NodeData final
{
public:
  // Make a shared_ptr of NodeData. Returns nullptr if construction fails.
  static std::shared_ptr<NodeData> make(
    const rmw_node_t * const node,
    std::size_t id,
    const z_loaned_session_t * session,
    std::size_t domain_id,
    const std::string & namespace_,
    const std::string & node_name,
    const std::string & enclave);

  // Get the id of this node.
  std::size_t id() const;

  // Create a new PublisherData for a publisher in this node.
  bool create_pub_data(
    const rmw_publisher_t * const publisher,
    const z_loaned_session_t * session,
    std::size_t id,
    const std::string & topic_name,
    const rosidl_message_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  /// Retrieve the PublisherData for a given rmw_publisher_t if present.
  PublisherDataPtr get_pub_data(const rmw_publisher_t * const publisher);

  // Delete the PublisherData for a given rmw_publisher_t if present.
  void delete_pub_data(const rmw_publisher_t * const publisher);

  // Create a new SubscriptionData for a publisher in this node.
  bool create_sub_data(
    const rmw_subscription_t * const publisher,
    z_session_t session,
    std::shared_ptr<GraphCache> graph_cache,
    std::size_t id,
    const std::string & topic_name,
    const rosidl_message_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  /// Retrieve the SubscriptionData for a given rmw_subscription_t if present.
  SubscriptionDataPtr get_sub_data(const rmw_subscription_t * const publisher);

  // Delete the SubscriptionData for a given rmw_subscription_t if present.
  void delete_sub_data(const rmw_subscription_t * const publisher);

  // Shutdown this NodeData.
  rmw_ret_t shutdown();

  // Check if this NodeData is shutdown.
  bool is_shutdown() const;

  // Destructor.
  ~NodeData();

private:
  // Constructor.
  NodeData(
    const rmw_node_t * const node,
    std::size_t id,
    std::shared_ptr<liveliness::Entity> entity,
    zc_owned_liveliness_token_t token);
  // Internal mutex.
  mutable std::mutex mutex_;
  // The rmw_node_t associated with this NodeData.
  const rmw_node_t * node_;
  // The entity id of this node as generated by get_next_entity_id().
  // Every interface created by this node will include this id in its liveliness token.
  std::size_t id_;
  // The Entity generated for the node.
  std::shared_ptr<liveliness::Entity> entity_;
  // Liveliness token for the node.
  zc_owned_liveliness_token_t token_;
  // Shutdown flag.
  bool is_shutdown_;
  // Map of publishers.
  std::unordered_map<const rmw_publisher_t *, PublisherDataPtr> pubs_;
  // Map of subscriptions.
  std::unordered_map<const rmw_subscription_t *, SubscriptionDataPtr> subs_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_NODE_DATA_HPP_
