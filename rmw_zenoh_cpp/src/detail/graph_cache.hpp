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

#ifndef DETAIL__GRAPH_CACHE_HPP_
#define DETAIL__GRAPH_CACHE_HPP_

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "event.hpp"
#include "liveliness_utils.hpp"
#include "ordered_map.hpp"

#include "rcutils/allocator.h"
#include "rcutils/types.h"

#include "rmw/rmw.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/names_and_types.h"

#include <zenoh/api/id.hxx>

namespace rmw_zenoh_cpp
{
///=============================================================================
// TODO(Yadunund): Consider changing this to an array of unordered_set where the index of the
// array corresponds to the EntityType enum. This way we don't need to mix
// pub/sub with client/service.
class TopicData;
using TopicDataPtr = std::shared_ptr<TopicData>;
class TopicData
{
public:
  liveliness::TopicInfo info_;

  using EntitySet = std::unordered_set<
    liveliness::ConstEntityPtr>;

  // The publishers or clients entities.
  EntitySet pubs_;

  // The subscriptions or services entities
  EntitySet subs_;

  // Returns nullptr if the entity does not contain topic_info.
  static TopicDataPtr make(liveliness::ConstEntityPtr entity);

private:
  // Private constructor to force users to rely on make.
  explicit TopicData(liveliness::ConstEntityPtr entity);
};

///=============================================================================
struct GraphNode
{
  std::string zid_;
  std::string nid_;
  std::string ns_;
  std::string name_;
  // TODO(Yadunund): Should enclave be the parent to the namespace key and not within a Node?
  std::string enclave_;

  // Map QoS (serialized using liveliness::qos_to_keyexpr) to TopicData
  using TopicQoSMap = std::unordered_map<std::string, TopicDataPtr>;
  // Map topic type to QoSMap
  using TopicTypeMap = std::unordered_map<std::string, TopicQoSMap>;
  // Map topic name to TopicTypeMap
  // This uses a map that remembers insertion order because some parts of the client libraries
  // expect that these are returned in the order that they were created.
  using TopicMap = tsl::ordered_map<std::string, TopicTypeMap>;

  // Entries for pub/sub.
  TopicMap pubs_ = {};
  TopicMap subs_ = {};

  // Entries for service/client.
  TopicMap clients_ = {};
  TopicMap services_ = {};
};
using GraphNodePtr = std::shared_ptr<GraphNode>;

///=============================================================================
class GraphCache final
{
public:
  /// @brief Signature for a function that will be invoked by the GraphCache when a QoS
  ///   event is detected.
  using GraphCacheEventCallback = std::function<void (int32_t change)>;

  /// @brief Constructor
  /// @param id The id of the zenoh session that is building the graph cache.
  ///   This is used to infer which entities originated from the current session
  ///   so that appropriate event callbacks may be triggered.
  explicit GraphCache(const zenoh::Id & zid);

  // Parse a PUT message over a token's key-expression and update the graph.
  void parse_put(const std::string & keyexpr, bool ignore_from_current_session = false);
  // Parse a DELETE message over a token's key-expression and update the graph.
  void parse_del(const std::string & keyexpr, bool ignore_from_current_session = false);

  rmw_ret_t get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_string_array_t * enclaves,
    rcutils_allocator_t * allocator) const;

  rmw_ret_t get_topic_names_and_types(
    rcutils_allocator_t * allocator,
    bool no_demangle,
    rmw_names_and_types_t * topic_names_and_types) const;

  rmw_ret_t publisher_count_matched_subscriptions(
    const liveliness::TopicInfo & pub_topic_info,
    size_t * subscription_count);

  rmw_ret_t subscription_count_matched_publishers(
    const liveliness::TopicInfo & sub_topic_info,
    size_t * publisher_count);

  rmw_ret_t get_service_names_and_types(
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * service_names_and_types) const;

  rmw_ret_t count_publishers(
    const char * topic_name,
    size_t * count) const;

  rmw_ret_t count_subscriptions(
    const char * topic_name,
    size_t * count) const;

  rmw_ret_t count_services(
    const char * service_name,
    size_t * count) const;

  rmw_ret_t count_clients(
    const char * service_name,
    size_t * count) const;

  rmw_ret_t get_entity_names_and_types_by_node(
    liveliness::EntityType entity_type,
    rcutils_allocator_t * allocator,
    const char * node_name,
    const char * node_namespace,
    bool no_demangle,
    rmw_names_and_types_t * names_and_types) const;

  rmw_ret_t get_entities_info_by_topic(
    liveliness::EntityType entity_type,
    rcutils_allocator_t * allocator,
    const char * topic_name,
    bool no_demangle,
    rmw_topic_endpoint_info_array_t * endpoints_info) const;

  rmw_ret_t service_server_is_available(
    const liveliness::TopicInfo & client_topic_info,
    bool * is_available) const;

  /// Set a qos event callback for an entity from the current session.
  /// @note The callback will be removed when the entity is removed from the graph.
  void set_qos_event_callback(
    std::size_t entity_keyexpr_hash,
    const rmw_zenoh_event_type_t & event_type,
    GraphCacheEventCallback callback);

  /// Remove all qos event callbacks for an entity.
  void remove_qos_event_callbacks(std::size_t entity_keyexpr_hash);

  /// Returns true if the entity is a publisher or client. False otherwise.
  static bool is_entity_pub(const liveliness::Entity & entity);

private:
  // Helper function to convert an Entity into a GraphNode.
  // Note: this will update bookkeeping variables in GraphCache.
  std::shared_ptr<GraphNode> make_graph_node(const liveliness::Entity & entity) const;

  // Helper function to update TopicMap within the node the cache for the entire graph.
  void update_topic_maps_for_put(
    GraphNodePtr graph_node,
    liveliness::ConstEntityPtr entity);

  void update_topic_map_for_put(
    GraphNode::TopicMap & topic_map,
    liveliness::ConstEntityPtr entity,
    bool report_events = false);

  void update_topic_maps_for_del(
    GraphNodePtr graph_node,
    liveliness::ConstEntityPtr entity);

  void update_topic_map_for_del(
    GraphNode::TopicMap & topic_map,
    liveliness::ConstEntityPtr entity,
    bool report_events = false);

  void remove_topic_map_from_cache(
    const GraphNode::TopicMap & to_remove,
    GraphNode::TopicMap & from_cache);

  /// Returns true if the entity was created within the same context / zenoh session.
  bool is_entity_local(const liveliness::Entity & entity) const;

  void update_event_counters(
    liveliness::ConstEntityPtr entity,
    const rmw_zenoh_event_type_t event_id,
    int32_t change);

  void handle_matched_events_for_put(
    liveliness::ConstEntityPtr entity,
    const GraphNode::TopicQoSMap & topic_qos_map);

  void handle_matched_events_for_del(
    liveliness::ConstEntityPtr entity,
    const GraphNode::TopicQoSMap & topic_qos_map);

  std::string zid_str_;
  /*
  namespace_1:
    node_1:
      enclave:
      publishers: [
          {
            topic:
            type:
            qos:
          }
      ],
      subscriptions: [
          {
            topic:
            type:
            qos:
          }
      ],
  namespace_2:
    node_n:
  */

  // We rely on a multimap to store nodes with duplicate names.
  using NodeMap = std::multimap<std::string, GraphNodePtr>;
  using NamespaceMap = std::unordered_map<std::string, NodeMap>;
  // Map namespace to a map of <node_name, GraphNodePtr>.
  NamespaceMap graph_ = {};
  size_t total_nodes_in_graph_{0};

  // Optimize pub/sub lookups across the graph.
  GraphNode::TopicMap graph_topics_ = {};
  // Optimize service/client lookups across the graph.
  GraphNode::TopicMap graph_services_ = {};

  using GraphEventCallbacks = std::unordered_map<rmw_zenoh_event_type_t, GraphCacheEventCallback>;
  // Map an entity's keyexpr_hash to a map of event callbacks.
  // Note: Since we use unordered_map, we will only store a single callback for an
  // entity string. So we do not support the case where a node create a duplicate
  // pub/sub with the exact same topic, type & QoS but registers a different callback
  // for the same event type. We could switch to a multimap here but removing the callback
  // will be impossible right now since entities do not have unique IDs.
  using GraphEventCallbackMap = std::unordered_map<std::size_t, GraphEventCallbacks>;
  // EventCallbackMap for each type of event we support in rmw_zenoh_cpp.
  GraphEventCallbackMap event_callbacks_;
  std::mutex events_mutex_;

  // Mutex to lock before modifying the members above.
  mutable std::mutex graph_mutex_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__GRAPH_CACHE_HPP_
