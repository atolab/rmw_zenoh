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

#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/find_and_replace.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/sanity_checks.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "rosidl_runtime_c/type_hash.h"

#include "graph_cache.hpp"
#include "logging_macros.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
using Entity = liveliness::Entity;
using ConstEntityPtr = liveliness::ConstEntityPtr;
using EntityType = liveliness::EntityType;
///=============================================================================
TopicDataPtr TopicData::make(ConstEntityPtr entity)
{
  if (!entity->topic_info().has_value()) {
    return nullptr;
  }

  std::shared_ptr<TopicData> topic_data(new TopicData{entity});

  return topic_data;
}

///=============================================================================
TopicData::TopicData(ConstEntityPtr entity)
: info_(entity->topic_info().value()),
  pubs_({}),
  subs_({})
{
  if (GraphCache::is_entity_pub(*entity)) {
    pubs_.emplace(std::move(entity));
  } else {
    subs_.emplace(std::move(entity));
  }
}

///=============================================================================
GraphCache::GraphCache(const zenoh::Id & zid)
: zid_str_(zid.to_string())
{
  // Do nothing.
}

///=============================================================================
std::shared_ptr<GraphNode> GraphCache::make_graph_node(const Entity & entity) const
{
  auto graph_node = std::make_shared<GraphNode>();
  graph_node->zid_ = entity.zid();
  graph_node->nid_ = entity.nid();
  graph_node->ns_ = entity.node_namespace();
  graph_node->name_ = entity.node_name();
  graph_node->enclave_ = entity.node_enclave();

  return graph_node;
}

///=============================================================================
void GraphCache::update_topic_maps_for_put(
  GraphNodePtr graph_node,
  liveliness::ConstEntityPtr entity)
{
  if (entity->type() == EntityType::Node) {
    // Nothing to update for a node entity.
    return;
  }

  // First update the topic map within the node.
  if (entity->type() == EntityType::Publisher) {
    update_topic_map_for_put(graph_node->pubs_, entity);
  } else if (entity->type() == EntityType::Subscription) {
    update_topic_map_for_put(graph_node->subs_, entity);
  } else if (entity->type() == EntityType::Service) {
    update_topic_map_for_put(graph_node->services_, entity);
  } else {
    update_topic_map_for_put(graph_node->clients_, entity);
  }

  // Then update the variables tracking topics across the graph.
  // We invoke update_topic_map_for_put() with report_events set to true for
  // pub/sub.
  if (entity->type() == EntityType::Publisher ||
    entity->type() == EntityType::Subscription)
  {
    update_topic_map_for_put(this->graph_topics_, entity, true);
  } else {
    update_topic_map_for_put(this->graph_services_, entity);
  }
}

///=============================================================================
void GraphCache::update_topic_map_for_put(
  GraphNode::TopicMap & topic_map,
  liveliness::ConstEntityPtr entity,
  bool report_events)
{
  TopicDataPtr graph_topic_data = TopicData::make(entity);
  if (graph_topic_data == nullptr) {
    // This should not happen as topic_info should be populated for all non-node entities.
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "update_topic_map_for_put() called for non-node entity without valid TopicInfo. "
      "Report this.");
    return;
  }

  // For the sake of reusing data structures and lookup functions, we treat publishers and
  // clients as equivalent. Similarly, subscriptions and services are equivalent.
  const bool is_pub = is_entity_pub(*entity);
  const std::string qos_str = liveliness::qos_to_keyexpr(graph_topic_data->info_.qos_);
  GraphNode::TopicQoSMap topic_qos_map = {
    {qos_str, graph_topic_data}};
  GraphNode::TopicMap::iterator topic_map_it = topic_map.find(graph_topic_data->info_.name_);

  if (topic_map_it == topic_map.end()) {
    // First time this topic name is discovered in the topic_map so we insert a TopicTypeMap.
    GraphNode::TopicTypeMap topic_data_map = {
      {graph_topic_data->info_.type_, std::move(topic_qos_map)}
    };
    topic_map.insert(std::make_pair(graph_topic_data->info_.name_, std::move(topic_data_map)));
    // We do not need to check for events since this is the first time and entiry for this topic
    // was added to the topic map.
    return;
  }
  // The topic exists in the topic_map so we check if the type also exists.
  GraphNode::TopicTypeMap::iterator topic_type_map_it = topic_map_it.value().find(
    graph_topic_data->info_.type_);
  if (topic_type_map_it == topic_map_it->second.end()) {
    // First time this topic type is added.
    topic_map_it.value().insert(
      std::make_pair(
        graph_topic_data->info_.type_,
        std::move(topic_qos_map)));
    // TODO(Yadunund) Check for and report an *_INCOMPATIBLE_TYPE events.
    return;
  }
  // The topic type already exists.
  if (report_events) {
    // With Zenoh, as long as topic name and type match, transport will ensure
    // payloads are received by subs. Hence, we can check for matched events
    // without having to check for any qos compatibilities.
    this->handle_matched_events_for_put(
      entity,
      topic_type_map_it->second);
  }
  // We check if an entity with the exact same qos also exists.
  GraphNode::TopicQoSMap::iterator topic_qos_map_it =
    topic_type_map_it->second.find(qos_str);
  if (topic_qos_map_it == topic_type_map_it->second.end()) {
    // First time this qos is added.
    // Update cache.
    topic_type_map_it->second.insert(std::make_pair(qos_str, graph_topic_data));
  } else {
    // We have another instance of a pub/sub over the same topic,
    // type and qos so we increment the counters.
    TopicDataPtr & existing_graph_topic = topic_qos_map_it->second;
    if (is_pub) {
      existing_graph_topic->pubs_.insert(entity);
    } else {
      existing_graph_topic->subs_.insert(entity);
    }
  }
}

///=============================================================================
void GraphCache::handle_matched_events_for_put(
  liveliness::ConstEntityPtr entity,
  const GraphNode::TopicQoSMap & topic_qos_map)
{
  if (!entity->topic_info().has_value()) {
    return;
  }
  const liveliness::TopicInfo topic_info = entity->topic_info().value();
  const bool is_pub = is_entity_pub(*entity);
  // The entity added may be local with callbacks registered but there
  // may be other local entities in the graph that are matched.
  int32_t match_count_for_entity = 0;
  for (const auto & [_, topic_data_ptr] : topic_qos_map) {
    if (is_pub) {
      // Count the number of matching subs for each set of qos settings.
      std::size_t sub_size = topic_data_ptr->subs_.size();
      if (sub_size > std::numeric_limits<int32_t>::max()) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Too many subscriptions on publisher; assuming 0. Report this bug.");
        sub_size = 0;
      }
      match_count_for_entity += static_cast<int32_t>(sub_size);
      // Also iterate through the subs to check if any are local and if update event counters.
      for (liveliness::ConstEntityPtr sub_entity : topic_data_ptr->subs_) {
        // Update counters only if key expressions match.
        if (entity->topic_info()->topic_keyexpr_ ==
          sub_entity->topic_info().value().topic_keyexpr_)
        {
          if (is_entity_local(*sub_entity)) {
            update_event_counters(sub_entity, ZENOH_EVENT_SUBSCRIPTION_MATCHED, 1);
          }
        }
      }
      // Update event counters for the new entity.
      if (is_entity_local(*entity) && match_count_for_entity > 0) {
        update_event_counters(entity,
          ZENOH_EVENT_PUBLICATION_MATCHED,
          match_count_for_entity);
      }
    } else {
      // Entity is a sub.
      // Count the number of matching pubs for each set of qos settings.
      std::size_t pub_size = topic_data_ptr->pubs_.size();
      if (pub_size > std::numeric_limits<int32_t>::max()) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Too many publishers on subscription; assuming 0. Report this bug.");
        pub_size = 0;
      }
      match_count_for_entity += static_cast<int32_t>(pub_size);
      // Also iterate through the pubs to check if any are local and if update event counters.
      for (liveliness::ConstEntityPtr pub_entity : topic_data_ptr->pubs_) {
        // Update counters only if key expressions match.
        if (entity->topic_info()->topic_keyexpr_ ==
          pub_entity->topic_info().value().topic_keyexpr_)
        {
          if (is_entity_local(*pub_entity)) {
            update_event_counters(pub_entity, ZENOH_EVENT_PUBLICATION_MATCHED, 1);
          }
        }
      }
      // Update event counters for the new entity.
      if (is_entity_local(*entity) && match_count_for_entity > 0) {
        update_event_counters(
          entity,
          ZENOH_EVENT_SUBSCRIPTION_MATCHED,
          match_count_for_entity);
      }
    }
  }
}

///=============================================================================
void GraphCache::handle_matched_events_for_del(
  liveliness::ConstEntityPtr entity,
  const GraphNode::TopicQoSMap & topic_qos_map)
{
  // We do not have to report any events for the entity removed event
  // as it is already destructed. So we only check for matched entities
  // in the graph that may be local.
  if (!entity->topic_info().has_value()) {
    return;
  }
  const liveliness::TopicInfo topic_info = entity->topic_info().value();
  if (is_entity_pub(*entity)) {
    // Notify any local subs of a matched event with change -1.
    for (const auto & [_, topic_data_ptr] : topic_qos_map) {
      for (liveliness::ConstEntityPtr sub_entity : topic_data_ptr->subs_) {
        if (is_entity_local(*sub_entity)) {
          update_event_counters(
            sub_entity,
            ZENOH_EVENT_SUBSCRIPTION_MATCHED,
            static_cast<int32_t>(-1));
        }
      }
    }
  } else {
    // Notify any local pubs of a matched event with change -1.
    for (const auto & [_, topic_data_ptr] : topic_qos_map) {
      for (liveliness::ConstEntityPtr pub_entity : topic_data_ptr->pubs_) {
        if (is_entity_local(*pub_entity)) {
          update_event_counters(
            pub_entity,
            ZENOH_EVENT_PUBLICATION_MATCHED,
            static_cast<int32_t>(-1));
        }
      }
    }
  }
}

///=============================================================================
void GraphCache::parse_put(
  const std::string & keyexpr,
  bool ignore_from_current_session)
{
  liveliness::ConstEntityPtr entity = liveliness::Entity::make(keyexpr);
  if (entity == nullptr) {
    // Error message has already been logged.
    return;
  }
  if (ignore_from_current_session && is_entity_local(*entity)) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Ignoring parse_put for %s from the same session.\n", entity->liveliness_keyexpr().c_str());
    return;
  }

  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If the namespace did not exist, create it and add the node to the graph and return.
  NamespaceMap::iterator ns_it = graph_.find(entity->node_namespace());
  if (ns_it == graph_.end()) {
    GraphNodePtr node = make_graph_node(*entity);
    if (node == nullptr) {
      // Error handled.
      return;
    }
    NodeMap node_map = {
      {entity->node_name(), node}};
    graph_.emplace(std::make_pair(entity->node_namespace(), std::move(node_map)));
    update_topic_maps_for_put(node, entity);
    total_nodes_in_graph_ += 1;
    return;
  }

  // Add the node to the namespace if it did not exist and return.
  // Case 1: First time a node with this name is added to the namespace.
  // Case 2: There are one or more nodes with the same name but the entity could
  // represent a node with the same name but a unique id which would make it a
  // new addition to the graph.
  std::pair<NodeMap::iterator, NodeMap::iterator> range = ns_it->second.equal_range(
    entity->node_name());
  NodeMap::iterator node_it = std::find_if(
    range.first, range.second,
    [entity](const std::pair<std::string, GraphNodePtr> & node_it)
    {
      // Match nodes if their zenoh session and node ids match.
      return entity->zid() == node_it.second->zid_ && entity->nid() == node_it.second->nid_;
    });
  if (node_it == range.second) {
    // Either the first time a node with this name is added or with an existing
    // name but unique id.
    GraphNodePtr node = make_graph_node(*entity);
    if (node == nullptr) {
      // Error handled.
      return;
    }
    NodeMap::iterator insertion_it =
      ns_it->second.insert(std::make_pair(entity->node_name(), node));
    update_topic_maps_for_put(node, entity);
    total_nodes_in_graph_ += 1;
    if (insertion_it == ns_it->second.end()) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to add a new node /%s to an "
        "existing namespace %s in the graph. Report this bug.",
        entity->node_name().c_str(),
        entity->node_namespace().c_str());
    }
    return;
  }
  // Otherwise, the entity represents a node that already exists in the graph.
  // Update topic info if required below.
  update_topic_maps_for_put(node_it->second, entity);

  // If the newly added entity is a publisher with transient_local qos durability,
  // we trigger any registered querying subscriber callbacks.
  if (entity->type() == liveliness::EntityType::Publisher &&
    entity->topic_info().has_value() &&
    entity->topic_info()->qos_.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
  {
    auto sub_cbs_it = querying_subs_cbs_.find(entity->topic_info()->topic_keyexpr_);
    if (sub_cbs_it != querying_subs_cbs_.end()) {
      for (auto sub_it = sub_cbs_it->second.begin(); sub_it != sub_cbs_it->second.end(); ++sub_it) {
        sub_it->second(entity->zid());
      }
    }
  }
}

///=============================================================================
void GraphCache::update_topic_maps_for_del(
  GraphNodePtr graph_node,
  liveliness::ConstEntityPtr entity)
{
  if (entity->type() == EntityType::Node) {
    // Nothing to update for a node entity->
    return;
  }
  // First update the topic map within the node.
  if (entity->type() == EntityType::Publisher) {
    update_topic_map_for_del(graph_node->pubs_, entity);
  } else if (entity->type() == EntityType::Subscription) {
    update_topic_map_for_del(graph_node->subs_, entity);
  } else if (entity->type() == EntityType::Service) {
    update_topic_map_for_del(graph_node->services_, entity);
  } else {
    update_topic_map_for_del(graph_node->clients_, entity);
  }

  // Then update the variables tracking topics across the graph.
  if (entity->type() == EntityType::Publisher ||
    entity->type() == EntityType::Subscription)
  {
    update_topic_map_for_del(this->graph_topics_, entity, true);
  } else {
    update_topic_map_for_del(this->graph_services_, entity);
  }
}

///=============================================================================
void GraphCache::update_topic_map_for_del(
  GraphNode::TopicMap & topic_map,
  liveliness::ConstEntityPtr entity,
  bool report_events)
{
  if (!entity->topic_info().has_value()) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "update_topic_maps_for_del() called for non-node entity without valid TopicInfo. "
      "Report this.");
    return;
  }
  const liveliness::TopicInfo topic_info = entity->topic_info().value();
  const bool is_pub = is_entity_pub(*entity);

  GraphNode::TopicMap::iterator cache_topic_it =
    topic_map.find(topic_info.name_);
  if (cache_topic_it == topic_map.end()) {
    // This should not happen.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "topic name %s not found in topic_map. Report this.",
      topic_info.name_.c_str());
    return;
  }

  GraphNode::TopicTypeMap::iterator cache_topic_type_it =
    cache_topic_it.value().find(topic_info.type_);
  if (cache_topic_type_it == cache_topic_it->second.end()) {
    // This should not happen.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "topic type %s not found in for topic %s. Report this.",
      topic_info.type_.c_str(), topic_info.name_.c_str());
    return;
  }

  const std::string qos_str = liveliness::qos_to_keyexpr(topic_info.qos_);
  GraphNode::TopicQoSMap::iterator cache_topic_qos_it = cache_topic_type_it->second.find(
    qos_str);
  if (cache_topic_qos_it == cache_topic_type_it->second.end()) {
    // This should not happen.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "qos %s not found in for topic type %s. Report this.",
      qos_str.c_str(), topic_info.type_.c_str());
    return;
  }

  // Decrement the relevant counters. If both counters are 0 remove from cache.
  if (is_pub) {
    cache_topic_qos_it->second->pubs_.erase(entity);
  } else {
    cache_topic_qos_it->second->subs_.erase(entity);
  }
  // If after removing the entity, the parent map is empty, then remove parent
  // map.
  if (cache_topic_qos_it->second->pubs_.empty() &&
    cache_topic_qos_it->second->subs_.empty())
  {
    cache_topic_type_it->second.erase(qos_str);
  }
  // Check for matched events
  if (report_events) {
    handle_matched_events_for_del(
      entity,
      cache_topic_type_it->second);
  }
  // If the type does not have any qos entries, erase it from the type map.
  if (cache_topic_type_it->second.empty()) {
    cache_topic_it.value().erase(cache_topic_type_it);
  }
  // If the topic does not have any TopicData entries, erase the topic from the map.
  if (cache_topic_it->second.empty()) {
    topic_map.erase(cache_topic_it);
  }
}

///=============================================================================
void GraphCache::remove_topic_map_from_cache(
  const GraphNode::TopicMap & to_remove,
  GraphNode::TopicMap & from_cache)
{
  for (GraphNode::TopicMap::const_iterator topic_it = to_remove.begin();
    topic_it != to_remove.end(); ++topic_it)
  {
    for (GraphNode::TopicTypeMap::const_iterator topic_type_it = topic_it->second.begin();
      topic_type_it != topic_it->second.end(); ++topic_type_it)
    {
      for (GraphNode::TopicQoSMap::const_iterator topic_qos_it =
        topic_type_it->second.begin();
        topic_qos_it != topic_type_it->second.end(); ++topic_qos_it)
      {
        // Technically only one of pubs_ or sub_ will be populated and with one
        // element at most since to_remove comes from a node. For completeness,
        // we iterate though both and call update_topic_map_for_del().
        for (const liveliness::ConstEntityPtr & entity : topic_qos_it->second->pubs_) {
          update_topic_map_for_del(
            from_cache,
            entity,
            true);
        }
        for (const liveliness::ConstEntityPtr & entity : topic_qos_it->second->subs_) {
          update_topic_map_for_del(
            from_cache,
            entity,
            true);
        }
      }
    }
  }
}

///=============================================================================
void GraphCache::parse_del(
  const std::string & keyexpr,
  bool ignore_from_current_session)
{
  liveliness::ConstEntityPtr entity = liveliness::Entity::make(keyexpr);
  if (entity == nullptr) {
    // Error message has already been logged.
    return;
  }
  if (ignore_from_current_session && is_entity_local(*entity)) {
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Ignoring parse_del for %s from the same session.\n", entity->liveliness_keyexpr().c_str());
    return;
  }
  // Lock the graph mutex before accessing the graph.
  std::lock_guard<std::mutex> lock(graph_mutex_);

  // If namespace does not exist, ignore the request.
  NamespaceMap::iterator ns_it = graph_.find(entity->node_namespace());
  if (ns_it == graph_.end()) {
    return;
  }

  // If the node does not exist, ignore the request.
  std::pair<NodeMap::iterator, NodeMap::iterator> range = ns_it->second.equal_range(
    entity->node_name());
  NodeMap::iterator node_it = std::find_if(
    range.first, range.second,
    [entity](const std::pair<std::string, GraphNodePtr> & node_it)
    {
      // Match nodes if their zenoh session and node ids match.
      return entity->zid() == node_it.second->zid_ && entity->nid() == node_it.second->nid_;
    });
  if (node_it == range.second) {
    // Node does not exist.
    RMW_ZENOH_LOG_DEBUG_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token to remove unknown node /%s from the graph. Ignoring...",
      entity->node_name().c_str()
    );
    return;
  }

  if (entity->type() == EntityType::Node) {
    // Node
    // When destroying a node, Zenoh does not guarantee that liveliness tokens to remove pub/subs
    // arrive before the one to remove the node from the graph despite un-registering those entities
    // earlier. In such scenarios, if we find any pub/subs present in the node, we reduce their
    // counts in graph_topics_.
    const GraphNodePtr graph_node = node_it->second;
    if (!graph_node->pubs_.empty() ||
      !graph_node->subs_.empty() ||
      !graph_node->clients_.empty() ||
      !graph_node->services_.empty())
    {
      RMW_ZENOH_LOG_DEBUG_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token to remove node /%s from the graph before all pub/subs/"
        "clients/services for this node have been removed. Removing all entities first...",
        entity->node_name().c_str()
      );
      // We update the tracking variables to reduce the count of entities present in this node.
      remove_topic_map_from_cache(graph_node->pubs_, graph_topics_);
      remove_topic_map_from_cache(graph_node->subs_, graph_topics_);
      remove_topic_map_from_cache(graph_node->services_, graph_services_);
      remove_topic_map_from_cache(graph_node->clients_, graph_services_);
    }
    ns_it->second.erase(node_it);
    total_nodes_in_graph_ -= 1;
    if (ns_it->second.size() == 0) {
      graph_.erase(entity->node_namespace());
    }
    return;
  }

  // Update the graph based on the entity->
  update_topic_maps_for_del(node_it->second, entity);
}

///=============================================================================
rmw_ret_t GraphCache::get_node_names(
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves,
  rcutils_allocator_t * allocator) const
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_names)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_RET_OK != rmw_check_zero_rmw_string_array(node_namespaces)) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (
    enclaves &&
    RMW_RET_OK != rmw_check_zero_rmw_string_array(enclaves))
  {
    return RMW_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  rcutils_ret_t rcutils_ret =
    rcutils_string_array_init(node_names, total_nodes_in_graph_, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    return RMW_RET_BAD_ALLOC;
  }
  auto free_node_names = rcpputils::make_scope_exit(
    [node_names]() {
      rcutils_ret_t ret = rcutils_string_array_fini(node_names);
      if (ret != RCUTILS_RET_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    });

  rcutils_ret =
    rcutils_string_array_init(node_namespaces, total_nodes_in_graph_, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    return RMW_RET_BAD_ALLOC;
  }
  auto free_node_namespaces = rcpputils::make_scope_exit(
    [node_namespaces]() {
      rcutils_ret_t ret = rcutils_string_array_fini(node_namespaces);
      if (ret != RCUTILS_RET_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    });

  auto free_enclaves_lambda = [enclaves]() -> void {
      rcutils_ret_t ret = rcutils_string_array_fini(enclaves);
      if (ret != RCUTILS_RET_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
      }
    };

  std::shared_ptr<rcpputils::scope_exit<decltype(free_enclaves_lambda)>> free_enclaves{nullptr};
  if (enclaves) {
    rcutils_ret =
      rcutils_string_array_init(enclaves, total_nodes_in_graph_, allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      return RMW_RET_BAD_ALLOC;
    }
    free_enclaves =
      std::make_shared<rcpputils::scope_exit<decltype(free_enclaves_lambda)>>(
      std::move(free_enclaves_lambda));
  }

  // Fill node names, namespaces and enclaves.
  std::size_t j = 0;
  for (NamespaceMap::const_iterator ns_it = graph_.begin(); ns_it != graph_.end(); ++ns_it) {
    const std::string & ns = ns_it->first;
    for (NodeMap::const_iterator node_it = ns_it->second.begin(); node_it != ns_it->second.end();
      ++node_it)
    {
      const GraphNodePtr node = node_it->second;
      node_names->data[j] = rcutils_strdup(node->name_.c_str(), *allocator);
      if (!node_names->data[j]) {
        return RMW_RET_BAD_ALLOC;
      }
      node_namespaces->data[j] = rcutils_strdup(
        ns.c_str(), *allocator);
      if (!node_namespaces->data[j]) {
        return RMW_RET_BAD_ALLOC;
      }
      if (enclaves) {
        enclaves->data[j] = rcutils_strdup(
          node->enclave_.c_str(), *allocator);
        if (!enclaves->data[j]) {
          return RMW_RET_BAD_ALLOC;
        }
      }
      ++j;
    }
  }

  if (free_enclaves) {
    free_enclaves->cancel();
  }
  free_node_namespaces.cancel();
  free_node_names.cancel();

  return RMW_RET_OK;
}

///=============================================================================
namespace
{
// Shamelessly copied from https://github.com/ros2/rmw_cyclonedds/blob/f7f67bdef82f59558366aa6ce94ef9af3c5ab569/rmw_cyclonedds_cpp/src/demangle.cpp#L67
std::string
_demangle_if_ros_type(const std::string & dds_type_string)
{
  if (dds_type_string[dds_type_string.size() - 1] != '_') {
    // not a ROS type
    return dds_type_string;
  }

  std::string substring = "dds_::";
  size_t substring_position = dds_type_string.find(substring);
  if (substring_position == std::string::npos) {
    // not a ROS type
    return dds_type_string;
  }

  std::string type_namespace = dds_type_string.substr(0, substring_position);
  type_namespace = rcpputils::find_and_replace(type_namespace, "::", "/");
  size_t start = substring_position + substring.size();
  std::string type_name = dds_type_string.substr(start, dds_type_string.length() - 1 - start);
  return type_namespace + type_name;
}

rmw_ret_t fill_names_and_types(
  const GraphNode::TopicMap & entity_map,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * names_and_types)
{
  const std::size_t entity_size = entity_map.size();
  rmw_ret_t ret = rmw_names_and_types_init(names_and_types, entity_size, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  auto cleanup_names_and_types = rcpputils::make_scope_exit(
    [names_and_types] {
      rmw_ret_t fail_ret = rmw_names_and_types_fini(names_and_types);
      if (fail_ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("failed to cleanup names and types during error handling");
      }
    });
  // Fill topic names and types.
  std::size_t index = 0;
  for (const std::pair<std::string, GraphNode::TopicTypeMap> & item : entity_map) {
    names_and_types->names.data[index] = rcutils_strdup(item.first.c_str(), *allocator);
    if (!names_and_types->names.data[index]) {
      return RMW_RET_BAD_ALLOC;
    }

    rcutils_ret_t rcutils_ret = rcutils_string_array_init(
      &names_and_types->types[index],
      item.second.size(),
      allocator);
    if (RCUTILS_RET_OK != rcutils_ret) {
      RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
      return RMW_RET_BAD_ALLOC;
    }

    size_t type_index = 0;
    for (const std::pair<const std::string, GraphNode::TopicQoSMap> & type : item.second) {
      char * type_name = rcutils_strdup(_demangle_if_ros_type(type.first).c_str(), *allocator);
      if (!type_name) {
        RMW_SET_ERROR_MSG("failed to allocate memory for type name");
        return RMW_RET_BAD_ALLOC;
      }
      names_and_types->types[index].data[type_index] = type_name;
      ++type_index;
    }
    ++index;
  }

  cleanup_names_and_types.cancel();

  return RMW_RET_OK;
}
}  // namespace

///=============================================================================
rmw_ret_t GraphCache::get_topic_names_and_types(
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types) const
{
  static_cast<void>(no_demangle);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  std::lock_guard<std::mutex> lock(graph_mutex_);
  return fill_names_and_types(graph_topics_, allocator, topic_names_and_types);
}

///=============================================================================
rmw_ret_t GraphCache::publisher_count_matched_subscriptions(
  const liveliness::TopicInfo & pub_topic_info,
  size_t * subscription_count)
{
  // TODO(Yadunund): Replace this logic by returning a number that is tracked once
  // we support matched qos events.
  *subscription_count = 0;
  GraphNode::TopicMap::const_iterator topic_it = graph_topics_.find(pub_topic_info.name_);
  if (topic_it != graph_topics_.end()) {
    GraphNode::TopicTypeMap::const_iterator topic_data_it = topic_it->second.find(
      pub_topic_info.type_);
    if (topic_data_it != topic_it->second.end()) {
      for (const auto & [_, topic_data]  : topic_data_it->second) {
        // If a subscription exists with compatible QoS, update the subscription count.
        if (!topic_data->subs_.empty()) {
          rmw_qos_compatibility_type_t is_compatible;
          rmw_ret_t ret = rmw_qos_profile_check_compatible(
            pub_topic_info.qos_,
            topic_data->info_.qos_,
            &is_compatible,
            nullptr,
            0);
          if (ret == RMW_RET_OK && is_compatible != RMW_QOS_COMPATIBILITY_ERROR) {
            *subscription_count = *subscription_count + topic_data->subs_.size();
          }
        }
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::subscription_count_matched_publishers(
  const liveliness::TopicInfo & sub_topic_info,
  size_t * publisher_count)
{
  // TODO(Yadunund): Replace this logic by returning a number that is tracked once
  // we support matched qos events.
  *publisher_count = 0;
  GraphNode::TopicMap::const_iterator topic_it = graph_topics_.find(sub_topic_info.name_);
  if (topic_it != graph_topics_.end()) {
    GraphNode::TopicTypeMap::const_iterator topic_data_it = topic_it->second.find(
      sub_topic_info.type_);
    if (topic_data_it != topic_it->second.end()) {
      for (const auto & [_, topic_data]  : topic_data_it->second) {
        // If a subscription exists with compatible QoS, update the subscription count.
        if (!topic_data->pubs_.empty()) {
          rmw_qos_compatibility_type_t is_compatible;
          rmw_ret_t ret = rmw_qos_profile_check_compatible(
            sub_topic_info.qos_,
            topic_data->info_.qos_,
            &is_compatible,
            nullptr,
            0);
          if (ret == RMW_RET_OK && is_compatible != RMW_QOS_COMPATIBILITY_ERROR) {
            *publisher_count = *publisher_count + topic_data->pubs_.size();
          }
        }
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::get_service_names_and_types(
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types) const
{
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  std::lock_guard<std::mutex> lock(graph_mutex_);
  return fill_names_and_types(graph_services_, allocator, service_names_and_types);
}

///=============================================================================
rmw_ret_t GraphCache::count_publishers(
  const char * topic_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_topics_.count(topic_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_topics_.at(topic_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->pubs_.size();
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::count_subscriptions(
  const char * topic_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_topics_.count(topic_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_topics_.at(topic_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->subs_.size();
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::count_services(
  const char * service_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_services_.count(service_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_services_.at(service_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->subs_.size();
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::count_clients(
  const char * service_name,
  size_t * count) const
{
  *count = 0;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (graph_services_.count(service_name) != 0) {
    // Iterate through all the types and increment count.
    for (const std::pair<const std::string,
      GraphNode::TopicQoSMap> & topic_data : graph_services_.at(service_name))
    {
      for (auto it = topic_data.second.begin(); it != topic_data.second.end(); ++it) {
        *count += it->second->pubs_.size();
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::get_entity_names_and_types_by_node(
  liveliness::EntityType entity_type,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * names_and_types) const
{
  static_cast<void>(no_demangle);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);
  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(node_name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("node_name argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(node_namespace, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return ret;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_namespace_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("node_namespace argument is invalid: %s", reason);
    return RMW_RET_INVALID_ARGUMENT;
  }
  ret = rmw_names_and_types_check_zero(names_and_types);
  if (RMW_RET_OK != ret) {
    return ret;
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);

  // Check if namespace exists.
  NamespaceMap::const_iterator ns_it = graph_.find(node_namespace);
  if (ns_it == graph_.end()) {
    return RMW_RET_NODE_NAME_NON_EXISTENT;
  }

  // Check if node exists.
  // Since NodeMap is a multimap, this will return the first node with the same
  // name that is found.
  NodeMap::const_iterator node_it = ns_it->second.find(node_name);
  if (node_it == ns_it->second.end()) {
    return RMW_RET_NODE_NAME_NON_EXISTENT;
  }

  // TODO(Yadunund): Support service and client when ready.
  if (entity_type == EntityType::Publisher) {
    return fill_names_and_types(node_it->second->pubs_, allocator, names_and_types);
  } else if (entity_type == EntityType::Subscription) {
    return fill_names_and_types(node_it->second->subs_, allocator, names_and_types);
  } else if (entity_type == EntityType::Service) {
    return fill_names_and_types(node_it->second->services_, allocator, names_and_types);
  } else if (entity_type == EntityType::Client) {
    return fill_names_and_types(node_it->second->clients_, allocator, names_and_types);
  } else {
    return RMW_RET_UNSUPPORTED;
  }
}


///=============================================================================
rmw_ret_t GraphCache::get_entities_info_by_topic(
  liveliness::EntityType entity_type,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_demangle,
  rmw_topic_endpoint_info_array_t * endpoints_info) const
{
  static_cast<void>(no_demangle);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator argument is invalid", return RMW_RET_INVALID_ARGUMENT);

  if (entity_type != EntityType::Publisher && entity_type != EntityType::Subscription) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);

  GraphNode::TopicMap::const_iterator topic_it = graph_topics_.find(topic_name);
  // Exit early if the topic does not exist in the graph.
  if (topic_it == graph_topics_.end()) {
    return RMW_RET_OK;
  }

  std::vector<rmw_topic_endpoint_info_t> endpoints;
  for (const auto & [topic_type_name, topic_qos_map] : topic_it->second) {
    for (const auto & [_, topic_data] : topic_qos_map) {
      const TopicData::EntitySet & entity_set =
        entity_type == EntityType::Publisher ? topic_data->pubs_ :
        topic_data->subs_;
      for (const liveliness::ConstEntityPtr & entity : entity_set) {
        rmw_topic_endpoint_info_t ep = rmw_get_zero_initialized_topic_endpoint_info();

        rmw_ret_t ret;

        ret = rmw_topic_endpoint_info_set_node_name(&ep, entity->node_name().c_str(), allocator);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_node_namespace(
          &ep, entity->node_namespace().c_str(), allocator);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_topic_type(
          &ep, _demangle_if_ros_type(topic_type_name).c_str(), allocator);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_endpoint_type(
          &ep,
          entity_type ==
          EntityType::Publisher ? RMW_ENDPOINT_PUBLISHER : RMW_ENDPOINT_SUBSCRIPTION);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        ret = rmw_topic_endpoint_info_set_qos_profile(&ep, &topic_data->info_.qos_);
        if (RMW_RET_OK != ret) {
          return ret;
        }

        rosidl_type_hash_t type_hash;
        rcutils_ret_t rc_ret = rosidl_parse_type_hash_string(
          topic_data->info_.type_hash_.c_str(),
          &type_hash);
        if (RCUTILS_RET_OK == rc_ret) {
          ret = rmw_topic_endpoint_info_set_topic_type_hash(&ep, &type_hash);
          if (RMW_RET_OK != ret) {
            return ret;
          }
        }

        memcpy(ep.endpoint_gid, entity->copy_gid().data(), RMW_GID_STORAGE_SIZE);

        endpoints.push_back(ep);
      }
    }
  }

  rmw_ret_t ret = rmw_topic_endpoint_info_array_init_with_size(
    endpoints_info, endpoints.size(), allocator);
  if (RMW_RET_OK != ret) {
    return ret;
  }

  memcpy(
    endpoints_info->info_array, &endpoints[0],
    sizeof(rmw_topic_endpoint_info_t) * endpoints.size());

  return RMW_RET_OK;
}

///=============================================================================
rmw_ret_t GraphCache::service_server_is_available(
  const liveliness::TopicInfo & client_topic_info,
  bool * is_available) const
{
  *is_available = false;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  GraphNode::TopicMap::const_iterator service_it = graph_services_.find(client_topic_info.name_);
  if (service_it != graph_services_.end()) {
    GraphNode::TopicTypeMap::const_iterator type_it =
      service_it->second.find(client_topic_info.type_);
    if (type_it != service_it->second.end()) {
      for (const auto & [_, topic_data] : type_it->second) {
        if (!topic_data->subs_.empty()) {
          *is_available = true;
          return RMW_RET_OK;
        }
      }
    }
  }

  return RMW_RET_OK;
}

///=============================================================================
void GraphCache::set_qos_event_callback(
  std::size_t entity_keyexpr_hash,
  const rmw_zenoh_event_type_t & event_type,
  GraphCacheEventCallback callback)
{
  std::lock_guard<std::mutex> lock(events_mutex_);

  if (event_type > ZENOH_EVENT_ID_MAX) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "set_qos_event_callback() called for unsupported event. Report this.");
    return;
  }

  const GraphEventCallbackMap::iterator event_cb_it = event_callbacks_.find(entity_keyexpr_hash);
  if (event_cb_it == event_callbacks_.end()) {
    // First time a callback is being set for this entity.
    event_callbacks_[entity_keyexpr_hash] = {std::make_pair(event_type, std::move(callback))};
    return;
  }
  event_cb_it->second[event_type] = std::move(callback);
}

///=============================================================================
void GraphCache::remove_qos_event_callbacks(std::size_t entity_keyexpr_hash)
{
  std::lock_guard<std::mutex> lock(events_mutex_);
  event_callbacks_.erase(entity_keyexpr_hash);
}

///=============================================================================
bool GraphCache::is_entity_local(const liveliness::Entity & entity) const
{
  // For now zenoh does not expose unique IDs for its entities and hence the id
  // assigned to an entity is always the zenoh session id. When we update liveliness
  // tokens to contain globally unique ids for entities, we should also update the logic here.
  return entity.zid() == zid_str_;
}

///=============================================================================
bool GraphCache::is_entity_pub(const liveliness::Entity & entity)
{
  if (entity.type() == EntityType::Publisher ||
    entity.type() == EntityType::Client)
  {
    return true;
  }
  return false;
}

///=============================================================================
void GraphCache::update_event_counters(
  liveliness::ConstEntityPtr entity,
  const rmw_zenoh_event_type_t event_id,
  int32_t change)
{
  if (event_id > ZENOH_EVENT_ID_MAX) {
    return;
  }

  std::lock_guard<std::mutex> lock(events_mutex_);

  // Trigger callback set for this entity for the event type.
  GraphEventCallbackMap::const_iterator event_callbacks_it =
    event_callbacks_.find(entity->keyexpr_hash());
  if (event_callbacks_it != event_callbacks_.end()) {
    GraphEventCallbacks::const_iterator callback_it =
      event_callbacks_it->second.find(event_id);
    if (callback_it != event_callbacks_it->second.end()) {
      callback_it->second(change);
    }
  }
}
}  // namespace rmw_zenoh_cpp
