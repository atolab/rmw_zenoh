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

#ifndef DETAIL__LIVELINESS_UTILS_HPP_
#define DETAIL__LIVELINESS_UTILS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <zenoh.hxx>

#include "rmw/types.h"

namespace rmw_zenoh_cpp
{
namespace liveliness
{
///=============================================================================
struct NodeInfo
{
  std::size_t domain_id_;
  std::string ns_;
  std::string name_;
  std::string enclave_;

  NodeInfo(
    std::size_t domain_id,
    std::string ns,
    std::string name,
    std::string enclave);
};

///=============================================================================
struct TopicInfo
{
  std::string name_;
  std::string type_;
  std::string type_hash_;
  std::string topic_keyexpr_;
  rmw_qos_profile_t qos_;

  TopicInfo(
    std::size_t domain_id,
    std::string name,
    std::string type,
    std::string type_hash,
    rmw_qos_profile_t qos);
};

///=============================================================================
/// Returns the keyexpr for liveliness subscription.
std::string subscription_token(size_t domain_id);

///=============================================================================
enum class EntityType : uint8_t
{
  Node,
  Publisher,
  Subscription,
  Service,
  Client
};

///=============================================================================
// An struct to bundle results of parsing a token.
/**
 * Every entity will generate a unique key-expression for setting up a liveliness token.
 *
 * The minimal key-expression is of the form:
 *
 * <ADMIN_SPACE>/<domainid>/<zid>/<nid>/<id>/<entity>/<namespace>/<nodename>
 *
 * Where:
 *  <domainid> - A number set by the user to "partition" graphs.  Roughly equivalent to the domain ID in DDS.
 *  <zid> - The zenoh session's id with elements concatenated into a string using '.' as separator.
 *  <nid> - A unique ID within the zenoh session of the node which created this entity.
 *  <id> - A unique ID within the zenoh session to identify this entity. When entity is a node, the id and nid are equal.
 *  <entity> - The type of entity.  This can be one of "NN" for a Network Node, "MP" for a Message Publisher, "MS" for a Message Subscription, "SS" for a Service Server, or "SC" for a Service Client.
 *  <namespace> - The ROS namespace for this entity.  If the namespace is absolute, this function will add in an _ for later parsing reasons.
 *  <nodename> - The ROS node name for this entity.
 *
 * For entities with topic information, the liveliness token keyexpr have additional fields:
 *
 * <ADMIN_SPACE>/<domainid>/<zid>/<id>/<entity>/<namespace>/<nodename>/<topic_name>/<topic_type>/<topic_type_hash>/<topic_qos>
 *  <topic_name> - The ROS topic name for this entity.
 *  <topic_type> - The type for the topic.
 *  <topic_type_hash> - The type hash for the topic.
 *  <topic_qos> - The qos for the topic (see qos_to_keyexpr() docstring for more information).
 *
 * For example, the liveliness expression for a publisher within a /talker node that publishes
 * an std_msgs/msg/String over topic /chatter and with QoS settings of Reliability: best_effort,
 * Durability: transient_local, History: keep_all, and depth: 10, would be
 * "@ros2_lv/0/q1w2e3r4t5y6/1/32/MP/_/talker/dds_::std_msgs::msg::String/2:1:2,10".
 * Note: The domain_id is assumed to be 0 and a random id is used in the example. Also the
 *  _dds:: prefix in the topic_type is an artifact of the type support implementation and is
 *  removed when reporting the topic_type in graph_cache.cpp (see _demangle_if_ros_type()).
 */
class Entity;
using EntityPtr = std::shared_ptr<Entity>;
using ConstEntityPtr = std::shared_ptr<const Entity>;
class Entity
{
public:
  // TODO(Yadunund): Find a way to better bundle the type and the associated data.
  /// @brief Make an Entity from datatypes. This will return nullopt if the required
  ///   fields are not present for the EntityType.
  /// @param zid The zenoh session id within which this entity was created.
  /// @param id A unique id for this entity within the zenoh session.
  /// @param type The type of the entity.
  /// @param node_info The node information that is required for all entities.
  /// @param topic_info An optional topic information for relevant entities.
  /// @return An entity if all inputs are valid. This way no invalid entities can be created.
  static EntityPtr make(
    zenoh::Id zid,
    const std::string & nid,
    const std::string & id,
    EntityType type,
    NodeInfo node_info,
    std::optional<TopicInfo> topic_info = std::nullopt);

  /// Make an Entity from a liveliness keyexpr.
  static EntityPtr make(const std::string & keyexpr);

  // Get the zenoh session id as a string. This is not unique as entities
  // created within the same session, will have the same ids.
  std::string zid() const;

  // Get the id of the node of this entity.
  std::string nid() const;

  // Get the id of the entity local to a zenoh session.
  // Use keyexpr_hash() to retrieve a globally unique id.
  std::string id() const;

  // Interim method to get a globally unique id for this entity which is the hash of the keyexpr.
  std::size_t keyexpr_hash() const;

  /// Get the entity type.
  EntityType type() const;

  std::string node_namespace() const;

  std::string node_name() const;

  std::string node_enclave() const;

  /// Get the NodeInfo.
  NodeInfo node_info() const;

  /// Get the TopicInfo if present.
  std::optional<TopicInfo> topic_info() const;

  /// Get the liveliness keyexpr for this entity.
  std::string liveliness_keyexpr() const;

  // Two entities are equal if their keyexpr_hash are equal.
  bool operator==(const Entity & other) const;

  std::array<uint8_t, RMW_GID_STORAGE_SIZE> copy_gid() const;

private:
  Entity(
    std::string zid,
    std::string nid,
    std::string id,
    EntityType type,
    NodeInfo node_info,
    std::optional<TopicInfo> topic_info);

  std::string zid_;
  std::string nid_;
  std::string id_;
  std::size_t keyexpr_hash_;
  EntityType type_;
  NodeInfo node_info_;
  std::optional<TopicInfo> topic_info_;
  std::string liveliness_keyexpr_;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid_{};
};

///=============================================================================
/// Replace "/" instances with "%".
std::string mangle_name(const std::string & input);

///=============================================================================
/// Replace "%" instances with "/".
std::string demangle_name(const std::string & input);

///=============================================================================
/**
 * Convert a rmw_qos_profile_t to a string with format:
 *
 * <ReliabilityKind>:<DurabilityKind>:<HistoryKind>,<HistoryDepth>:<DeadlineSec, DeadlineNSec>:<LifespanSec, LifespanNSec>:<Liveliness, LivelinessSec, LivelinessNSec>"
 * Where:
 *  <ReliabilityKind> - enum value from rmw_qos_reliability_policy_e.
 *  <DurabilityKind> - enum value from rmw_qos_durability_policy_e.
 *  <HistoryKind> - enum value from rmw_qos_history_policy_e.
 *  <HistoryDepth> - The depth number.
 *  <DeadlineSec> - The seconds component of the deadline duration.
 *  <DeadlineNSec> - The nanoseconds component of the deadline duration.
 *  <LifespanSec> - The seconds component of the lifespan duration.
 *  <LifespanNSec> - The nanoseconds component of the lifespan duration.
 *  <Liveliness> - enum value from rmw_qos_liveliness_policy_e.
 *  <LivelinessSec> - The seconds component of the liveliness duration.
 *  <LivelinessNSec> - The nanoseconds component of the liveliness duration.
 * For example, the liveliness substring for a topic with Reliability policy: reliable,
 * Durability policy: volatile, History policy: keep_last, and depth: 10,
 * Deadline: 5s0ns, Lifespan 60s3000ns, Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
 * LivelinessDuration: 0s0ns would be "1:2:1,10:0,0:60,3000:1,0,0".
 *
 * See rmw/types.h for the values of each policy enum.
 */
std::string qos_to_keyexpr(const rmw_qos_profile_t & qos);

///=============================================================================
/// Convert a rmw_qos_profile_t from a keyexpr. Return std::nullopt if invalid.
std::optional<rmw_qos_profile_t> keyexpr_to_qos(const std::string & keyexpr);
}  // namespace liveliness

using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
}  // namespace rmw_zenoh_cpp

///=============================================================================
// Allow Entity to be hashed and used as a key in unordered_maps/sets
namespace std
{
template<>
struct hash<rmw_zenoh_cpp::liveliness::Entity>
{
  auto operator()(const rmw_zenoh_cpp::liveliness::Entity & entity) const -> size_t
  {
    return entity.keyexpr_hash();
  }
};

template<>
struct hash<rmw_zenoh_cpp::liveliness::ConstEntityPtr>
{
  auto operator()(const rmw_zenoh_cpp::liveliness::ConstEntityPtr & entity) const -> size_t
  {
    return entity->keyexpr_hash();
  }
};

template<>
struct equal_to<rmw_zenoh_cpp::liveliness::ConstEntityPtr>
{
  auto operator()(
    const rmw_zenoh_cpp::liveliness::ConstEntityPtr & lhs,
    const rmw_zenoh_cpp::liveliness::ConstEntityPtr & rhs) const -> bool
  {
    return lhs->keyexpr_hash() == rhs->keyexpr_hash();
  }
};

template<>
struct hash<rmw_zenoh_cpp::Gid>
{
  std::size_t operator()(const rmw_zenoh_cpp::Gid & gid) const noexcept
  {
    // This function implemented FNV-1a 64-bit as the GID is small enough (e.g. as of commit db4d05e
    // of ros2/rmw RMW_GID_STORAGE_SIZE is set to 16) and FNV is known to be efficient in such cases.
    //
    // See https://github.com/ros2/rmw/blob/db4d05e/rmw/include/rmw/types.h#L44
    // and https://datatracker.ietf.org/doc/html/draft-eastlake-fnv-17.html
    static constexpr uint64_t FNV_OFFSET_BASIS_64 = 0x00000100000001b3;
    static constexpr uint64_t FNV_PRIME_64 = 0xcbf29ce484222325;

    uint64_t hash = FNV_OFFSET_BASIS_64;
    for (const uint8_t & byte : gid) {
      hash ^= byte;
      hash *= FNV_PRIME_64;
    }
    return hash;
  }
};
}  // namespace std

#endif  // DETAIL__LIVELINESS_UTILS_HPP_
