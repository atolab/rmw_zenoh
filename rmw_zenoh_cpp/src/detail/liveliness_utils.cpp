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

#include "liveliness_utils.hpp"

#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <zenoh.hxx>

#include "logging_macros.hpp"
#include "qos.hpp"
#include "simplified_xxhash3.hpp"

#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"

namespace rmw_zenoh_cpp
{
namespace liveliness
{
///=============================================================================
NodeInfo::NodeInfo(
  std::size_t domain_id,
  std::string ns,
  std::string name,
  std::string enclave)
: domain_id_(std::move(domain_id)),
  ns_(std::move(ns)),
  name_(std::move(name)),
  enclave_(std::move(enclave))
{
  // Do nothing.
}

namespace
{
// Helper function to create a copy of a string after removing any
// leading or trailing slashes.
std::string strip_slashes(const std::string & str)
{
  std::string ret = str;
  std::size_t start = 0;
  std::size_t end = str.length() - 1;
  if (str[0] == '/') {
    ++start;
  }
  if (str[end] == '/') {
    --end;
  }
  return ret.substr(start, end - start + 1);
}
}  // namespace
///=============================================================================
TopicInfo::TopicInfo(
  std::size_t domain_id,
  std::string name,
  std::string type,
  std::string type_hash,
  rmw_qos_profile_t qos)
: name_(std::move(name)),
  type_(std::move(type)),
  type_hash_(std::move(type_hash)),
  qos_(std::move(qos))
{
  topic_keyexpr_ = std::to_string(domain_id);
  topic_keyexpr_ += "/";
  topic_keyexpr_ += strip_slashes(name_);
  topic_keyexpr_ += "/";
  topic_keyexpr_ += type_;
  topic_keyexpr_ += "/";
  topic_keyexpr_ += type_hash_;
}

///=============================================================================
namespace
{
/// Enum of liveliness key-expression components.
enum KeyexprIndex
{
  AdminSpace,
  DomainId,
  Zid,
  Nid,
  Id,
  EntityStr,
  Enclave,
  Namespace,
  NodeName,
  TopicName,
  TopicType,
  TopicTypeHash,
  TopicQoS
};

// Every keyexpression will have components upto node name.
#define KEYEXPR_INDEX_MIN KeyexprIndex::NodeName
#define KEYEXPR_INDEX_MAX KeyexprIndex::TopicQoS

/// The admin space used to prefix the liveliness tokens.
static const char ADMIN_SPACE[] = "@ros2_lv";
static const char NODE_STR[] = "NN";
static const char PUB_STR[] = "MP";
static const char SUB_STR[] = "MS";
static const char SRV_STR[] = "SS";
static const char CLI_STR[] = "SC";
static const char KEYEXPR_DELIMITER = '/';
static const char SLASH_REPLACEMENT = '%';
static const char QOS_DELIMITER = ':';
static const char QOS_COMPONENT_DELIMITER = ',';

static const std::unordered_map<EntityType, std::string> entity_to_str = {
  {EntityType::Node, NODE_STR},
  {EntityType::Publisher, PUB_STR},
  {EntityType::Subscription, SUB_STR},
  {EntityType::Service, SRV_STR},
  {EntityType::Client, CLI_STR}
};

static const std::unordered_map<std::string, EntityType> str_to_entity = {
  {NODE_STR, EntityType::Node},
  {PUB_STR, EntityType::Publisher},
  {SUB_STR, EntityType::Subscription},
  {SRV_STR, EntityType::Service},
  {CLI_STR, EntityType::Client}
};

static const std::unordered_map<std::string, rmw_qos_history_policy_e> str_to_qos_history = {
  {std::to_string(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT), RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_HISTORY_KEEP_LAST), RMW_QOS_POLICY_HISTORY_KEEP_LAST},
  {std::to_string(RMW_QOS_POLICY_HISTORY_KEEP_ALL), RMW_QOS_POLICY_HISTORY_KEEP_ALL},
  {std::to_string(RMW_QOS_POLICY_HISTORY_UNKNOWN), RMW_QOS_POLICY_HISTORY_UNKNOWN}
};

static const std::unordered_map<std::string,
  rmw_qos_reliability_policy_e> str_to_qos_reliability = {
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT),
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_RELIABLE), RMW_QOS_POLICY_RELIABILITY_RELIABLE},
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT), RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
  {std::to_string(RMW_QOS_POLICY_RELIABILITY_UNKNOWN), RMW_QOS_POLICY_RELIABILITY_UNKNOWN}
};

static const std::unordered_map<std::string, rmw_qos_durability_policy_e> str_to_qos_durability = {
  {std::to_string(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT),
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
  {std::to_string(RMW_QOS_POLICY_DURABILITY_VOLATILE), RMW_QOS_POLICY_DURABILITY_VOLATILE},
  {std::to_string(RMW_QOS_POLICY_DURABILITY_UNKNOWN), RMW_QOS_POLICY_DURABILITY_UNKNOWN}
};

static const std::unordered_map<std::string, rmw_qos_liveliness_policy_e> str_to_qos_liveliness = {
  {std::to_string(RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT),
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT},
  {std::to_string(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC),
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC},
  {std::to_string(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC),
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC},
  {std::to_string(RMW_QOS_POLICY_LIVELINESS_UNKNOWN), RMW_QOS_POLICY_LIVELINESS_UNKNOWN},
  {std::to_string(RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE),
    RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE}
};

std::vector<std::string> split_keyexpr(
  const std::string & keyexpr,
  const char delim = KEYEXPR_DELIMITER)
{
  std::vector<std::string> result = {};
  size_t start = 0;
  size_t end = keyexpr.find(delim);
  while (end != std::string::npos) {
    result.push_back(keyexpr.substr(start, end - start));
    start = end + 1;
    end = keyexpr.find(delim, start);
  }
  // Finally add the last substr.
  result.push_back(keyexpr.substr(start));
  return result;
}

///=============================================================================
// Helper function to convert string to size_t.
// The function is templated to enable conversion to size_t or std::size_t.
template<typename T>
std::optional<T> str_to_size_t(const std::string & str, const T default_value)
{
  if (str.empty()) {
    return default_value;
  }
  errno = 0;
  char * endptr;
  // TODO(Yadunund): strtoul returns an unsigned long, not size_t.
  // Depending on the architecture and platform, these may not be the same size.
  // Further, if the incoming str is a signed integer, storing it in a size_t is incorrect.
  // We should fix this piece of code to deal with both of those situations.
  size_t num = strtoul(str.c_str(), &endptr, 10);
  if (endptr == str.c_str()) {
    // No values were converted, this is an error
    RMW_SET_ERROR_MSG("no valid numbers available");
    return std::nullopt;
  } else if (*endptr != '\0') {
    // There was junk after the number
    RMW_SET_ERROR_MSG("non-numeric values");
    return std::nullopt;
  } else if (errno != 0) {
    // Some other error occurred, which may include overflow or underflow
    RMW_SET_ERROR_MSG(
      "an undefined error occurred while getting the number, this may be an overflow");
    return std::nullopt;
  }
  return num;
}
}  // namespace

///=============================================================================
// TODO(Yadunund): Rely on maps to retrieve strings.
std::string qos_to_keyexpr(const rmw_qos_profile_t & qos)
{
  std::string keyexpr = "";
  const rmw_qos_profile_t & default_qos = QoS::get().default_qos();

  // Reliability.
  if (qos.reliability != default_qos.reliability) {
    keyexpr += std::to_string(qos.reliability);
  }
  keyexpr += QOS_DELIMITER;

  // Durability.
  if (qos.durability != default_qos.durability) {
    keyexpr += std::to_string(qos.durability);
  }
  keyexpr += QOS_DELIMITER;

  // History.
  if (qos.history != default_qos.history) {
    keyexpr += std::to_string(qos.history);
  }
  keyexpr += QOS_COMPONENT_DELIMITER;
  if (qos.depth != default_qos.depth) {
    keyexpr += std::to_string(qos.depth);
  }
  keyexpr += QOS_DELIMITER;

  // Deadline.
  if (qos.deadline.sec != default_qos.deadline.sec) {
    keyexpr += std::to_string(qos.deadline.sec);
  }
  keyexpr += QOS_COMPONENT_DELIMITER;
  if (qos.deadline.nsec != default_qos.deadline.nsec) {
    keyexpr += std::to_string(qos.deadline.nsec);
  }
  keyexpr += QOS_DELIMITER;

  // Lifespan.
  if (qos.lifespan.sec != default_qos.lifespan.sec) {
    keyexpr += std::to_string(qos.lifespan.sec);
  }
  keyexpr += QOS_COMPONENT_DELIMITER;
  if (qos.lifespan.nsec != default_qos.lifespan.nsec) {
    keyexpr += std::to_string(qos.lifespan.nsec);
  }
  keyexpr += QOS_DELIMITER;

  // Liveliness.
  if (qos.liveliness != default_qos.liveliness) {
    keyexpr += std::to_string(qos.liveliness);
  }
  keyexpr += QOS_COMPONENT_DELIMITER;
  if (qos.liveliness_lease_duration.sec != default_qos.liveliness_lease_duration.sec) {
    keyexpr += std::to_string(qos.liveliness_lease_duration.sec);
  }
  keyexpr += QOS_COMPONENT_DELIMITER;
  if (qos.liveliness_lease_duration.nsec != default_qos.liveliness_lease_duration.nsec) {
    keyexpr += std::to_string(qos.liveliness_lease_duration.nsec);
  }

  return keyexpr;
}

///=============================================================================
std::optional<rmw_qos_profile_t> keyexpr_to_qos(const std::string & keyexpr)
{
  const rmw_qos_profile_t & default_qos = QoS::get().default_qos();
  rmw_qos_profile_t qos;

  const std::vector<std::string> parts = split_keyexpr(keyexpr, QOS_DELIMITER);
  if (parts.size() < 6) {
    return std::nullopt;
  }
  const std::vector<std::string> history_parts = split_keyexpr(parts[2], QOS_COMPONENT_DELIMITER);
  if (history_parts.size() < 2) {
    return std::nullopt;
  }
  const std::vector<std::string> deadline_parts = split_keyexpr(parts[3], QOS_COMPONENT_DELIMITER);
  if (deadline_parts.size() < 2) {
    return std::nullopt;
  }
  const std::vector<std::string> lifespan_parts = split_keyexpr(parts[4], QOS_COMPONENT_DELIMITER);
  if (lifespan_parts.size() < 2) {
    return std::nullopt;
  }
  const std::vector<std::string> liveliness_parts =
    split_keyexpr(parts[5], QOS_COMPONENT_DELIMITER);
  if (liveliness_parts.size() < 3) {
    return std::nullopt;
  }

  try {
    qos.history = history_parts[0].empty() ? default_qos.history : str_to_qos_history.at(
      history_parts[0]);
    qos.reliability = parts[0].empty() ? default_qos.reliability : str_to_qos_reliability.at(
      parts[0]);
    qos.durability = parts[1].empty() ? default_qos.durability : str_to_qos_durability.at(parts[1]);
    qos.liveliness =
      liveliness_parts[0].empty() ? default_qos.liveliness : str_to_qos_liveliness.at(
      liveliness_parts[0]);
  } catch (const std::exception & e) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("Error setting QoS values from strings: %s", e.what());
    return std::nullopt;
  }
  const auto maybe_depth = str_to_size_t(history_parts[1], default_qos.depth);
  const auto maybe_deadline_s = str_to_size_t(deadline_parts[0], default_qos.deadline.sec);
  const auto maybe_deadline_ns = str_to_size_t(deadline_parts[1], default_qos.deadline.nsec);
  const auto maybe_lifespan_s = str_to_size_t(lifespan_parts[0], default_qos.lifespan.sec);
  const auto maybe_lifespan_ns = str_to_size_t(lifespan_parts[1], default_qos.lifespan.nsec);
  const auto maybe_liveliness_s = str_to_size_t(
    liveliness_parts[1],
    default_qos.liveliness_lease_duration.sec);
  const auto maybe_liveliness_ns = str_to_size_t(
    liveliness_parts[2],
    default_qos.liveliness_lease_duration.nsec);
  if (maybe_depth == std::nullopt ||
    maybe_deadline_s == std::nullopt ||
    maybe_deadline_ns == std::nullopt ||
    maybe_lifespan_s == std::nullopt ||
    maybe_lifespan_ns == std::nullopt ||
    maybe_liveliness_s == std::nullopt ||
    maybe_liveliness_ns == std::nullopt)
  {
    // Error already set.
    return std::nullopt;
  }
  qos.depth = *maybe_depth;
  qos.deadline.sec = *maybe_deadline_s;
  qos.deadline.nsec = *maybe_deadline_ns;
  qos.lifespan.sec = *maybe_lifespan_s;
  qos.lifespan.nsec = *maybe_lifespan_ns;
  qos.liveliness_lease_duration.sec = *maybe_liveliness_s;
  qos.liveliness_lease_duration.nsec = *maybe_liveliness_ns;

  return qos;
}

///=============================================================================
std::string subscription_token(size_t domain_id)
{
  std::string token = std::string(ADMIN_SPACE) + "/" + std::to_string(domain_id) + "/**";
  return token;
}

///=============================================================================
Entity::Entity(
  std::string zid,
  std::string nid,
  std::string id,
  EntityType type,
  NodeInfo node_info,
  std::optional<TopicInfo> topic_info)
: zid_(std::move(zid)),
  nid_(std::move(nid)),
  id_(std::move(id)),
  type_(std::move(type)),
  node_info_(std::move(node_info)),
  topic_info_(std::move(topic_info))
{
  std::string keyexpr_parts[KEYEXPR_INDEX_MAX + 1] {};
  keyexpr_parts[KeyexprIndex::AdminSpace] = ADMIN_SPACE;
  keyexpr_parts[KeyexprIndex::DomainId] = std::to_string(node_info_.domain_id_);
  keyexpr_parts[KeyexprIndex::Zid] = zid_;
  keyexpr_parts[KeyexprIndex::Nid] = nid_;
  keyexpr_parts[KeyexprIndex::Id] = id_;
  keyexpr_parts[KeyexprIndex::EntityStr] = entity_to_str.at(type_);
  // An empty namespace from rcl will contain "/" but zenoh does not allow keys with "//".
  // Hence we mangle the empty namespace such that splitting the key
  // will always result in 5 parts.
  keyexpr_parts[KeyexprIndex::Enclave] = mangle_name(node_info_.enclave_);
  keyexpr_parts[KeyexprIndex::Namespace] = mangle_name(node_info_.ns_);
  keyexpr_parts[KeyexprIndex::NodeName] = mangle_name(node_info_.name_);
  // If this entity has a topic info, append it to the token.
  if (topic_info_.has_value()) {
    const auto & topic_info = this->topic_info_.value();
    keyexpr_parts[KeyexprIndex::TopicName] = mangle_name(topic_info.name_);
    keyexpr_parts[KeyexprIndex::TopicType] = mangle_name(topic_info.type_);
    keyexpr_parts[KeyexprIndex::TopicTypeHash] = mangle_name(topic_info.type_hash_);
    keyexpr_parts[KeyexprIndex::TopicQoS] = qos_to_keyexpr(topic_info.qos_);
  }

  for (std::size_t i = 0; i < KEYEXPR_INDEX_MAX + 1; ++i) {
    bool last = false;
    if (!keyexpr_parts[i].empty()) {
      this->liveliness_keyexpr_ += std::move(keyexpr_parts[i]);
    }
    if (i == KEYEXPR_INDEX_MAX || keyexpr_parts[i + 1].empty()) {
      last = true;
    }
    if (last) {
      break;
    }
    // Append the delimiter unless it is the last component.
    this->liveliness_keyexpr_ += KEYEXPR_DELIMITER;
  }

  // Here we hash the complete std::string that comprises the liveliness keyexpression
  // into a GID that is associated with every entity in the system.  This is the GID that will be
  // returned to the RMW layer as necessary.
  simplified_XXH128_hash_t keyexpr_gid =
    simplified_XXH3_128bits(this->liveliness_keyexpr_.c_str(), this->liveliness_keyexpr_.length());
  memcpy(this->gid_.data(), &keyexpr_gid.low64, sizeof(keyexpr_gid.low64));
  memcpy(this->gid_.data() + sizeof(keyexpr_gid.low64), &keyexpr_gid.high64,
        sizeof(keyexpr_gid.high64));

  // We also hash the liveliness keyexpression into a size_t that we use to index into our maps.
  this->keyexpr_hash_ = std::hash<Gid>{}(this->gid_);
}

///=============================================================================
std::shared_ptr<Entity> Entity::make(
  zenoh::Id zid,
  const std::string & nid,
  const std::string & id,
  EntityType type,
  NodeInfo node_info,
  std::optional<TopicInfo> topic_info)
{
  if (id.empty()) {
    RCUTILS_SET_ERROR_MSG("Invalid id.");
    return nullptr;
  }
  if (entity_to_str.find(type) == entity_to_str.end()) {
    RCUTILS_SET_ERROR_MSG("Invalid entity type.");
    return nullptr;
  }
  if (node_info.ns_.empty() || node_info.name_.empty()) {
    RCUTILS_SET_ERROR_MSG("Invalid node_info for entity.");
    return nullptr;
  }
  if (type != EntityType::Node && !topic_info.has_value()) {
    RCUTILS_SET_ERROR_MSG("Invalid topic_info for entity.");
    return nullptr;
  }

  return std::make_shared<Entity>(
    Entity{
        std::string(zid.to_string()),
        nid,
        id,
        std::move(type),
        std::move(node_info),
        std::move(topic_info)});
}

///=============================================================================
std::shared_ptr<Entity> Entity::make(const std::string & keyexpr)
{
  std::vector<std::string> parts = split_keyexpr(keyexpr);
  // Every token will contain at least 7 parts:
  // (ADMIN_SPACE, domain_id, zid, id, entity_type, namespace, node_name).
  // Basic validation.
  if (parts.size() < KEYEXPR_INDEX_MIN + 1) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received invalid liveliness token with %lu/%d parts: %s",
      parts.size(),
      KEYEXPR_INDEX_MIN + 1, keyexpr.c_str());
    return nullptr;
  }
  for (const std::string & p : parts) {
    if (p.empty()) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received invalid liveliness token with empty parts: %s", keyexpr.c_str());
      return nullptr;
    }
  }

  if (parts[KeyexprIndex::AdminSpace] != ADMIN_SPACE) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid admin space.");
    return nullptr;
  }

  // Get the entity, ie NN, MP, MS, SS, SC.
  std::string & entity_str = parts[KeyexprIndex::EntityStr];
  std::unordered_map<std::string, EntityType>::const_iterator entity_it =
    str_to_entity.find(entity_str);
  if (entity_it == str_to_entity.end()) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Received liveliness token with invalid entity %s.", entity_str.c_str());
    return nullptr;
  }

  EntityType entity_type = entity_it->second;
  std::size_t domain_id = std::stoul(parts[KeyexprIndex::DomainId]);
  std::string & zid = parts[KeyexprIndex::Zid];
  std::string & nid = parts[KeyexprIndex::Nid];
  std::string & id = parts[KeyexprIndex::Id];
  std::string enclave = demangle_name(std::move(parts[KeyexprIndex::Enclave]));
  std::string ns = demangle_name(std::move(parts[KeyexprIndex::Namespace]));
  std::string node_name = demangle_name(std::move(parts[KeyexprIndex::NodeName]));
  std::optional<TopicInfo> topic_info = std::nullopt;

  // Populate topic_info if we have a token for an entity other than a node.
  if (entity_type != EntityType::Node) {
    if (parts.size() < KEYEXPR_INDEX_MAX + 1) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token for non-node entity without required parameters.");
      return nullptr;
    }
    std::optional<rmw_qos_profile_t> qos = keyexpr_to_qos(parts[KeyexprIndex::TopicQoS]);
    if (!qos.has_value()) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Received liveliness token with invalid qos keyexpr");
      return nullptr;
    }
    topic_info = TopicInfo{
      domain_id,
      demangle_name(std::move(parts[KeyexprIndex::TopicName])),
      demangle_name(std::move(parts[KeyexprIndex::TopicType])),
      demangle_name(std::move(parts[KeyexprIndex::TopicTypeHash])),
      std::move(qos.value())
    };
  }

  return std::make_shared<Entity>(
    Entity{
        std::move(zid),
        std::move(nid),
        std::move(id),
        std::move(entity_type),
        NodeInfo{std::move(domain_id), std::move(ns), std::move(node_name), std::move(enclave)},
        std::move(topic_info)});
}

///=============================================================================
std::string Entity::zid() const
{
  return this->zid_;
}

///=============================================================================
std::string Entity::nid() const
{
  return this->nid_;
}

///=============================================================================
std::string Entity::id() const
{
  return this->id_;
}

///=============================================================================
std::size_t Entity::keyexpr_hash() const
{
  return this->keyexpr_hash_;
}

///=============================================================================
EntityType Entity::type() const
{
  return this->type_;
}

std::string Entity::node_namespace() const
{
  return this->node_info_.ns_;
}

std::string Entity::node_name() const
{
  return this->node_info_.name_;
}

std::string Entity::node_enclave() const
{
  return this->node_info_.enclave_;
}

///=============================================================================
NodeInfo Entity::node_info() const
{
  return this->node_info_;
}

///=============================================================================
std::optional<TopicInfo> Entity::topic_info() const
{
  return this->topic_info_;
}

///=============================================================================
std::string Entity::liveliness_keyexpr() const
{
  return this->liveliness_keyexpr_;
}

///=============================================================================
std::array<uint8_t, RMW_GID_STORAGE_SIZE> Entity::copy_gid() const
{
  return gid_;
}

///=============================================================================
bool Entity::operator==(const Entity & other) const
{
  return other.keyexpr_hash() == keyexpr_hash_;
}

///=============================================================================
std::string mangle_name(const std::string & input)
{
  std::string output = "";
  for (std::size_t i = 0; i < input.length(); ++i) {
    if (input[i] == '/') {
      output += SLASH_REPLACEMENT;
    } else {
      output += input[i];
    }
  }
  return output;
}

///=============================================================================
std::string demangle_name(const std::string & input)
{
  std::string output = "";
  for (std::size_t i = 0; i < input.length(); ++i) {
    if (input[i] == SLASH_REPLACEMENT) {
      output += '/';
    } else {
      output += input[i];
    }
  }
  return output;
}
}  // namespace liveliness
}  // namespace rmw_zenoh_cpp
