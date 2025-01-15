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

#ifndef DETAIL__RMW_SUBSCRIPTION_DATA_HPP_
#define DETAIL__RMW_SUBSCRIPTION_DATA_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

#include <zenoh.hxx>

#include "event.hpp"
#include "graph_cache.hpp"
#include "liveliness_utils.hpp"
#include "message_type_support.hpp"
#include "attachment_helpers.hpp"
#include "type_support_common.hpp"
#include "zenoh_utils.hpp"

#include "rcutils/allocator.h"

#include "rmw/rmw.h"
#include "rmw/ret_types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
class SubscriptionData final : public std::enable_shared_from_this<SubscriptionData>
{
public:
  struct Message
  {
    explicit Message(
      const zenoh::Bytes & bytes,
      uint64_t recv_ts,
      AttachmentData && attachment);

    ~Message() = default;

    Payload payload;
    uint64_t recv_timestamp;
    AttachmentData attachment;
  };

  // Make a shared_ptr of SubscriptionData.
  static std::shared_ptr<SubscriptionData> make(
    std::shared_ptr<zenoh::Session> session,
    std::shared_ptr<GraphCache> graph_cache,
    const rmw_node_t * const node,
    liveliness::NodeInfo node_info,
    std::size_t node_id,
    std::size_t Subscription_id,
    const std::string & topic_name,
    const rosidl_message_type_support_t * type_support,
    const rmw_qos_profile_t * qos_profile);

  // Get a copy of the keyexpr_hash of this SubscriptionData's liveliness::Entity.
  std::size_t keyexpr_hash() const;

  // Get a copy of the TopicInfo of this SubscriptionData.
  liveliness::TopicInfo topic_info() const;

  // Returns true if liveliness token is still valid.
  bool liveliness_is_valid() const;

  // Get the events manager of this SubscriptionData.
  std::shared_ptr<EventsManager> events_mgr() const;

  // Shutdown this SubscriptionData.
  rmw_ret_t shutdown();

  // Check if this SubscriptionData is shutdown.
  bool is_shutdown() const;

  // Add a new message to the queue.
  void add_new_message(std::unique_ptr<Message> msg, const std::string & topic_name);

  bool queue_has_data_and_attach_condition_if_not(rmw_wait_set_data_t * wait_set_data);

  bool detach_condition_and_queue_is_empty();

  rmw_ret_t take_one_message(
    void * ros_message,
    rmw_message_info_t * message_info,
    bool * taken);

  rmw_ret_t take_serialized_message(
    rmw_serialized_message_t * serialized_message,
    bool * taken,
    rmw_message_info_t * message_info);

  void set_on_new_message_callback(
    rmw_event_callback_t callback,
    const void * user_data);

  std::shared_ptr<GraphCache> graph_cache() const;

  // Destructor.
  ~SubscriptionData();

private:
  SubscriptionData(
    const rmw_node_t * rmw_node,
    std::shared_ptr<GraphCache> graph_cache,
    std::shared_ptr<liveliness::Entity> entity,
    std::shared_ptr<zenoh::Session> session,
    const void * type_support_impl,
    std::unique_ptr<MessageTypeSupport> type_support);

  bool init();

  // Internal mutex.
  mutable std::mutex mutex_;
  // The parent node.
  const rmw_node_t * rmw_node_;
  // The graph cache.
  std::shared_ptr<GraphCache> graph_cache_;
  // The Entity generated for the subscription.
  std::shared_ptr<liveliness::Entity> entity_;
  // A shared session
  std::shared_ptr<zenoh::Session> sess_;
  // An owned advanced subscriber.
  std::optional<zenoh::ext::AdvancedSubscriber<void>> sub_;
  // Liveliness token for the subscription.
  std::optional<zenoh::LivelinessToken> token_;
  // Type support fields
  const void * type_support_impl_;
  std::unique_ptr<MessageTypeSupport> type_support_;
  std::deque<std::unique_ptr<Message>> message_queue_;
  // Map GID of a subscription to the sequence number of the message it published.
  std::unordered_map<size_t, int64_t> last_known_published_msg_;
  // Wait set data.
  rmw_wait_set_data_t * wait_set_data_;
  // Callback managers.
  DataCallbackManager data_callback_mgr_;
  std::shared_ptr<EventsManager> events_mgr_;
  // Shutdown flag.
  bool is_shutdown_;
  // Whether the object has ever successfully been initialized.
  bool initialized_;
};
using SubscriptionDataPtr = std::shared_ptr<SubscriptionData>;
using SubscriptionDataConstPtr = std::shared_ptr<const SubscriptionData>;
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__RMW_SUBSCRIPTION_DATA_HPP_
