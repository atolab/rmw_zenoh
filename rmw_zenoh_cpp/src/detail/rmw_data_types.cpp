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

#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

#include "logging_macros.hpp"
#include "rmw_data_types.hpp"

#include "rmw/impl/cpp/macros.hpp"


///=============================================================================
namespace rmw_zenoh_cpp
{
///=============================================================================
void rmw_client_data_t::notify()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (wait_set_data_ != nullptr) {
    std::lock_guard<std::mutex> wait_set_lock(wait_set_data_->condition_mutex);
    wait_set_data_->triggered = true;
    wait_set_data_->condition_variable.notify_one();
  }
}

///=============================================================================
void rmw_client_data_t::add_new_reply(std::unique_ptr<ZenohReply> reply)
{
  std::lock_guard<std::mutex> lock(reply_queue_mutex_);
  if (adapted_qos_profile.history != RMW_QOS_POLICY_HISTORY_KEEP_ALL &&
    reply_queue_.size() >= adapted_qos_profile.depth)
  {
    // Log warning if message is discarded due to hitting the queue depth
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_loan(this->keyexpr), &keystr);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Reply queue depth of %ld reached, discarding oldest reply "
      "for client for %s",
      adapted_qos_profile.depth,
      z_string_data(z_loan(keystr)));
    reply_queue_.pop_front();
  }
  reply_queue_.emplace_back(std::move(reply));

  // Since we added new data, trigger user callback and guard condition if they
  // are available
  data_callback_mgr.trigger_callback();
  notify();
}

///=============================================================================
bool rmw_client_data_t::queue_has_data_and_attach_condition_if_not(
  rmw_wait_set_data_t * wait_set_data)
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  if (!reply_queue_.empty()) {
    return true;
  }
  wait_set_data_ = wait_set_data;

  return false;
}

///=============================================================================
bool rmw_client_data_t::detach_condition_and_queue_is_empty()
{
  std::lock_guard<std::mutex> lock(condition_mutex_);
  wait_set_data_ = nullptr;

  return reply_queue_.empty();
}

///=============================================================================
std::unique_ptr<ZenohReply> rmw_client_data_t::pop_next_reply()
{
  std::lock_guard<std::mutex> lock(reply_queue_mutex_);

  if (reply_queue_.empty()) {
    return nullptr;
  }

  std::unique_ptr<ZenohReply> latest_reply = std::move(reply_queue_.front());
  reply_queue_.pop_front();

  return latest_reply;
}

//==============================================================================
// See the comment about the "num_in_flight" class variable in the rmw_client_data_t class
// for the use of this method.
void rmw_client_data_t::increment_in_flight_callbacks()
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  num_in_flight_++;
}

//==============================================================================
// See the comment about the "num_in_flight" class variable in the rmw_client_data_t class
// for the use of this method.
bool rmw_client_data_t::shutdown_and_query_in_flight()
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  is_shutdown_ = true;

  return num_in_flight_ > 0;
}

//==============================================================================
// See the comment about the "num_in_flight" class variable in the
// rmw_client_data_t structure for the use of this method.
bool rmw_client_data_t::decrement_queries_in_flight_and_is_shutdown(bool & queries_in_flight)
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  queries_in_flight = --num_in_flight_ > 0;
  return is_shutdown_;
}

bool rmw_client_data_t::is_shutdown() const
{
  std::lock_guard<std::mutex> lock(in_flight_mutex_);
  return is_shutdown_;
}


///=============================================================================
size_t rmw_client_data_t::get_next_sequence_number()
{
  std::lock_guard<std::mutex> lock(sequence_number_mutex_);
  return sequence_number_++;
}

//==============================================================================
void client_data_handler(z_loaned_reply_t * reply, void * data)
{
  auto client_data = static_cast<rmw_client_data_t *>(data);
  if (client_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t "
    );
    return;
  }

  // See the comment about the "num_in_flight" class variable in the rmw_client_data_t class for
  // why we need to do this.
  if (client_data->is_shutdown()) {
    return;
  }

  if (z_reply_is_ok(reply)) {
    z_owned_reply_t owned_reply;
    z_reply_clone(&owned_reply, reply);
    client_data->add_new_reply(std::make_unique<ZenohReply>(owned_reply));
  } else {
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_loan(client_data->keyexpr), &keystr);
    const z_loaned_reply_err_t * err = z_reply_err(reply);
    const z_loaned_bytes_t * err_payload = z_reply_err_payload(err);

    z_owned_string_t err_str;
    z_bytes_to_string(err_payload, &err_str);
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "z_reply_is_ok returned False for keyexpr %s. Reason: %.*s",
      z_string_data(z_loan(keystr)), static_cast<int>(z_string_len(z_loan(err_str))),
      z_string_data(z_loan(err_str)));
    z_drop(z_move(err_str));
    return;
  }

  std::chrono::nanoseconds::rep received_timestamp =
    std::chrono::system_clock::now().time_since_epoch().count();

  client_data->add_new_reply(std::make_unique<ZenohReply>(reply, received_timestamp));
  // Since we took ownership of the reply, null it out here
  *reply = z_reply_null();
}

void client_data_drop(void * data)
{
  auto client_data = static_cast<rmw_client_data_t *>(data);
  if (client_data == nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to obtain client_data_t "
    );
    return;
  }

  // See the comment about the "num_in_flight" class variable in the rmw_client_data_t class for
  // why we need to do this.
  bool queries_in_flight = false;
  bool is_shutdown = client_data->decrement_queries_in_flight_and_is_shutdown(queries_in_flight);

  if (is_shutdown) {
    if (!queries_in_flight) {
      RMW_TRY_DESTRUCTOR(client_data->~rmw_client_data_t(), rmw_client_data_t, );
      client_data->context->options.allocator.deallocate(
        client_data, client_data->context->options.allocator.state);
    }
  }
}

}  // namespace rmw_zenoh_cpp
