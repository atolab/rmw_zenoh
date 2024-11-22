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

#include "zenoh_utils.hpp"

#include <chrono>

#include "attachment_helpers.hpp"
#include "rmw/types.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
zenoh::Bytes create_map_and_set_sequence_num(
  int64_t sequence_number, uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now);
  int64_t source_timestamp = now_ns.count();

  rmw_zenoh_cpp::AttachmentData data(sequence_number, source_timestamp, gid);
  return std::move(data.serialize_to_zbytes());
}

///=============================================================================
ZenohQuery::ZenohQuery(const zenoh::Query & query, std::chrono::nanoseconds::rep received_timestamp)
{
  query_ = query.clone();
  received_timestamp_ = received_timestamp;
}

///=============================================================================
std::chrono::nanoseconds::rep ZenohQuery::get_received_timestamp() const
{
  return received_timestamp_;
}

///=============================================================================
ZenohQuery::~ZenohQuery() {}

///=============================================================================
const zenoh::Query & ZenohQuery::get_query() const {return query_.value();}

///=============================================================================
ZenohReply::ZenohReply(
  const zenoh::Reply & reply,
  std::chrono::nanoseconds::rep received_timestamp)
{
  reply_ = reply.clone();
  received_timestamp_ = received_timestamp;
}

///=============================================================================
ZenohReply::~ZenohReply() {}

///=============================================================================
const zenoh::Reply & ZenohReply::get_sample() const
{
  return reply_.value();
}

///=============================================================================
std::chrono::nanoseconds::rep ZenohReply::get_received_timestamp() const
{
  return received_timestamp_;
}
}  // namespace rmw_zenoh_cpp
