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

#include <zenoh.h>

#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "rmw/types.h"

#include "attachment_helpers.hpp"
#include "liveliness_utils.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
AttachmentData::AttachmentData(
  const int64_t sequence_number,
  const int64_t source_timestamp,
  const uint8_t source_gid[16])
: sequence_number_(sequence_number),
  source_timestamp_(source_timestamp)
{
  memcpy(source_gid_, source_gid, 16);
  gid_hash_ = hash_gid(source_gid_);
}

///=============================================================================
AttachmentData::AttachmentData(AttachmentData && data)
: sequence_number_(std::move(data.sequence_number_)),
  source_timestamp_(std::move(data.source_timestamp_)),
  gid_hash_(std::move(data.gid_hash_))
{
  memcpy(source_gid_, data.source_gid_, 16);
}

///=============================================================================
AttachmentData::AttachmentData(const z_loaned_bytes_t * attachment)
{
  ze_deserializer_t deserializer = ze_deserializer_from_bytes(attachment);
  z_owned_string_t key;

  // Deserialize the sequence_number
  ze_deserializer_deserialize_string(&deserializer, &key);
  if (std::string_view(
      z_string_data(z_loan(key)),
      z_string_len(z_loan(key))) != "sequence_number")
  {
    throw std::runtime_error("sequence_number is not found in the attachment.");
  }
  z_drop(z_move(key));
  if (ze_deserializer_deserialize_int64(&deserializer, &this->sequence_number_)) {
    throw std::runtime_error("Failed to deserialize the sequence_number.");
  }

  // Deserialize the source_timestamp
  ze_deserializer_deserialize_string(&deserializer, &key);
  if (std::string_view(
      z_string_data(z_loan(key)),
      z_string_len(z_loan(key))) != "source_timestamp")
  {
    throw std::runtime_error("source_timestamp is not found in the attachment");
  }
  z_drop(z_move(key));
  if (ze_deserializer_deserialize_int64(&deserializer, &this->source_timestamp_)) {
    throw std::runtime_error("Failed to deserialize the source_timestamp.");
  }

  // Deserialize the source_gid
  ze_deserializer_deserialize_string(&deserializer, &key);
  if (std::string_view(z_string_data(z_loan(key)), z_string_len(z_loan(key))) != "source_gid") {
    throw std::runtime_error("Invalid attachment: the key source_gid is not found");
  }
  z_drop(z_move(key));
  z_owned_slice_t slice;
  if (ze_deserializer_deserialize_slice(&deserializer, &slice)) {
    throw std::runtime_error("Failed to deserialize the source_gid.");
  }
  if (z_slice_len(z_loan(slice)) != 16) {
    throw std::runtime_error("The length of source_gid mismatched.");
  }
  memcpy(this->source_gid_, z_slice_data(z_loan(slice)), z_slice_len(z_loan(slice)));
  z_drop(z_move(slice));
  gid_hash_ = hash_gid(this->source_gid_);
}

///=============================================================================
int64_t AttachmentData::sequence_number() const
{
  return sequence_number_;
}

///=============================================================================
int64_t AttachmentData::source_timestamp() const
{
  return source_timestamp_;
}

///=============================================================================
void AttachmentData::copy_gid(uint8_t out_gid[16]) const
{
  memcpy(out_gid, source_gid_, 16);
}

///=============================================================================
size_t AttachmentData::gid_hash() const
{
  return gid_hash_;
}

///=============================================================================
void AttachmentData::serialize_to_zbytes(z_owned_bytes_t * attachment)
{
  ze_owned_serializer_t serializer;
  ze_serializer_empty(&serializer);
  ze_serializer_serialize_str(z_loan_mut(serializer), "sequence_number");
  ze_serializer_serialize_int64(z_loan_mut(serializer), this->sequence_number_);
  ze_serializer_serialize_str(z_loan_mut(serializer), "source_timestamp");
  ze_serializer_serialize_int64(z_loan_mut(serializer), this->source_timestamp_);
  ze_serializer_serialize_str(z_loan_mut(serializer), "source_gid");
  ze_serializer_serialize_buf(z_loan_mut(serializer), this->source_gid_, 16);
  ze_serializer_finish(z_move(serializer), attachment);
}
}  // namespace rmw_zenoh_cpp
