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

#include "buffer_pool.hpp"

#include <cstddef>
#include <cstdlib>
#include <mutex>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/env.h"
#include "logging_macros.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
BufferPool::BufferPool()
: buffers_(), mutex_()
{
  const char * env_value;
  const char * error_str = rcutils_get_env("RMW_ZENOH_BUFFER_POOL_MAX_SIZE_BYTES", &env_value);
  if (error_str != nullptr) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Unable to read maximum buffer pool size, falling back to default.");
    max_size_ = DEFAULT_MAX_SIZE;
  } else {
    max_size_ = std::atoll(env_value);
  }
  size_ = 0;
}

///=============================================================================
BufferPool::~BufferPool()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  for (Buffer & buffer : buffers_) {
    allocator.deallocate(buffer.data, allocator.state);
  }
}

///=============================================================================
BufferPool::Buffer BufferPool::allocate(size_t size)
{
  std::lock_guard<std::mutex> guard(mutex_);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (buffers_.empty()) {
    if (size_ + size > max_size_) {
      return {};
    } else {
      size_ += size;
    }
    uint8_t * data = static_cast<uint8_t *>(allocator.allocate(size, allocator.state));
    if (data == nullptr) {
      return {};
    } else {
      return Buffer {data, size};
    }
  } else {
    Buffer buffer = buffers_.back();
    buffers_.pop_back();
    if (buffer.size < size) {
      size_t size_diff = size - buffer.size;
      if (size_ + size_diff > max_size_) {
        return {};
      }
      uint8_t * data = static_cast<uint8_t *>(allocator.reallocate(
          buffer.data, size, allocator.state));
      if (data == nullptr) {
        return {};
      }
      size_ += size_diff;
      buffer.data = data;
      buffer.size = size;
    }
    return buffer;
  }
}

///=============================================================================
void BufferPool::deallocate(BufferPool::Buffer buffer)
{
  std::lock_guard<std::mutex> guard(mutex_);
  buffers_.push_back(buffer);
}

}  // namespace rmw_zenoh_cpp
