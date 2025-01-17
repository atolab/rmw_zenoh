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

#ifndef DETAIL__BUFFER_POOL_HPP_
#define DETAIL__BUFFER_POOL_HPP_

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
class BufferPool
{
public:
  struct Buffer
  {
    uint8_t * data;
    size_t size;
  };

  BufferPool();

  ~BufferPool();

  Buffer allocate(size_t size);

  void deallocate(Buffer buffer);

private:
  std::vector<Buffer> buffers_;
  std::mutex mutex_;
  size_t max_size_;
  size_t size_;
  // NOTE(fuzzypixelz): Pooled buffers are recycled with the expectation that they would reside in
  // cache, thus this this value should be comparable to the size of a modern CPU cache. The default
  // value (16 MiB) is relatively conservative as CPU cache sizes range from a few MiB to a few
  // hundred MiB.
  const size_t DEFAULT_MAX_SIZE = 16 * 1024 * 1024;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__BUFFER_POOL_HPP_
