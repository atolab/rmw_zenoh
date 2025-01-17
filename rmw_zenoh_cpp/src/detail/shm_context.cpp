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

#ifdef RMW_ZENOH_BUILD_WITH_SHARED_MEMORY

#include <stdexcept>

#include "shm_context.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
ShmContext::ShmContext(size_t alloc_size, size_t msgsize_threshold)
:  // Create Layout for provider's memory
   // Provider's alignment is 1 (=2^0) bytes as we are going to make only 1-byte aligned allocations
   // TODO(yellowhatter): use zenoh_shm_message_size_threshold as base for alignment
  shm_provider(zenoh::PosixShmProvider(zenoh::MemoryLayout(alloc_size, zenoh::AllocAlignment {0}))),
  msgsize_threshold(msgsize_threshold)
{}
}  // namespace rmw_zenoh_cpp

#endif
