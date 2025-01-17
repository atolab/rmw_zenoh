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

#ifndef DETAIL__SHM_CONTEXT_HPP_
#define DETAIL__SHM_CONTEXT_HPP_

#ifdef RMW_ZENOH_BUILD_WITH_SHARED_MEMORY

#include <cstddef>

#include <zenoh.hxx>

namespace rmw_zenoh_cpp
{
///=============================================================================
struct ShmContext
{
  zenoh::PosixShmProvider shm_provider;
  size_t msgsize_threshold;

  ShmContext(size_t alloc_size, size_t msgsize_threshold);
};
}  // namespace rmw_zenoh_cpp

#endif

#endif  // DETAIL__SHM_CONTEXT_HPP_
