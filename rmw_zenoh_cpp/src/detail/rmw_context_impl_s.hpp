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

#ifndef DETAIL__RMW_CONTEXT_IMPL_S_HPP_
#define DETAIL__RMW_CONTEXT_IMPL_S_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include <zenoh.hxx>

#include "graph_cache.hpp"
#include "rmw_node_data.hpp"
#include "buffer_pool.hpp"

#include "rmw/ret_types.h"
#include "rmw/types.h"

///=============================================================================
struct rmw_context_impl_s final
{
public:
  // Constructor that internally initializes the Zenoh session and other artifacts.
  // Throws an std::runtime_error if any of the initializations fail.
  // The construction will block until a Zenoh router is detected.
  // TODO(Yadunund): Make this a non-blocking call by checking for the Zenoh
  // router in a separate thread. Instead block when creating a node if router
  // check has not succeeded.
  rmw_context_impl_s(
    const std::size_t domain_id,
    const std::string & enclave);

  ~rmw_context_impl_s();

  // Get a copy of the enclave.
  std::string enclave() const;

  // Loan the Zenoh session.
  const std::shared_ptr<zenoh::Session> session() const;

  // Get a reference to the shm_provider.
  // Note: This is not thread-safe.
  // TODO(Yadunund): Remove this API and instead include a publish() API
  // that handles the shm_provider once the context manages publishers.
  std::optional<zenoh::ShmProvider> & shm_provider();

  // Get the graph guard condition.
  rmw_guard_condition_t * graph_guard_condition();

  // Get a unique id for a new entity.
  std::size_t get_next_entity_id();

  // Shutdown the Zenoh session.
  rmw_ret_t shutdown();

  // Check if the Zenoh session is shutdown.
  bool is_shutdown() const;

  // Returns true if the Zenoh session is valid.
  bool session_is_valid() const;

  /// Return a shared_ptr to the GraphCache stored in this context.
  std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache();

  /// Create a NodeData and store it within this context. The NodeData can be
  /// retrieved using get_node().
  /// Returns false if parameters are invalid.
  bool create_node_data(
    const rmw_node_t * const node,
    const std::string & ns,
    const std::string & node_name);

  /// Retrieve the NodeData for a given rmw_node_t if present.
  std::shared_ptr<rmw_zenoh_cpp::NodeData> get_node_data(
    const rmw_node_t * const node);

  /// Delete the NodeData for a given rmw_node_t if present.
  void delete_node_data(const rmw_node_t * const node);

  // Forward declaration
  class Data;

  // Pool of serialization buffers.
  rmw_zenoh_cpp::BufferPool serialization_buffer_pool;

private:
  std::shared_ptr<Data> data_{nullptr};
};

#endif  // DETAIL__RMW_CONTEXT_IMPL_S_HPP_
