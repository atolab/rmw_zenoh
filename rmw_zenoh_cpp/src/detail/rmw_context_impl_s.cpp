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

#include "rmw_context_impl_s.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <zenoh.hxx>

#include "graph_cache.hpp"
#include "guard_condition.hpp"
#include "identifier.hpp"
#include "logging_macros.hpp"
#include "rmw_node_data.hpp"
#include "zenoh_config.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"
#include "rmw_dds_common/security.hpp"
#include "zenoh_utils.hpp"

// Megabytes of SHM to reserve.
// TODO(clalancette): Make this configurable, or get it from the configuration
#define SHM_BUFFER_SIZE_MB 10

// Bundle all class members into a data struct which can be passed as a
// weak ptr to various threads for thread-safe access without capturing
// "this" ptr by reference.
class rmw_context_impl_s::Data final : public std::enable_shared_from_this<Data>
{
public:
  // Constructor.
  Data(
    std::size_t domain_id,
    const std::string & enclave,
    const rmw_security_options_t * security_options)
  : domain_id_(std::move(domain_id)),
    enclave_(std::move(enclave)),
    is_shutdown_(false),
    next_entity_id_(0),
    nodes_({}),
    liveliness_keyexpr_(rmw_zenoh_cpp::liveliness::subscription_token(domain_id))
  {
    // Initialize the zenoh configuration.
    std::optional<zenoh::Config> config = rmw_zenoh_cpp::get_z_config(
      rmw_zenoh_cpp::ConfigurableEntity::Session);

    if (!config.has_value()) {
      throw std::runtime_error("Error configuring Zenoh session.");
    }
#ifdef HAVE_SECURITY
    std::unordered_map<std::string, std::string> security_files_paths;
    if (rmw_dds_common::get_security_files(
        true, "", security_options->security_root_path, security_files_paths))
    {
      // TODO(ahcorde): Fill this
    } else {
      std::cout << "Error getting secutiry data" << std::endl;
    }
#endif
    zenoh::ZResult result;

#ifndef _MSC_VER
    // Check if shm is enabled.
    std::string shm_enabled = config.value().get(Z_CONFIG_SHARED_MEMORY_KEY, &result);
    if (result != Z_OK) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Not able to get %s from the config file",
        Z_CONFIG_SHARED_MEMORY_KEY);
    }
#endif

    // Initialize the zenoh session.
    session_ = std::make_shared<zenoh::Session>(
      std::move(config.value()),
      zenoh::Session::SessionOptions::create_default(),
      &result);
    if (result != Z_OK) {
      throw std::runtime_error("Error setting up zenoh session. ");
    }

    // Verify if the zenoh router is running if configured.
    const std::optional<uint64_t> configured_connection_attempts =
      rmw_zenoh_cpp::zenoh_router_check_attempts();
    if (configured_connection_attempts.has_value()) {
      uint64_t connection_attempts = 0;
      // Retry until the connection is successful.
      constexpr std::chrono::milliseconds sleep_time(1000);
      constexpr int64_t ticks_between_print(std::chrono::milliseconds(1000) / sleep_time);
      do {
        zenoh::ZResult result;
        this->session_->get_routers_z_id(&result);
        if (result == Z_OK) {
          break;
        }
        if ((connection_attempts % ticks_between_print) == 0) {
          RMW_ZENOH_LOG_WARN_NAMED(
            "rmw_zenoh_cpp",
            "Unable to connect to a Zenoh router. "
            "Have you started a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`?");
        }
        if (++connection_attempts >= configured_connection_attempts.value()) {
          break;
        }
        std::this_thread::sleep_for(sleep_time);
      } while(connection_attempts < configured_connection_attempts.value());
    }

    // Initialize the graph cache.
    graph_cache_ =
      std::make_shared<rmw_zenoh_cpp::GraphCache>(this->session_->get_zid());
    // Setup liveliness subscriptions for discovery.
    // Query router/liveliness participants to get graph information before the session was started.
    // We create a blocking channel that is unbounded, ie. `bound` = 0, to receive
    // replies for the z_liveliness_get() call. This is necessary as if the `bound`
    // is too low, the channel may starve the zenoh executor of its threads which
    // would lead to deadlocks when trying to receive replies and block the
    // execution here.
    // The blocking channel will return when the sender end is closed which is
    // the moment the query finishes.
    // The non-blocking fifo exists only for the use case where we don't want to
    // block the thread between responses (including the request termination response).
    // In general, unless we want to cooperatively schedule other tasks on the same
    // thread as reading the fifo, the blocking fifo will be more appropriate as
    // the code will be simpler, and if we're just going to spin over the non-blocking
    // reads until we obtain responses, we'll just be hogging CPU time by convincing
    // the OS that we're doing actual work when it could instead park the thread.

    // Intuitively use a FIFO channel rather than building it up with a closure and a
    // handler to FIFO channel
    zenoh::Session::GetOptions options = zenoh::Session::GetOptions::create_default();
    options.target = zenoh::QueryTarget::Z_QUERY_TARGET_ALL;
    options.payload = "";

    auto replies = session_->liveliness_get(
      liveliness_keyexpr_,
      zenoh::channels::FifoChannel(SIZE_MAX - 1),
      zenoh::Session::LivelinessGetOptions::create_default(),
      &result);

    if (result != Z_OK) {
      throw std::runtime_error("Error getting liveliness. ");
    }

    for (auto res = replies.recv(); std::holds_alternative<zenoh::Reply>(res);
      res = replies.recv())
    {
      const zenoh::Reply & reply = std::get<zenoh::Reply>(res);
      if (reply.is_ok()) {
        const auto & sample = reply.get_ok();
        graph_cache_->parse_put(std::string(sample.get_keyexpr().as_string_view()), true);
      } else {
        RMW_ZENOH_LOG_DEBUG_NAMED(
          "rmw_zenoh_cpp", "[rmw_context_impl_s] z_call received an invalid reply.\n");
      }
    }

    // Initialize the shm manager if shared_memory is enabled in the config.
    shm_provider_ = std::nullopt;
#ifndef _MSC_VER
    if (shm_enabled == "true") {
      auto layout = zenoh::MemoryLayout(
        SHM_BUFFER_SIZE_MB * 1024 * 1024,
        zenoh::AllocAlignment({5}));
      zenoh::PosixShmProvider provider(layout, &result);
      if (result != Z_OK) {
        throw std::runtime_error("Unable to create shm provider.");
      }
      shm_provider_ = std::move(provider);
    }
#endif
    graph_guard_condition_ = std::make_unique<rmw_guard_condition_t>();
    graph_guard_condition_->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
    graph_guard_condition_->data = &guard_condition_data_;
  }

  void init()
  {
    // Setup the liveliness subscriber to receives updates from the ROS graph
    // and update the graph cache.
    zenoh::Session::LivelinessSubscriberOptions sub_options =
      zenoh::Session::LivelinessSubscriberOptions::create_default();
    sub_options.history = true;

    // This can't be in the constructor since shared_from_this() doesn't work there.
    std::weak_ptr<Data> context_impl_data_wp = shared_from_this();
    zenoh::ZResult result;
    graph_subscriber_ = session_->liveliness_declare_subscriber(
      liveliness_keyexpr_,
      [context_impl_data_wp](const zenoh::Sample & sample) {
        // Update the graph cache.
        std::shared_ptr<Data> context_impl_data = context_impl_data_wp.lock();
        if (context_impl_data == nullptr) {
          RMW_ZENOH_LOG_ERROR_NAMED("rmw_zenoh_cpp", "Unable to obtain context_impl.");
          return;
        }
        context_impl_data->update_graph_cache(
          sample,
          std::string(sample.get_keyexpr().as_string_view()));
      },
      zenoh::closures::none,
      std::move(sub_options),
      &result);

    if (result != Z_OK) {
      RMW_SET_ERROR_MSG("unable to create zenoh subscription");
      throw std::runtime_error("Unable to subscribe to ROS graph updates.");
    }
  }

  // Shutdown the Zenoh session.
  rmw_ret_t shutdown()
  {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      rmw_ret_t ret = RMW_RET_OK;
      if (is_shutdown_) {
        return ret;
      }

      zenoh::ZResult result;
      std::move(graph_subscriber_).value().undeclare(&result);
      if (result != Z_OK) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp",
          "Unable to undeclare liveliness token");
        return RMW_RET_ERROR;
      }

      is_shutdown_ = true;

      // We specifically do *not* hold the mutex_ while tearing down the session; this allows us
      // to avoid an AB/BA deadlock if shutdown is racing with graph_sub_data_handler().
    }

    // Drop the shared session.
    session_.reset();

    return RMW_RET_OK;
  }

  std::string enclave() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return enclave_;
  }

  const std::shared_ptr<zenoh::Session> session() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return session_;
  }

  std::optional<zenoh::ShmProvider> & shm_provider()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return shm_provider_;
  }

  rmw_guard_condition_t * graph_guard_condition()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return graph_guard_condition_.get();
  }

  std::size_t get_next_entity_id()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return next_entity_id_++;
  }

  bool is_shutdown() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return is_shutdown_;
  }

  bool session_is_valid() const
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return !session_->is_closed();
  }

  std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return graph_cache_;
  }

  bool create_node_data(
    const rmw_node_t * const node,
    const std::string & ns,
    const std::string & node_name)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (nodes_.count(node) > 0) {
      // Node already exists.
      return false;
    }

    // Check that the Zenoh session is still valid.
    if (!session_is_valid()) {
      RMW_ZENOH_LOG_ERROR_NAMED(
        "rmw_zenoh_cpp",
        "Unable to create NodeData as Zenoh session is invalid.");
      return false;
    }

    auto node_data = rmw_zenoh_cpp::NodeData::make(
      node,
      this->get_next_entity_id(),
      session(),
      domain_id_,
      ns,
      node_name,
      enclave_);
    if (node_data == nullptr) {
      // Error already handled.
      return false;
    }

    auto node_insertion = nodes_.insert(std::make_pair(node, std::move(node_data)));
    if (!node_insertion.second) {
      return false;
    }

    return true;
  }

  std::shared_ptr<rmw_zenoh_cpp::NodeData> get_node_data(const rmw_node_t * const node)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto node_it = nodes_.find(node);
    if (node_it == nodes_.end()) {
      return nullptr;
    }
    return node_it->second;
  }

  void delete_node_data(const rmw_node_t * const node)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    nodes_.erase(node);
  }

  void update_graph_cache(const zenoh::Sample & sample_kind, const std::string & keystr)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (is_shutdown_) {
      return;
    }
    switch (sample_kind.get_kind()) {
      case zenoh::SampleKind::Z_SAMPLE_KIND_PUT:
        graph_cache_->parse_put(keystr);
        break;
      case zenoh::SampleKind::Z_SAMPLE_KIND_DELETE:
        graph_cache_->parse_del(keystr);
        break;
      default:
        return;
    }

    // Trigger the ROS graph guard condition.
    rmw_ret_t rmw_ret = rmw_trigger_guard_condition(graph_guard_condition_.get());
    if (RMW_RET_OK != rmw_ret) {
      RMW_ZENOH_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "[update_graph_cache] Unable to trigger graph guard condition."
      );
    }
  }

  // Destructor.
  ~Data()
  {
    auto ret = this->shutdown();
    nodes_.clear();
    static_cast<void>(ret);
  }

private:
  // Mutex to lock when accessing members.
  mutable std::recursive_mutex mutex_;
  // The ROS domain id of this context.
  std::size_t domain_id_;
  // Enclave, name used to find security artifacts in a sros2 keystore.
  std::string enclave_;
  // An owned session.
  std::shared_ptr<zenoh::Session> session_;
  // An optional SHM manager that is initialized of SHM is enabled in the
  // zenoh session config.
  std::optional<zenoh::ShmProvider> shm_provider_;
  // Graph cache.
  std::shared_ptr<rmw_zenoh_cpp::GraphCache> graph_cache_;
  // ROS graph liveliness subscriber.
  // The graph_subscriber *must* exist in order for anything in this Data class,
  // and hence rmw_zenoh_cpp, to work.
  // However, zenoh::Subscriber does not have an empty constructor,
  // so just declaring this as a zenoh::Subscriber fails to compile.
  // We work around that by wrapping it in a std::optional,
  // so the std::optional gets constructed at Data constructor time,
  // and then we initialize graph_subscriber_ later. Note that the zenoh-cpp API
  // liveliness_declare_subscriber() throws an exception if it fails,
  // so this should all be safe to do.
  std::optional<zenoh::Subscriber<void>> graph_subscriber_;
  // Equivalent to rmw_dds_common::Context's guard condition.
  // Guard condition that should be triggered when the graph changes.
  std::unique_ptr<rmw_guard_condition_t> graph_guard_condition_;
  // The GuardCondition data structure.
  rmw_zenoh_cpp::GuardCondition guard_condition_data_;
  // Shutdown flag.
  bool is_shutdown_;
  // A counter to assign a local id for every entity created in this session.
  std::size_t next_entity_id_;
  // Nodes created from this context.
  std::unordered_map<const rmw_node_t *, std::shared_ptr<rmw_zenoh_cpp::NodeData>> nodes_;

  zenoh::KeyExpr liveliness_keyexpr_;
};

///=============================================================================
rmw_context_impl_s::rmw_context_impl_s(
  const std::size_t domain_id,
  const std::string & enclave,
  const rmw_security_options_t * security_options)
{
  data_ = std::make_shared<Data>(domain_id, std::move(enclave), security_options);
  data_->init();
}

///=============================================================================
rmw_context_impl_s::~rmw_context_impl_s()
{
  this->shutdown();
}

///=============================================================================
std::string rmw_context_impl_s::enclave() const
{
  return data_->enclave();
}

///=============================================================================
const std::shared_ptr<zenoh::Session> rmw_context_impl_s::session() const
{
  return data_->session();
}

///=============================================================================
std::optional<zenoh::ShmProvider> & rmw_context_impl_s::shm_provider()
{
  return data_->shm_provider();
}

///=============================================================================
rmw_guard_condition_t * rmw_context_impl_s::graph_guard_condition()
{
  return data_->graph_guard_condition();
}

///=============================================================================
std::size_t rmw_context_impl_s::get_next_entity_id()
{
  return data_->get_next_entity_id();
}

///=============================================================================
rmw_ret_t rmw_context_impl_s::shutdown()
{
  return data_->shutdown();
}

///=============================================================================
bool rmw_context_impl_s::is_shutdown() const
{
  return data_->is_shutdown();
}

///=============================================================================
bool rmw_context_impl_s::session_is_valid() const
{
  return data_->session_is_valid();
}

///=============================================================================
std::shared_ptr<rmw_zenoh_cpp::GraphCache> rmw_context_impl_s::graph_cache()
{
  return data_->graph_cache();
}

///=============================================================================
bool rmw_context_impl_s::create_node_data(
  const rmw_node_t * const node,
  const std::string & ns,
  const std::string & node_name)
{
  return data_->create_node_data(node, ns, node_name);
}

///=============================================================================
std::shared_ptr<rmw_zenoh_cpp::NodeData> rmw_context_impl_s::get_node_data(
  const rmw_node_t * const node)
{
  return data_->get_node_data(node);
}

///=============================================================================
void rmw_context_impl_s::delete_node_data(const rmw_node_t * const node)
{
  data_->delete_node_data(node);
}
