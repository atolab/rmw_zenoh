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

#include "zenoh_config.hpp"

#include <rcutils/env.h>

#include <limits>
#include <string>

#include <zenoh.hxx>
#include <zenoh/api/config.hxx>

#include "logging_macros.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rmw/impl/cpp/macros.hpp>

///=============================================================================
namespace rmw_zenoh_cpp
{
namespace
{
/// Map the configurable entity to a pair of environment variable name that
/// stores the absolute path to the Zenoh config and the default config filename.
/// Note: The default config file should be located within rmw_zenoh_cpp/config/.
static const std::unordered_map<ConfigurableEntity,
  std::pair<const char *, const char *>> envar_map = {
  {ConfigurableEntity::Session,
    {"ZENOH_SESSION_CONFIG_URI", "DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"}},
  {ConfigurableEntity::Router, {"ZENOH_ROUTER_CONFIG_URI", "DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"}}
};

static const char * router_check_attempts_envar = "ZENOH_ROUTER_CHECK_ATTEMPTS";
#ifdef RMW_ZENOH_BUILD_WITH_SHARED_MEMORY
static const char * zenoh_shm_enabled_envar = "ZENOH_SHM_ENABLED";
static const bool zenoh_shm_enabled_default = true;
static const char * zenoh_shm_alloc_size_envar = "ZENOH_SHM_ALLOC_SIZE";
static const size_t zenoh_shm_alloc_size_default = 16 * 1024 * 1024;
static const char * zenoh_shm_message_size_threshold_envar = "ZENOH_SHM_MESSAGE_SIZE_THRESHOLD";
static const size_t zenoh_shm_message_size_threshold_default = 2 * 1024;
#endif

std::optional<zenoh::Config> _get_z_config(
  const char * envar_name,
  const char * default_uri)
{
  const char * configured_uri;
  const char * envar_uri;
  // Get the path to the zenoh configuration file from the environment variable.
  if (NULL != rcutils_get_env(envar_name, &envar_uri)) {
    // NULL is returned if everything is ok.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read.", envar_name);
    return std::nullopt;
  }
  // If the environment variable is set, try to read the configuration from the file,
  // if the environment variable is not set use internal configuration
  configured_uri = envar_uri[0] != '\0' ? envar_uri : default_uri;
  // Try to read the configuration
  zenoh::ZResult result;
  zenoh::Config config = zenoh::Config::from_file(configured_uri, &result);
  if (result != Z_OK) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp",
      "Invalid configuration file %s", configured_uri);
    return std::nullopt;
  }
  RMW_ZENOH_LOG_DEBUG_NAMED(
    "rmw_zenoh_cpp",
    "configured using configuration file %s", configured_uri);
  return config;
}
}  // namespace

///=============================================================================
std::optional<zenoh::Config> get_z_config(const ConfigurableEntity & entity)
{
  auto envar_map_it = envar_map.find(entity);
  if (envar_map_it == envar_map.end()) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "get_z_config called with invalid ConfigurableEntity.");
    return std::nullopt;
  }
  // Get the absolute path to the default configuration file.
  static const std::string path_to_config_folder =
    ament_index_cpp::get_package_share_directory("rmw_zenoh_cpp") + "/config/";
  const std::string default_config_path = path_to_config_folder + envar_map_it->second.second;

  return _get_z_config(envar_map_it->second.first, default_config_path.c_str());
}

///=============================================================================
std::optional<uint64_t> zenoh_router_check_attempts()
{
  const char * envar_value;
  // The default is to check only once.
  uint64_t default_value = 1;

  if (NULL != rcutils_get_env(router_check_attempts_envar, &envar_value)) {
    // NULL is returned if everything is ok.
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read. Report this bug.",
      router_check_attempts_envar);
    return default_value;
  }
  // If the environment variable contains a value, handle it accordingly.
  if (envar_value[0] != '\0') {
    const int64_t read_value = std::strtoll(envar_value, nullptr, 10);
    if (read_value > 0) {
      return read_value;
    } else if (read_value < 0) {
      // If less than 0, we skip the check.
      return std::nullopt;
    }
    // If the value is 0, check indefinitely.
    return std::numeric_limits<uint64_t>::max();
  }

  // If unset, use the default.
  return default_value;
}

#ifdef RMW_ZENOH_BUILD_WITH_SHARED_MEMORY
///=============================================================================
bool zenoh_shm_enabled()
{
  const char * envar_value;

  if (NULL != rcutils_get_env(zenoh_shm_enabled_envar, &envar_value)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read. Report this bug.",
      zenoh_shm_enabled_envar);
    return zenoh_shm_enabled_default;
  }

  if (strlen(envar_value) == strlen("true") &&
    strncmp(envar_value, "true", strlen(envar_value)) == 0)
  {
    return true;
  }

  if (strlen(envar_value) == strlen("false") &&
    strncmp(envar_value, "false", strlen(envar_value)) == 0)
  {
    return false;
  }

  return zenoh_shm_enabled_default;
}
///=============================================================================
size_t zenoh_shm_alloc_size()
{
  const char * envar_value;

  if (NULL != rcutils_get_env(zenoh_shm_alloc_size_envar, &envar_value)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read. Report this bug.",
      zenoh_shm_alloc_size_envar);
    return zenoh_shm_alloc_size_default;
  }

  // If the environment variable contains a value, handle it accordingly.
  if (envar_value[0] != '\0') {
    const auto read_value = std::strtoull(envar_value, nullptr, 10);
    if (read_value > 0) {
      if (read_value > std::numeric_limits<size_t>::max()) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp", "Envar %s: value is too large!",
          zenoh_shm_alloc_size_envar);
      } else {
        return read_value;
      }
    }
  }

  return zenoh_shm_alloc_size_default;
}
///=============================================================================
size_t zenoh_shm_message_size_threshold()
{
  const char * envar_value;

  if (NULL != rcutils_get_env(zenoh_shm_message_size_threshold_envar, &envar_value)) {
    RMW_ZENOH_LOG_ERROR_NAMED(
      "rmw_zenoh_cpp", "Envar %s cannot be read. Report this bug.",
      zenoh_shm_message_size_threshold_envar);
    return zenoh_shm_message_size_threshold_default;
  }

  // If the environment variable contains a value, handle it accordingly.
  if (envar_value[0] != '\0') {
    const auto read_value = std::strtoull(envar_value, nullptr, 10);
    if (read_value > 0) {
      if (read_value > std::numeric_limits<size_t>::max()) {
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp", "Envar %s: value is too large!",
          zenoh_shm_message_size_threshold_envar);
      } else if ((read_value & (read_value - 1)) != 0) {  // power of 2 check
        RMW_ZENOH_LOG_ERROR_NAMED(
          "rmw_zenoh_cpp", "Envar %s: value must be power of 2!",
          zenoh_shm_message_size_threshold_envar);

      } else {
        return read_value;
      }
    }
  }

  return zenoh_shm_message_size_threshold_default;
}
#endif
}  // namespace rmw_zenoh_cpp
