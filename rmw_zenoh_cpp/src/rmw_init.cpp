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

#include <string>
#include <thread>

#include <zenoh.hxx>

#include "detail/identifier.hpp"
#include "detail/liveliness_utils.hpp"
#include "detail/rmw_context_impl_s.hpp"
#include "detail/zenoh_config.hpp"

#include "rcutils/env.h"
#include "detail/logging_macros.hpp"
#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/init.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init_options.h"

#include "rcpputils/scope_exit.hpp"

extern "C"
{
//==============================================================================
/// Initialize the middleware with the given options, and yield a context.
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->implementation_identifier,
    "expected initialized init options",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    options->enclave,
    "expected non-null enclave",
    return RMW_RET_INVALID_ARGUMENT);
  if (NULL != context->implementation_identifier) {
    RMW_SET_ERROR_MSG("expected a zero-initialized context");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto restore_context = rcpputils::make_scope_exit(
    [context]() {*context = rmw_get_zero_initialized_context();});

  context->instance_id = options->instance_id;
  context->implementation_identifier = rmw_zenoh_cpp::rmw_zenoh_identifier;
  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  context->actual_domain_id =
    RMW_DEFAULT_DOMAIN_ID != options->domain_id ? options->domain_id : 0u;

  const rcutils_allocator_t * allocator = &options->allocator;

  rmw_ret_t ret;
  if ((ret = rmw_init_options_copy(options, &context->options)) != RMW_RET_OK) {
    // error already set
    return ret;
  }
  auto free_options = rcpputils::make_scope_exit(
    [context]() {
      rmw_ret_t ret = rmw_init_options_fini(&context->options);
      if (ret != RMW_RET_OK) {
        RMW_SAFE_FWRITE_TO_STDERR("Failed to cleanup context options during error handling");
      }
    });

  if (!rcutils_set_env_overwrite(ZENOH_LOG_ENV_VAR_STR, ZENOH_LOG_WARN_LEVEL_STR, 0)) {
    RMW_SET_ERROR_MSG("Error configuring Zenoh logging.");
    return RMW_RET_ERROR;
  }

  // Enable the zenoh built-in logger
  zc_try_init_log_from_env();

  // Create the context impl.
  context->impl = static_cast<rmw_context_impl_t *>(
    allocator->zero_allocate(1, sizeof(rmw_context_impl_t), allocator->state));
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "failed to allocate context impl",
    return RMW_RET_BAD_ALLOC);
  auto free_impl = rcpputils::make_scope_exit(
    [context, allocator]() {
      allocator->deallocate(context->impl, allocator->state);
    });

  RMW_TRY_PLACEMENT_NEW(
    context->impl,
    context->impl,
    return RMW_RET_BAD_ALLOC,
    rmw_context_impl_t,
    context->actual_domain_id,
    std::string(options->enclave),
    &context->options.security_options
  );

  free_options.cancel();
  free_impl.cancel();
  restore_context.cancel();

  return RMW_RET_OK;
}

//==============================================================================
/// Shutdown the middleware for a given context.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(context->impl);
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, RMW_RET_INVALID_ARGUMENT);

  return context_impl->shutdown();
}

//==============================================================================
/// Finalize a context.
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_zenoh_cpp::rmw_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!context->impl->is_shutdown()) {
    RCUTILS_SET_ERROR_MSG("context has not been shutdown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  const rcutils_allocator_t * allocator = &context->options.allocator;

  RMW_TRY_DESTRUCTOR(context->impl->~rmw_context_impl_t(), rmw_context_impl_t *, );

  allocator->deallocate(context->impl, allocator->state);

  rmw_ret_t ret = rmw_init_options_fini(&context->options);

  *context = rmw_get_zero_initialized_context();

  return ret;
}
}  // extern "C"
