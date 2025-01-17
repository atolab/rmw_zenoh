cmake_minimum_required(VERSION 3.16)
project(zenoh_cpp_vendor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Disable default features and enable only the most useful ones. This reduces build time and footprint.
# For a complete list of features see: https://github.com/eclipse-zenoh/zenoh/blob/main/zenoh/Cargo.toml
# Note: We separate the two args needed for cargo with "$<SEMICOLON>" and not ";" as the
# latter is a list separater in cmake and hence the string will be split into two
# when expanded.
set(ZENOHC_CARGO_FLAGS "--no-default-features$<SEMICOLON>--features=zenoh/transport_compression zenoh/transport_tcp zenoh/transport_tls")

set(RMW_ZENOH_BUILD_WITH_SHARED_MEMORY TRUE CACHE BOOL "Compile Zenoh RMW with Shared Memory support")

# Set VCS_VERSION to include latest changes from zenoh/zenoh-c to benefit from :
# - https://github.com/eclipse-zenoh/zenoh/pull/1685 (Fix deadlock in advanced subscription undeclaration)
# - https://github.com/eclipse-zenoh/zenoh/pull/1696 (Fix SHM Garbage Collection (GC) policy)
# - https://github.com/eclipse-zenoh/zenoh/pull/1708 (Fix gossip with TLS endpoints)
# - https://github.com/eclipse-zenoh/zenoh/pull/1717 (Improve performance of a large number of peers)
ament_vendor(zenoh_c_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-c.git
  VCS_VERSION 328736fe9bb9b654b1d9f47eecfc6d52f0d7d587
  CMAKE_ARGS
    "-DZENOHC_CARGO_FLAGS=${ZENOHC_CARGO_FLAGS}"
    "-DZENOHC_BUILD_WITH_SHARED_MEMORY:BOOL=${RMW_ZENOH_BUILD_WITH_SHARED_MEMORY}"
    "-DZENOHC_BUILD_WITH_UNSTABLE_API=TRUE"
    "-DZENOHC_CUSTOM_TARGET=${ZENOHC_CUSTOM_TARGET}"
)

ament_export_dependencies(zenohc)

# Set VCS_VERSION to include latest changes from zenoh-cpp to benefit from :
# - https://github.com/eclipse-zenoh/zenoh-cpp/pull/342 (Fix include what you use)
# - https://github.com/eclipse-zenoh/zenoh-cpp/pull/363 (Fix memory leak in string deserialization)
ament_vendor(zenoh_cpp_vendor
  VCS_URL https://github.com/eclipse-zenoh/zenoh-cpp
  VCS_VERSION bbfef04e843289aae70b5aa060a925e8ee5b1b6f
  CMAKE_ARGS
      -DZENOHCXX_ZENOHC=ON
      -DZENOHCXX_ZENOHPICO=OFF
  )

externalproject_add_stepdependencies(zenoh_cpp_vendor configure zenoh_c_vendor)

ament_export_dependencies(zenohcxx)

ament_package()
