cmake_minimum_required(VERSION 3.20)

project(waypoint
    VERSION 1.0.0
    DESCRIPTION "Real-time spatial routing and fleet dispatch engine"
    LANGUAGES CXX)

option(WAYPOINT_BUILD_TESTS "Build unit tests" ON)
option(WAYPOINT_BUILD_APPS "Build demo and benchmark executables" ON)
option(WAYPOINT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(WAYPOINT_SANITIZE "Build with AddressSanitizer and UBSan" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Shared compiler settings
# ---------------------------------------------------------------------------
add_library(waypoint_options INTERFACE)
target_compile_features(waypoint_options INTERFACE cxx_std_20)

if(MSVC)
    target_compile_options(waypoint_options INTERFACE /W4 /permissive-)
    if(WAYPOINT_WARNINGS_AS_ERRORS)
        target_compile_options(waypoint_options INTERFACE /WX)
    endif()
else()
    target_compile_options(waypoint_options INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
        -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual
        # Config structs are built with designated initializers that rely on
        # default member values; GCC would flag every omitted field.
        -Wno-missing-field-initializers)
    if(WAYPOINT_WARNINGS_AS_ERRORS)
        target_compile_options(waypoint_options INTERFACE -Werror)
    endif()
    if(WAYPOINT_SANITIZE)
        target_compile_options(waypoint_options INTERFACE
            -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
        target_link_options(waypoint_options INTERFACE -fsanitize=address,undefined)
    endif()
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(waypoint
    src/geo/geodesy.cpp
    src/graph/adjacency_index.cpp
    src/graph/road_network.cpp
    src/graph/road_network_builder.cpp
    src/routing/shortest_path_engine.cpp
    src/spatial/quad_tree.cpp
    src/spatial/node_locator.cpp
    src/dispatch/fleet_index.cpp
    src/dispatch/matcher.cpp
    src/dispatch/greedy_matcher.cpp
    src/dispatch/hungarian_matcher.cpp
    src/dispatch/dispatcher.cpp
    src/sim/city_generator.cpp)
add_library(waypoint::waypoint ALIAS waypoint)

target_include_directories(waypoint PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(waypoint PUBLIC waypoint_options)

# ---------------------------------------------------------------------------
# Executables and tests
# ---------------------------------------------------------------------------
if(WAYPOINT_BUILD_APPS)
    add_executable(waypoint_demo apps/demo.cpp)
    target_link_libraries(waypoint_demo PRIVATE waypoint::waypoint)

    add_executable(waypoint_bench
        bench/main.cpp
        bench/routing_benchmark.cpp
        bench/spatial_index_benchmark.cpp
        bench/candidate_pricing_benchmark.cpp
        bench/batch_dispatch_benchmark.cpp)
    target_link_libraries(waypoint_bench PRIVATE waypoint::waypoint)
endif()

if(WAYPOINT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
