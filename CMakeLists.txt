cmake_minimum_required(VERSION 3.16)

project(ADASEnvironment LANGUAGES CXX)

enable_testing()

option(
  ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL
  "Use the analytical Reeds-Shepp connector by default (numerical remains selectable)"
  OFF
)
option(
  ADAS_ENABLE_OMPL_REFERENCE_TESTS
  "Build analytical Reeds-Shepp distance comparisons against installed OMPL"
  OFF
)

add_library(parking_motion_api INTERFACE)
target_include_directories(parking_motion_api INTERFACE motion_api/include)
target_compile_features(parking_motion_api INTERFACE cxx_std_17)

add_library(
  reference_parking_stack STATIC
  reference_adas_stacks/core/src/common/math.cpp
  reference_adas_stacks/core/src/common/footprint_geometry.cpp
  reference_adas_stacks/core/src/control/trajectory_tracker.cpp
  reference_adas_stacks/core/src/control/pure_pursuit_tracker.cpp
  reference_adas_stacks/core/src/planning/parking_adas_stack.cpp
  reference_adas_stacks/core/src/planning/path_planner.cpp
  reference_adas_stacks/core/src/planning/collision_checker.cpp
  reference_adas_stacks/core/src/planning/parking_goal_evaluator.cpp
  reference_adas_stacks/core/src/planning/reeds_shepp_connector.cpp
  reference_adas_stacks/core/src/planning/reeds_shepp_analytic.cpp
  reference_adas_stacks/core/src/planning/trajectory_generator.cpp
  reference_adas_stacks/core/src/planning/trajectory_optimizer.cpp
  reference_adas_stacks/src/reeds_shepp_local/reeds_shepp_local_planner.cpp
  reference_adas_stacks/src/hybrid_astar/hybrid_a_star.cpp
)

target_include_directories(
  reference_parking_stack
  PUBLIC reference_adas_stacks/core/include reference_adas_stacks/include
)
target_link_libraries(reference_parking_stack PUBLIC parking_motion_api)
target_compile_features(reference_parking_stack PUBLIC cxx_std_17)
if(ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL)
  target_compile_definitions(reference_parking_stack PRIVATE ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL=1)
endif()

add_executable(
  tcp_json_sim_bridge
  sim_bridge/src/app/main.cpp
  sim_bridge/src/cli.cpp
  sim_bridge/src/protocol/json_fields.cpp
  sim_bridge/src/protocol/simulator_protocol.cpp
  sim_bridge/src/transport/tcp_client.cpp
)

target_include_directories(
  tcp_json_sim_bridge
  PRIVATE sim_bridge/include
)
target_link_libraries(tcp_json_sim_bridge PRIVATE reference_parking_stack)
target_compile_features(tcp_json_sim_bridge PRIVATE cxx_std_17)

add_executable(model_parity_test tests/cpp/model_parity_test.cpp)
target_link_libraries(model_parity_test PRIVATE reference_parking_stack)
target_compile_features(model_parity_test PRIVATE cxx_std_17)
add_test(NAME model_parity_test COMMAND model_parity_test)

add_executable(
  functional_components_test
  tests/cpp/functional_components_test.cpp
  sim_bridge/src/protocol/json_fields.cpp
  sim_bridge/src/protocol/simulator_protocol.cpp
)
target_include_directories(functional_components_test PRIVATE sim_bridge/include)
target_link_libraries(functional_components_test PRIVATE reference_parking_stack)
target_compile_features(functional_components_test PRIVATE cxx_std_17)
add_test(NAME functional_components_test COMMAND functional_components_test)

add_executable(
  reeds_shepp_verification_test
  tests/cpp/reeds_shepp_verification_test.cpp
)
target_link_libraries(reeds_shepp_verification_test PRIVATE reference_parking_stack)
target_compile_features(reeds_shepp_verification_test PRIVATE cxx_std_17)
add_test(NAME reeds_shepp_verification_test COMMAND reeds_shepp_verification_test)

if(ADAS_ENABLE_OMPL_REFERENCE_TESTS)
  find_package(ompl CONFIG REQUIRED)
  add_executable(reeds_shepp_ompl_reference_test
    tests/cpp/reeds_shepp_ompl_reference_test.cpp
  )
  if(NOT TARGET ompl::ompl)
    message(FATAL_ERROR "The OMPL package must provide the ompl::ompl CMake target")
  endif()
  target_link_libraries(reeds_shepp_ompl_reference_test
    PRIVATE reference_parking_stack ompl::ompl
  )
  target_compile_features(reeds_shepp_ompl_reference_test PRIVATE cxx_std_17)
  if(NOT DEFINED ompl_VERSION)
    set(ompl_VERSION "unknown")
  endif()
  target_compile_definitions(reeds_shepp_ompl_reference_test
    PRIVATE
      OMPL_REFERENCE_VERSION="${ompl_VERSION}"
      OMPL_TEST_CONFIGURATION="$<CONFIG>"
  )
  add_test(NAME reeds_shepp_ompl_reference_test COMMAND reeds_shepp_ompl_reference_test)
endif()

if(WIN32)
  target_link_libraries(tcp_json_sim_bridge PRIVATE Ws2_32)
endif()

set(build_targets reference_parking_stack tcp_json_sim_bridge model_parity_test functional_components_test reeds_shepp_verification_test)
if(ADAS_ENABLE_OMPL_REFERENCE_TESTS)
  list(APPEND build_targets reeds_shepp_ompl_reference_test)
endif()

foreach(target ${build_targets})
  if(ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL AND target STREQUAL "reeds_shepp_verification_test")
    target_compile_definitions(${target} PRIVATE ADAS_REEDS_SHEPP_DEFAULT_ANALYTICAL=1)
  endif()
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()
