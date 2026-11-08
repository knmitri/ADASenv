cmake_minimum_required(VERSION 3.16)

project(ParkingEnv LANGUAGES CXX)

enable_testing()

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

if(WIN32)
  target_link_libraries(tcp_json_sim_bridge PRIVATE Ws2_32)
endif()

foreach(target reference_parking_stack tcp_json_sim_bridge model_parity_test functional_components_test)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()
