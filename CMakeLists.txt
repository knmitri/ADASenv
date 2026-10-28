cmake_minimum_required(VERSION 3.16)

project(ParkingEnv LANGUAGES CXX)

enable_testing()

add_library(
  reference_parking_stack STATIC
  adas_stack/src/common/math.cpp
  adas_stack/src/common/footprint_geometry.cpp
  adas_stack/src/control/trajectory_tracker.cpp
  adas_stack/src/planning/parking_adas_stack.cpp
  adas_stack/src/planning/collision_checker.cpp
  adas_stack/src/planning/parking_goal_evaluator.cpp
  adas_stack/src/planning/reeds_shepp_connector.cpp
  adas_stack/src/planning/hybrid_a_star.cpp
)

target_include_directories(
  reference_parking_stack
  PUBLIC adas_stack/include
)
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
