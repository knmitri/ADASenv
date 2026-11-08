#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "sim_bridge/protocol/json_fields.hpp"
#include "sim_bridge/protocol/simulator_protocol.hpp"

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Function>
void require_throws(Function function, const std::string& expected, const std::string& context) {
    try {
        function();
    } catch (const Exception& error) {
        require(std::string(error.what()).find(expected) != std::string::npos, context);
        return;
    }
    throw std::runtime_error(context + ": no exception was thrown");
}

void test_nested_reset_and_state_parsing() {
    const std::string reset = R"({
        "type":"reset",
        "start":[1.0,2.0],
        "start_heading":0.25,
        "goal":[5.0,6.0,1.5],
        "initial_state":{"position":[90,91],"heading":3.0},
        "vehicle":{"length":4.7,"width":1.9,"wheelbase":2.8,
                   "front_overhang":0.9,"rear_overhang":1.0,"max_speed":2.0,
                   "max_reverse_speed":-1.0},
        "simulation":{"dt":0.05,"world_min_x":-2,"world_min_y":-3,
                      "world_max_x":20,"world_max_y":30},
        "goal_slot":{"corners":[[4,5],[6,5],[6,7],[4,7]]},
        "static_obstacles":[{"x":3,"y":4,"heading":0.2,"length":4,"width":2,
                             "metadata":{"note":"nested [ value ]"},
                             "corners":[[1,1],[2,1],[2,2],[1,2]]}]
    })";
    const parking_adas::PlanRequest request = sim_bridge::parse_reset_request(reset);
    require(request.start_x == 1.0 && request.start_y == 2.0,
            "reset start coordinates were not parsed");
    require(request.goal.x == 5.0 && request.goal.y == 6.0 && request.goal.yaw == 1.5,
            "reset goal was not parsed");
    require(request.vehicle.front_overhang == 0.9 && request.vehicle.rear_overhang == 1.0,
            "vehicle dimensions were not parsed");
    require(request.vehicle.max_reverse_speed == 1.0, "reverse speed sign was not normalized");
    require(request.goal_slot_footprint.size() == 4,
            "nested goal slot corners were not parsed");
    require(request.static_obstacles.size() == 1 &&
                request.static_obstacles[0].footprint.size() == 4,
            "nested obstacle corners were not parsed");

    const std::string state_json = R"({
        "type":"state",
        "step":3,
        "state":{"position":[7.5,-1.25],"heading":-0.5,"velocity":[0.8,0.0]},
        "dynamic_obstacles":[{"x":2,"y":3,"heading":0,"length":4,"width":2}]
    })";
    const parking_adas::VehicleState state = sim_bridge::parse_state_request(state_json);
    require(state.x == 7.5 && state.y == -1.25 && state.yaw == -0.5,
            "nested vehicle state was not parsed");
    require(state.velocity == 0.8 && state.dynamic_obstacles.size() == 1,
            "state velocity or dynamic obstacles were not parsed");
}

void test_schema_errors() {
    require_throws<std::invalid_argument>(
        [] { sim_bridge::parse_reset_request(R"({"type":"reset","start":[1,2]})"); },
        "goal",
        "missing required reset field was not reported"
    );
    require_throws<std::invalid_argument>(
        [] {
            sim_bridge::parse_reset_request(
                R"({"type":"reset","start":"1,2","goal":[1,2,0],"start_heading":0})"
            );
        },
        "start must be an array",
        "wrong reset field type was not reported"
    );
    require_throws<std::invalid_argument>(
        [] {
            sim_bridge::parse_state_request(
                R"({"type":"state","state":{"position":[1,2],"heading":0,"velocity":"fast"}})"
            );
        },
        "velocity must be an array",
        "wrong state field type was not reported"
    );
    require_throws<std::invalid_argument>(
        [] { sim_bridge::parse_reset_request(R"({"type":"reset","start":[1,2],)"); },
        "Malformed JSON",
        "malformed JSON was not reported"
    );
    require_throws<std::invalid_argument>(
        [] { sim_bridge::parse_reset_request(R"({"type":"state"})"); },
        "Expected message type 'reset'",
        "unexpected protocol type was not reported"
    );
}

void test_escaped_strings_and_serialized_messages() {
    const sim_bridge::Json nested = sim_bridge::parse_json_object(
        R"json({"outer":{"text":"quote: \" slash: \\","items":[1,{"n":2}]}})json",
        "test message"
    );
    require(nested.at("outer").at("text") == "quote: \" slash: \\",
            "escaped input string was not decoded");
    require(nested.at("outer").at("items")[1].at("n") == 2,
            "nested JSON values were not parsed");

    const std::string message = "quote: \" slash: \\ line:\n tab:\t end";
    const sim_bridge::Json error = sim_bridge::Json::parse(sim_bridge::error_response(message));
    require(error.at("type") == "error" && error.at("error") == message,
            "error response did not preserve escaped text");

    const std::vector<parking_adas::Waypoint> path{
        {1.0, 2.0, -1, 0.5, 0.25, 0.75, true},
    };
    const sim_bridge::Json ready = sim_bridge::Json::parse(sim_bridge::ready_response(path));
    require(ready.at("type") == "ready" && ready.at("planned_path").size() == 1,
            "ready response was not valid JSON");
    require(ready.at("planned_path")[0].at("gear_change") == true,
            "waypoint gear-change field was not preserved");

    parking_adas::CommandDiagnostics diagnostics;
    diagnostics.replan_reason = "replanned: \"tight\\turn\"\n";
    const sim_bridge::Json command = sim_bridge::Json::parse(sim_bridge::command_response(
        {0.25, -0.1}, path, diagnostics));
    require(command.at("type") == "command" &&
                command.at("command_diagnostics").at("replan_reason") == diagnostics.replan_reason,
            "command response did not escape its string fields");
    require(sim_bridge::Json::parse(sim_bridge::hello_response()).at("type") == "hello",
            "hello response was not valid JSON");
    require(sim_bridge::Json::parse(sim_bridge::ack_response()).at("type") == "ack",
            "ack response was not valid JSON");
}

void test_legacy_optional_fields_keep_defaults() {
    const parking_adas::PlanRequest request = sim_bridge::parse_reset_request(
        R"({"type":"reset","start":[0,0],"start_heading":0,"goal_pose":[3,4,0]})"
    );
    require(request.goal.x == 3.0 && request.vehicle.length == 4.67,
            "valid clients omitting optional fields lost their defaults");
}

}  // namespace

int main() {
    try {
        test_nested_reset_and_state_parsing();
        test_schema_errors();
        test_escaped_strings_and_serialized_messages();
        test_legacy_optional_fields_keep_defaults();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
