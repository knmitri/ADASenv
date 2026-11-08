#include <charconv>
#include <exception>
#include <iostream>
#include <string>
#include <system_error>

#include "parking_adas/stack/parking_adas_stack.hpp"
#include "sim_bridge/cli.hpp"
#include "sim_bridge/protocol/simulator_protocol.hpp"
#include "sim_bridge/transport/tcp_client.hpp"

namespace sim_bridge {

int run_tcp_client(int argc, char** argv) {
    const std::string host = arg_value(argc, argv, "--host", "127.0.0.1");
    const std::string port = arg_value(argc, argv, "--port", "8765");
    const std::string frame_size_arg = arg_value(
        argc,
        argv,
        "--max-frame-size",
        std::to_string(TcpClient::kDefaultMaxFrameSize)
    );
    std::size_t max_frame_size = 0;
    const auto frame_size_result = std::from_chars(
        frame_size_arg.data(),
        frame_size_arg.data() + frame_size_arg.size(),
        max_frame_size
    );
    if (frame_size_result.ec != std::errc{} ||
        frame_size_result.ptr != frame_size_arg.data() + frame_size_arg.size() ||
        max_frame_size == 0) {
        throw std::invalid_argument("--max-frame-size must be a positive integer");
    }
    const std::string planner_id = arg_value(argc, argv, "--path-planner", "hybrid_astar");
    const std::string tracker_id = arg_value(argc, argv, "--trajectory-tracker", "pure_pursuit");

    parking_adas::ParkingAdasStack stack(
        parking_adas::make_path_planner(planner_id),
        parking_adas::make_trajectory_tracker(tracker_id));
    TcpClient client(max_frame_size);
    client.connect_to(host, port);

    std::string line;
    while (client.read_line(line)) {
        try {
            const std::string type = message_type(line);
            if (type == "hello") {
                client.write_line(hello_response());
            } else if (type == "reset") {
                client.write_line(ready_response(
                    stack.reset(parse_reset_request(line)), stack.planning_diagnostics()));
            } else if (type == "state") {
                client.write_line(command_response(
                    stack.command(parse_state_request(line)), stack.trajectory(),
                    stack.command_diagnostics()));
            } else if (type == "result") {
                client.write_line(ack_response());
            } else if (type == "shutdown") {
                client.write_line(ack_response());
                break;
            } else {
                client.write_line(error_response("unknown remote request type"));
            }
        } catch (const std::exception& exc) {
            client.write_line(error_response(exc.what()));
        }
    }
    return 0;
}

}  // namespace sim_bridge

int main(int argc, char** argv) {
    try {
        if (!sim_bridge::has_arg(argc, argv, "--tcp-client")) {
            std::cerr << "tcp_json_sim_bridge only supports TCP mode. "
                      << "Use --tcp-client --host <host> --port <port>.\n";
            return 2;
        }
        return sim_bridge::run_tcp_client(argc, argv);
    } catch (const std::exception& exc) {
        std::cerr << "ADAS stack failed: " << exc.what() << '\n';
        return 1;
    }
}
