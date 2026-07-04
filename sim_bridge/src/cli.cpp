#include "sim_bridge/cli.hpp"

namespace sim_bridge {

std::string arg_value(int argc, char** argv, const std::string& name, const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (name == argv[i]) {
            return argv[i + 1];
        }
    }
    return fallback;
}

bool has_arg(int argc, char** argv, const std::string& name) {
    for (int i = 1; i < argc; ++i) {
        if (name == argv[i]) {
            return true;
        }
    }
    return false;
}

}  // namespace sim_bridge
