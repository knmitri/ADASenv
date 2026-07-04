#pragma once

#include <string>

namespace sim_bridge {

std::string arg_value(int argc, char** argv, const std::string& name, const std::string& fallback);
bool has_arg(int argc, char** argv, const std::string& name);

}  // namespace sim_bridge
