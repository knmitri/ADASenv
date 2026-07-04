#pragma once

#include <string>
#include <vector>

namespace sim_bridge {

std::string find_string(
    const std::string& text,
    const std::string& key,
    const std::string& fallback = ""
);

double find_number(const std::string& text, const std::string& key, double fallback = 0.0);

std::vector<double> find_number_array(
    const std::string& text,
    const std::string& key,
    const std::vector<double>& fallback = {}
);

}  // namespace sim_bridge
