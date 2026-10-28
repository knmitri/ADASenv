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
std::vector<std::vector<double>> find_number_arrays(const std::string& text, const std::string& key);

std::string find_object(const std::string& text, const std::string& key);
std::vector<std::string> find_object_array(const std::string& text, const std::string& key);

}  // namespace sim_bridge
