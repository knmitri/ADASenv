#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace sim_bridge {

using Json = nlohmann::json;

Json parse_json_object(const std::string& text, const std::string& context);
Json optional_object_field(const Json& object, const std::string& key, const std::string& path);
Json optional_array_field(const Json& object, const std::string& key, const std::string& path);

std::string required_string_field(
    const Json& object,
    const std::string& key,
    const std::string& path
);
double required_number_field(
    const Json& object,
    const std::string& key,
    const std::string& path
);
double optional_number_field(
    const Json& object,
    const std::string& key,
    const std::string& path,
    double fallback
);
std::vector<double> required_number_array_field(
    const Json& object,
    const std::string& key,
    const std::string& path,
    std::size_t minimum_size
);
std::vector<std::vector<double>> optional_number_arrays_field(
    const Json& object,
    const std::string& key,
    const std::string& path,
    std::size_t row_minimum_size
);

}  // namespace sim_bridge
