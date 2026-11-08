#include "sim_bridge/protocol/json_fields.hpp"

#include <cmath>
#include <stdexcept>

namespace sim_bridge {

namespace {

const Json& require_object(const Json& value, const std::string& path) {
    if (!value.is_object()) {
        throw std::invalid_argument(path + " must be a JSON object");
    }
    return value;
}

std::string field_path(const std::string& path, const std::string& key) {
    return path.empty() ? key : path + "." + key;
}

const Json& required_field(const Json& object, const std::string& key, const std::string& path) {
    require_object(object, path.empty() ? "request" : path);
    const auto found = object.find(key);
    if (found == object.end()) {
        throw std::invalid_argument("Missing required field '" + field_path(path, key) + "'");
    }
    return *found;
}

double number_value(const Json& value, const std::string& path) {
    if (!value.is_number()) {
        throw std::invalid_argument(path + " must be a number");
    }
    const double number = value.get<double>();
    if (!std::isfinite(number)) {
        throw std::invalid_argument(path + " must be finite");
    }
    return number;
}

std::vector<double> number_array(
    const Json& value,
    const std::string& path,
    std::size_t minimum_size
) {
    if (!value.is_array()) {
        throw std::invalid_argument(path + " must be an array");
    }
    if (value.size() < minimum_size) {
        throw std::invalid_argument(
            path + " must contain at least " + std::to_string(minimum_size) + " values"
        );
    }
    std::vector<double> numbers;
    numbers.reserve(value.size());
    for (std::size_t index = 0; index < value.size(); ++index) {
        numbers.push_back(number_value(value[index], path + "[" + std::to_string(index) + "]"));
    }
    return numbers;
}

}  // namespace

Json parse_json_object(const std::string& text, const std::string& context) {
    Json value;
    try {
        value = Json::parse(text);
    } catch (const Json::parse_error& error) {
        throw std::invalid_argument("Malformed JSON in " + context + ": " + error.what());
    }
    require_object(value, context);
    return value;
}

Json optional_object_field(const Json& object, const std::string& key, const std::string& path) {
    const Json& parent = require_object(object, path.empty() ? "request" : path);
    const auto found = parent.find(key);
    if (found == parent.end()) {
        return Json::object();
    }
    if (!found->is_object()) {
        throw std::invalid_argument(field_path(path, key) + " must be a JSON object");
    }
    return *found;
}

Json optional_array_field(const Json& object, const std::string& key, const std::string& path) {
    const Json& parent = require_object(object, path.empty() ? "request" : path);
    const auto found = parent.find(key);
    if (found == parent.end()) {
        return Json::array();
    }
    if (!found->is_array()) {
        throw std::invalid_argument(field_path(path, key) + " must be an array");
    }
    return *found;
}

std::string required_string_field(
    const Json& object,
    const std::string& key,
    const std::string& path
) {
    const Json& value = required_field(object, key, path);
    if (!value.is_string()) {
        throw std::invalid_argument(field_path(path, key) + " must be a string");
    }
    return value.get<std::string>();
}

double required_number_field(
    const Json& object,
    const std::string& key,
    const std::string& path
) {
    return number_value(required_field(object, key, path), field_path(path, key));
}

double optional_number_field(
    const Json& object,
    const std::string& key,
    const std::string& path,
    double fallback
) {
    const Json& parent = require_object(object, path.empty() ? "request" : path);
    const auto found = parent.find(key);
    return found == parent.end()
        ? fallback
        : number_value(*found, field_path(path, key));
}

std::vector<double> required_number_array_field(
    const Json& object,
    const std::string& key,
    const std::string& path,
    std::size_t minimum_size
) {
    return number_array(
        required_field(object, key, path),
        field_path(path, key),
        minimum_size
    );
}

std::vector<std::vector<double>> optional_number_arrays_field(
    const Json& object,
    const std::string& key,
    const std::string& path,
    std::size_t row_minimum_size
) {
    const Json& parent = require_object(object, path.empty() ? "request" : path);
    const auto found = parent.find(key);
    if (found == parent.end()) {
        return {};
    }
    const std::string array_path = field_path(path, key);
    if (!found->is_array()) {
        throw std::invalid_argument(array_path + " must be an array");
    }

    std::vector<std::vector<double>> rows;
    rows.reserve(found->size());
    for (std::size_t index = 0; index < found->size(); ++index) {
        rows.push_back(number_array(
            (*found)[index],
            array_path + "[" + std::to_string(index) + "]",
            row_minimum_size
        ));
    }
    return rows;
}

}  // namespace sim_bridge
