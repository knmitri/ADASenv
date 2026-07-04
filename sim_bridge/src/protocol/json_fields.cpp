#include "sim_bridge/protocol/json_fields.hpp"

#include <cstdlib>

namespace sim_bridge {

std::string find_string(
    const std::string& text,
    const std::string& key,
    const std::string& fallback
) {
    const std::string pattern = "\"" + key + "\"";
    const std::size_t key_pos = text.find(pattern);
    if (key_pos == std::string::npos) {
        return fallback;
    }
    const std::size_t colon = text.find(':', key_pos + pattern.size());
    if (colon == std::string::npos) {
        return fallback;
    }
    const std::size_t first_quote = text.find('"', colon + 1);
    if (first_quote == std::string::npos) {
        return fallback;
    }
    const std::size_t second_quote = text.find('"', first_quote + 1);
    if (second_quote == std::string::npos) {
        return fallback;
    }
    return text.substr(first_quote + 1, second_quote - first_quote - 1);
}

double find_number(const std::string& text, const std::string& key, double fallback) {
    const std::string pattern = "\"" + key + "\"";
    const std::size_t key_pos = text.find(pattern);
    if (key_pos == std::string::npos) {
        return fallback;
    }
    const std::size_t colon = text.find(':', key_pos + pattern.size());
    if (colon == std::string::npos) {
        return fallback;
    }
    const char* begin = text.c_str() + colon + 1;
    char* end = nullptr;
    const double value = std::strtod(begin, &end);
    if (end == begin) {
        return fallback;
    }
    return value;
}

std::vector<double> find_number_array(
    const std::string& text,
    const std::string& key,
    const std::vector<double>& fallback
) {
    const std::string pattern = "\"" + key + "\"";
    const std::size_t key_pos = text.find(pattern);
    if (key_pos == std::string::npos) {
        return fallback;
    }
    const std::size_t open = text.find('[', key_pos + pattern.size());
    if (open == std::string::npos) {
        return fallback;
    }
    const std::size_t close = text.find(']', open + 1);
    if (close == std::string::npos) {
        return fallback;
    }

    std::vector<double> values;
    const char* cursor = text.c_str() + open + 1;
    const char* stop = text.c_str() + close;
    while (cursor < stop) {
        char* end = nullptr;
        const double value = std::strtod(cursor, &end);
        if (end != cursor) {
            values.push_back(value);
            cursor = end;
        } else {
            ++cursor;
        }
    }
    return values.empty() ? fallback : values;
}

}  // namespace sim_bridge
