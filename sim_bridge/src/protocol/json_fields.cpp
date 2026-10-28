#include "sim_bridge/protocol/json_fields.hpp"

#include <cstdlib>

namespace {

std::size_t matching_delimiter(const std::string& text, std::size_t open, char left, char right) {
    int depth = 0;
    for (std::size_t index = open; index < text.size(); ++index) {
        if (text[index] == left) { ++depth; }
        if (text[index] == right && --depth == 0) { return index; }
    }
    return std::string::npos;
}

}  // namespace

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

std::vector<std::vector<double>> find_number_arrays(const std::string& text, const std::string& key) {
    const std::size_t key_pos = text.find("\"" + key + "\"");
    if (key_pos == std::string::npos) return {};
    const std::size_t open = text.find('[', key_pos);
    const std::size_t close = open == std::string::npos ? std::string::npos : matching_delimiter(text, open, '[', ']');
    if (close == std::string::npos) return {};
    std::vector<std::vector<double>> result;
    for (std::size_t index = open + 1; index < close;) {
        const std::size_t row_open = text.find('[', index);
        if (row_open == std::string::npos || row_open >= close) break;
        const std::size_t row_close = matching_delimiter(text, row_open, '[', ']');
        if (row_close == std::string::npos || row_close > close) break;
        std::vector<double> row;
        const char* cursor = text.c_str() + row_open + 1;
        const char* stop = text.c_str() + row_close;
        while (cursor < stop) {
            char* end = nullptr;
            const double value = std::strtod(cursor, &end);
            if (end != cursor) { row.push_back(value); cursor = end; } else { ++cursor; }
        }
        result.push_back(row);
        index = row_close + 1;
    }
    return result;
}

std::string find_object(const std::string& text, const std::string& key) {
    const std::size_t key_pos = text.find("\"" + key + "\"");
    if (key_pos == std::string::npos) { return {}; }
    const std::size_t open = text.find('{', key_pos);
    if (open == std::string::npos) { return {}; }
    const std::size_t close = matching_delimiter(text, open, '{', '}');
    return close == std::string::npos ? std::string{} : text.substr(open, close - open + 1);
}

std::vector<std::string> find_object_array(const std::string& text, const std::string& key) {
    std::vector<std::string> objects;
    const std::size_t key_pos = text.find("\"" + key + "\"");
    if (key_pos == std::string::npos) { return objects; }
    const std::size_t open = text.find('[', key_pos);
    if (open == std::string::npos) { return objects; }
    const std::size_t close = matching_delimiter(text, open, '[', ']');
    if (close == std::string::npos) { return objects; }
    for (std::size_t index = open + 1; index < close;) {
        const std::size_t object_open = text.find('{', index);
        if (object_open == std::string::npos || object_open >= close) { break; }
        const std::size_t object_close = matching_delimiter(text, object_open, '{', '}');
        if (object_close == std::string::npos) { break; }
        objects.push_back(text.substr(object_open, object_close - object_open + 1));
        index = object_close + 1;
    }
    return objects;
}

}  // namespace sim_bridge
