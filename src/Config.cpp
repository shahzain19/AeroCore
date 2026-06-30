/**
 * @file Config.cpp
 * @brief TOML-subset configuration file parser implementation.
 *
 * Supports:
 *  - Sections:  [section_name]
 *  - Key-value: key = value
 *  - Comments:  lines starting with '#' or ';'
 *  - Inline comments: value # comment (comment stripped)
 *  - Quoted string values: key = "hello world"
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Utilities/Config.h"
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace AeroCore {
namespace Utilities {

// ============================================================
//  Construction
// ============================================================

Config::Config(const std::string& filename) {
    loadFromFile(filename);
}

// ============================================================
//  File parsing
// ============================================================

void Config::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Config: cannot open file: " + filename);
    }

    std::string current_section;
    std::string line;
    int line_number = 0;

    while (std::getline(file, line)) {
        ++line_number;

        // --- Strip leading/trailing whitespace ---
        auto trim = [](std::string& s) {
            s.erase(0, s.find_first_not_of(" \t\r\n"));
            const auto last = s.find_last_not_of(" \t\r\n");
            if (last != std::string::npos) s.erase(last + 1);
        };
        trim(line);

        // --- Skip empty lines and comment-only lines ---
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }

        // --- Section header: [section_name] ---
        if (line.front() == '[') {
            const auto close = line.find(']');
            if (close == std::string::npos) {
                throw std::runtime_error(
                    "Config: malformed section on line " + std::to_string(line_number));
            }
            current_section = line.substr(1, close - 1);
            trim(current_section);
            continue;
        }

        // --- Key = value ---
        const auto eq_pos = line.find('=');
        if (eq_pos == std::string::npos || current_section.empty()) {
            continue;  // Skip lines without '=' or outside a section
        }

        std::string key   = line.substr(0, eq_pos);
        std::string value = line.substr(eq_pos + 1);
        trim(key);
        trim(value);

        // Strip inline comment (# after value)
        // Careful not to strip '#' inside quoted strings
        bool in_quotes = false;
        for (size_t i = 0; i < value.size(); ++i) {
            if (value[i] == '"') in_quotes = !in_quotes;
            if (!in_quotes && value[i] == '#') {
                value = value.substr(0, i);
                break;
            }
        }
        trim(value);

        // Strip surrounding double-quotes from string values
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
            value = value.substr(1, value.size() - 2);
        }

        if (!key.empty()) {
            data_[current_section][key] = value;
        }
    }
}

} // namespace Utilities
} // namespace AeroCore
