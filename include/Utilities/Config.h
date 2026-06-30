#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <stdexcept>
#include <type_traits>

namespace AeroCore {
namespace Utilities {

class Config {
public:
    explicit Config(const std::string& filename);
    
    template <typename T>
    T get(const std::string& section, const std::string& key) const;
    
    template <typename T>
    void set(const std::string& section, const std::string& key, const T& value);

private:
    std::map<std::string, std::map<std::string, std::string>> data_;
    void loadFromFile(const std::string& filename);
};

template <typename T>
T Config::get(const std::string& section, const std::string& key) const {
    auto sec_it = data_.find(section);
    if (sec_it == data_.end()) {
        throw std::runtime_error("Section not found: " + section);
    }
    auto key_it = sec_it->second.find(key);
    if (key_it == sec_it->second.end()) {
        throw std::runtime_error("Key not found: " + key + " in section " + section);
    }
    
    if constexpr (std::is_same_v<T, std::string>) {
        return key_it->second;
    } else {
        T value;
        std::istringstream iss(key_it->second);
        iss >> value;
        if (iss.fail()) {
            throw std::runtime_error("Failed to parse value for " + key + " in section " + section);
        }
        return value;
    }
}

template <typename T>
void Config::set(const std::string& section, const std::string& key, const T& value) {
    std::ostringstream oss;
    oss << value;
    data_[section][key] = oss.str();
}

} // namespace Utilities
} // namespace AeroCore
