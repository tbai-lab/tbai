#pragma once

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include <tbai_core/Throws.hpp>

namespace tbai {

template <typename T>
T getEnvAs(const std::string &var) {
    const char *value = std::getenv(var.c_str());
    TBAI_THROW_UNLESS(value != nullptr, "Environment variable {} is not set.", var);
    T returnValue;
    std::stringstream(value) >> returnValue;
    return returnValue;
}

template <typename T>
T getEnvAs(const std::string &var, T defaultValue) {
    const char *value = std::getenv(var.c_str());
    if (value == nullptr) {
        return defaultValue;
    }
    T returnValue;
    std::stringstream(value) >> returnValue;
    return returnValue;
}

template <>
bool getEnvAs(const std::string &var);

template <>
bool getEnvAs(const std::string &var, bool defaultValue);

template <typename T>
T getEnvAsChecked(const std::string &var, const std::vector<T> &allowedValues) {
    T value = getEnvAs<T>(var);
    if (std::find(allowedValues.begin(), allowedValues.end(), value) == allowedValues.end()) {
        TBAI_THROW("Environment variable {} has invalid value: '{}'.\nExpected one of {}", var, value, allowedValues);
    }
    return value;
}

template <typename T>
T getEnvAsChecked(const std::string &var, const std::vector<T> &allowedValues, T defaultValue) {
    T value = getEnvAs<T>(var, defaultValue);
    if (std::find(allowedValues.begin(), allowedValues.end(), value) == allowedValues.end()) {
        TBAI_THROW("Environment variable {} has invalid value: '{}'.\nExpected one of {}", var, value, allowedValues);
    }
    return value;
}

void setEnv(const std::string &var, const std::string &value);
void unsetEnv(const std::string &var);

}  // namespace tbai
