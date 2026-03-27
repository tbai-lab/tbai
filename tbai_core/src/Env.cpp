#include <tbai_core/Env.hpp>
#include <tbai_core/Throws.hpp>

#include <algorithm>

namespace tbai {

static bool parseBool(const std::string &raw) {
    std::string value = raw;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return (value == "true" || value == "1" || value == "yes" || value == "on");
}

template <>
bool getEnvAs(const std::string &var) {
    return parseBool(getEnvAs<std::string>(var));
}

template <>
bool getEnvAs(const std::string &var, bool defaultValue) {
    const char *value = std::getenv(var.c_str());
    if (value == nullptr) {
        return defaultValue;
    }
    return parseBool(value);
}

void setEnv(const std::string &var, const std::string &value) {
    if (setenv(var.c_str(), value.c_str(), 1) != 0) {
        TBAI_THROW("Failed to set environment variable " + var);
    }
}

void unsetEnv(const std::string &var) {
    if (unsetenv(var.c_str()) != 0) {
        TBAI_THROW("Failed to unset environment variable " + var);
    }
}

}  // namespace tbai
