#pragma once

// Persisting runtime parameter changes into the ROS 2 params YAML (config/params.yaml).

#include <string>
#include <utility>
#include <vector>

namespace trajectory_mode_fw
{

// YAML scalar for a double that ROS 2 loads back as a double (always has a decimal point: 30 -> "30.0")
std::string yamlDouble(double value);

// Replace the values of `key: value` lines in `yaml` (comments, indentation and other keys are kept).
// Keys that are missing are appended after the "ros__parameters:" line.
std::string updateYamlValues(const std::string &yaml, const std::vector<std::pair<std::string, std::string>> &values);

// Read, update and rewrite `path` in place (truncate + write, so bind mounts and file ownership survive)
bool updateYamlFile(const std::string &path, const std::vector<std::pair<std::string, std::string>> &values,
		    std::string &error);

} // namespace trajectory_mode_fw
