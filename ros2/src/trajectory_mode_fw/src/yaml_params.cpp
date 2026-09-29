#include "trajectory_mode_fw/yaml_params.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace trajectory_mode_fw
{

std::string yamlDouble(double value)
{
	char buf[64];
	snprintf(buf, sizeof(buf), "%.10g", value);
	std::string s = buf;

	if (s.find_first_of(".eEn") == std::string::npos) {
		s += ".0";
	}

	return s;
}

std::string updateYamlValues(const std::string &yaml, const std::vector<std::pair<std::string, std::string>> &values)
{
	std::vector<std::string> lines;
	std::istringstream in(yaml);

	for (std::string line; std::getline(in, line);) {
		lines.push_back(line);
	}

	for (const auto &[key, value] : values) {
		bool found = false;

		for (std::string &line : lines) {
			const size_t indent = line.find_first_not_of(" \t");

			if (indent == std::string::npos || line.compare(indent, key.size() + 1, key + ":") != 0) {
				continue;
			}

			// keep a trailing comment ("   # [m] ...") and the spacing before it
			const size_t value_start = indent + key.size() + 1;
			const size_t comment = line.find('#', value_start);

			if (comment != std::string::npos) {
				size_t spaces = comment;

				while (spaces > value_start && (line[spaces - 1] == ' ' || line[spaces - 1] == '\t')) {
					--spaces;
				}

				// keep the comment column if the new value is not longer than the old one
				const size_t old_value_end = spaces;
				const size_t column_width = comment - (value_start + 1);
				const std::string padded = value.size() < column_width ? value + std::string(column_width - value.size(), ' ')
							   : value + std::string(line, old_value_end, comment - old_value_end);
				line = line.substr(0, value_start) + " " + padded + line.substr(comment);

			} else {
				line = line.substr(0, value_start) + " " + value;
			}

			found = true;
			break;
		}

		if (!found) {
			// append after "ros__parameters:" with the indentation of the following line (or 4 spaces)
			for (size_t i = 0; i < lines.size(); ++i) {
				const size_t indent = lines[i].find_first_not_of(" \t");

				if (indent != std::string::npos && lines[i].compare(indent, 16, "ros__parameters:") == 0) {
					std::string child_indent(indent + 2, ' ');

					if (i + 1 < lines.size()) {
						const size_t next = lines[i + 1].find_first_not_of(" \t");

						if (next != std::string::npos && next > indent) {
							child_indent = std::string(next, ' ');
						}
					}

					lines.insert(lines.begin() + static_cast<long>(i) + 1, child_indent + key + ": " + value);
					break;
				}
			}
		}
	}

	std::string out;

	for (const std::string &line : lines) {
		out += line + "\n";
	}

	return out;
}

bool updateYamlFile(const std::string &path, const std::vector<std::pair<std::string, std::string>> &values,
		    std::string &error)
{
	std::ifstream in(path);

	if (!in) {
		error = "cannot read " + path;
		return false;
	}

	std::stringstream buffer;
	buffer << in.rdbuf();
	in.close();

	const std::string updated = updateYamlValues(buffer.str(), values);
	std::ofstream out(path, std::ios::trunc);

	if (!out || !(out << updated)) {
		error = "cannot write " + path;
		return false;
	}

	return true;
}

} // namespace trajectory_mode_fw
