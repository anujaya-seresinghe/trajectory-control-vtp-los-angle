#include "trajectory.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace
{

// Minimal cursor over the fixed trajectory JSON shape; not a general JSON parser.
struct Cursor {
	const char *p;
	const char *end;

	void skipWs()
	{
		while (p < end && std::isspace(static_cast<unsigned char>(*p))) {
			++p;
		}
	}

	bool consume(char c)
	{
		skipWs();

		if (p < end && *p == c) {
			++p;
			return true;
		}

		return false;
	}

	bool peek(char c)
	{
		skipWs();
		return p < end && *p == c;
	}

	bool number(double &value)
	{
		skipWs();
		char *num_end = nullptr;
		value = std::strtod(p, &num_end);

		if (num_end == p || num_end > end || !std::isfinite(value)) {
			return false;
		}

		p = num_end;
		return true;
	}

	bool string(std::string &value)
	{
		if (!consume('"')) {
			return false;
		}

		const char *start = p;

		while (p < end && *p != '"') {
			++p;
		}

		if (p >= end) {
			return false;
		}

		value.assign(start, p);
		++p;
		return true;
	}
};

bool parsePoints(Cursor &c, std::vector<TrajectoryPoint> &points, std::string &error)
{
	if (!c.consume('[')) {
		error = "\"points\" must be an array";
		return false;
	}

	if (c.consume(']')) {
		return true;
	}

	do {
		if (!c.consume('[')) {
			error = "each point must be an array [x, y, vx, vy, at, jt, t]";
			return false;
		}

		double v[7];

		for (int i = 0; i < 7; ++i) {
			if ((i > 0 && !c.consume(',')) || !c.number(v[i])) {
				error = "point " + std::to_string(points.size()) + " needs 7 finite numbers";
				return false;
			}
		}

		if (!c.consume(']')) {
			error = "point " + std::to_string(points.size()) + " has more than 7 values";
			return false;
		}

		points.push_back({float(v[0]), float(v[1]), float(v[2]), float(v[3]), float(v[4]), float(v[5]), float(v[6])});
	} while (c.consume(','));

	if (!c.consume(']')) {
		error = "unterminated \"points\" array";
		return false;
	}

	return true;
}

} // namespace

bool parseTrajectoryJson(const std::string &json, Trajectory &out, std::string &error)
{
	Cursor c{json.data(), json.data() + json.size()};
	bool have_id = false;
	bool have_points = false;
	out = Trajectory{};

	if (!c.consume('{')) {
		error = "expected a JSON object";
		return false;
	}

	if (!c.peek('}')) {
		do {
			std::string key;

			if (!c.string(key) || !c.consume(':')) {
				error = "malformed key";
				return false;
			}

			if (key == "id") {
				double id;

				if (!c.number(id) || id < 0 || id > 255 || id != std::floor(id)) {
					error = "\"id\" must be an integer 0-255";
					return false;
				}

				out.id = static_cast<uint8_t>(id);
				have_id = true;

			} else if (key == "points") {
				if (!parsePoints(c, out.points, error)) {
					return false;
				}

				have_points = true;

			} else {
				error = "unknown key \"" + key + "\"";
				return false;
			}
		} while (c.consume(','));
	}

	if (!c.consume('}')) {
		error = "expected '}'";
		return false;
	}

	if (!have_id || !have_points) {
		error = "\"id\" and \"points\" are required";
		return false;
	}

	if (out.points.empty()) {
		error = "trajectory has no points";
		return false;
	}

	if (out.points.size() > std::numeric_limits<uint16_t>::max()) {
		error = "too many points for a uint16 index";
		return false;
	}

	return true;
}
