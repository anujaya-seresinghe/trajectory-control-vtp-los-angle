#pragma once

// Same trajectory JSON as ground_station/bridge (copied from bridge/src/trajectory.hpp)

#include <cstdint>
#include <string>
#include <vector>

namespace trajectory_mode_fw
{

struct TrajectoryPoint {
	float x;  // North [m]
	float y;  // East [m]
	float vx; // [m/s]
	float vy; // [m/s]
	float at; // lateral acceleration, flight-path y (positive = turning right) [m/s^2]
	float jt; // d(at)/dt [m/s^3]
	float t;  // time from start of trajectory [s]
};

struct Trajectory {
	uint8_t id{0};
	std::vector<TrajectoryPoint> points; // in order; the array position is the waypoint index
};

/**
 * Parse the trajectory JSON published by the web app on <prefix>/<sysid>/cmd/trajectory:
 *
 *   {"id": 3, "points": [[x, y, vx, vy, at, jt, t], ...]}
 */
bool parseTrajectoryJson(const std::string &json, Trajectory &out, std::string &error);

} // namespace trajectory_mode_fw
