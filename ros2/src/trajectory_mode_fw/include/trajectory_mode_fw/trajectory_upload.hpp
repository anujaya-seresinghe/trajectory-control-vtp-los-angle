#pragma once

// Assembles a trajectory from TRAJECTORY_SETPOINT_INITIATE + TRAJECTORY_SETPOINT_UPLOAD, like PX4's
// TrajectoryManager: INITIATE starts a new trajectory with no_of_waypoints points, and it is complete once that
// many (distinct) indices have arrived. ROS- and MAVLink-free so it can be unit tested.

#include <cstdint>
#include <optional>
#include <vector>

#include "trajectory_mode_fw/trajectory_json.hpp"

namespace trajectory_mode_fw
{

class TrajectoryUpload
{
public:
	void initiate(uint8_t id, uint16_t no_of_waypoints)
	{
		_id = id;
		_points.assign(no_of_waypoints, TrajectoryPoint{});
		_received.assign(no_of_waypoints, false);
		_count = 0;
	}

	// Returns the complete trajectory when this point was the last one missing
	std::optional<Trajectory> add(uint16_t index, const TrajectoryPoint &point)
	{
		if (index >= _points.size() || _received[index]) {
			return std::nullopt; // no INITIATE yet, out of range, or a duplicate
		}

		_points[index] = point;
		_received[index] = true;

		if (++_count < _points.size()) {
			return std::nullopt;
		}

		Trajectory trajectory;
		trajectory.id = _id;
		trajectory.points = std::move(_points);
		_points.clear(); // further UPLOADs are ignored until the next INITIATE
		_received.clear();
		_count = 0;
		return trajectory;
	}

	size_t expected() const { return _points.size(); }
	size_t received() const { return _count; }

private:
	uint8_t _id{0};
	std::vector<TrajectoryPoint> _points;
	std::vector<bool> _received;
	size_t _count{0};
};

} // namespace trajectory_mode_fw
