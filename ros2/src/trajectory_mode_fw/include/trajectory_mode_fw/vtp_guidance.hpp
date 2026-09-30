#pragma once

// ROS-free port of the PX4-internal Trajectory flight mode:
//   TrajectoryManager -> PX4/src/modules/flight_mode_manager/tasks/Traj/trajectory_manager/TrajectoryManager.cpp
//   VtpGuidance       -> PX4/src/modules/flight_mode_manager/tasks/Traj/FlightTaskTraj.cpp
// The equations and the virtual target selection are kept identical to the PX4 code (float math).

#include <algorithm>
#include <cstdint>
#include <vector>

#include "trajectory_mode_fw/trajectory_json.hpp"

namespace trajectory_mode_fw
{

struct Vec2 {
	float x{0.f}; // North
	float y{0.f}; // East
};

// Same fields as the continuous_trajectory_output uORB message
struct ManagerOutput {
	float lambda{0.f};   // LOS angle to the virtual target [rad]
	float lambda_d{0.f}; // (published by PX4 but unused by the guidance)
	float gamma_t{0.f};  // virtual target flight-path angle [rad]
	float gamma_m{0.f};  // vehicle flight-path angle [rad]
	float v_t{0.f};      // virtual target speed [m/s]
	float v_m{0.f};      // vehicle ground speed [m/s]
	float r{0.f};        // distance to the virtual target [m] (INFINITY if none was found)
	float a_t{0.f};      // lateral acceleration of the virtual target [m/s^2]
	float j_t{0.f};      // d(a_t)/dt [m/s^3]
	uint16_t index{0};   // current virtual target index
};

class TrajectoryManager
{
public:
	// Equivalent of TRAJECTORY_SETPOINT_INITIATE + all TRAJECTORY_SETPOINT_UPLOAD messages
	void load(const Trajectory &trajectory);
	void clear();
	bool active() const { return !_points.empty(); }
	uint8_t id() const { return _id; }
	size_t size() const { return _points.size(); }
	uint16_t currentIndex() const { return _current_wp_index; }
	const TrajectoryPoint &point(size_t i) const { return _points[std::min(i, _points.size() - 1)]; } // requires active()

	// TrajectoryManager::update() for one vehicle position/velocity sample
	ManagerOutput update(Vec2 pos_vehicle, Vec2 vel_vehicle, float r_star, int search_window);

private:
	std::vector<TrajectoryPoint> _points; // index == position in the vector
	uint8_t _id{0};
	uint16_t _current_wp_index{0};

	// Members that persist between updates, like in PX4
	float _a_t{0.f};
	float _j_t{0.f};
	Vec2 _v_t{};
	Vec2 _pos_vtp{};
	Vec2 _pos_d{};
};

struct GuidanceParams {
	float alpha{1.4f};      // TRAJ_SMC_ALPHA
	float beta{0.12f};      // TRAJ_SMC_BETA
	float epsilon{0.05f};   // TRAJ_SMC_EPS
	float r_star{15.f};     // TRAJ_R_STAR
	float k_long{0.5f};     // TRAJ_K_LONG
	float a_m_max{2.f};     // TRAJ_A_M_MAX
	float a_long_max{0.3f}; // TRAJ_A_LONG_MAX
};

struct GuidanceOutput {
	float a_m{0.f};    // lateral acceleration, flight-path y (positive = right) [m/s^2]
	float a_long{0.f}; // longitudinal acceleration, along the velocity [m/s^2]
	float s{0.f};      // sliding surface (for logging)
	bool finite{false};
};

// FlightTaskTraj::update() guidance law for one continuous_trajectory_output sample
GuidanceOutput computeGuidance(const ManagerOutput &in, const GuidanceParams &p);

} // namespace trajectory_mode_fw
