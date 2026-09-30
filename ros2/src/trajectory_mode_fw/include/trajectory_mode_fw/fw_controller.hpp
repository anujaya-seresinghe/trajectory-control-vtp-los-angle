#pragma once

// Per-cycle lateral guidance of the fixed-wing Trajectory mode, without ROS (so the unit tests fly the same code):
//   acquisition (acquisition.hpp) until the vehicle is within r* of the target point, then the unchanged
//   TrajectoryManager + sliding-mode guidance of the internal mode. If the vehicle loses the virtual target
//   (r > max(3 r*, 2 R), or flying away from it, for 3 s), it acquires the current target point again.

#include "trajectory_mode_fw/acquisition.hpp"

namespace trajectory_mode_fw
{

struct FwControllerParams {
	GuidanceParams guidance;
	int search_window{100};
	float acq_a_max{4.f}; // lateral acceleration limit while acquiring [m/s^2]
};

struct FwControllerOutput {
	enum class State : uint8_t { Holding = 1, Tracking = 2, Acquiring = 3, Finished = 4 };
	State state{State::Holding};
	float a_m{0.f};      // lateral acceleration command (Tracking / Acquiring)
	float v_t{0.f};      // target speed for the longitudinal law
	float r{0.f};        // distance to the virtual target (Tracking) or to the point being acquired [m]
	uint16_t index{0};
	float a_long{0.f};   // internal-mode longitudinal command (Tracking), for display
	const char *reason{nullptr}; // Holding / Finished
};

class FwTrajectoryController
{
public:
	void load(const Trajectory &trajectory);
	void clear();
	bool active() const { return _manager.active(); }
	size_t size() const { return _manager.size(); }

	FwControllerOutput update(Vec2 pos, Vec2 vel, float dt, const FwControllerParams &params);

private:
	TrajectoryManager _manager;
	PathAcquisition _acquisition;
	bool _captured{false};
	bool _finished{false};
	float _lost_time{0.f};
};

} // namespace trajectory_mode_fw
