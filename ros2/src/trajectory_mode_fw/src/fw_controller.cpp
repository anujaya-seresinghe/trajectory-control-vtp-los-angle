#include "trajectory_mode_fw/fw_controller.hpp"

#include <cmath>

namespace trajectory_mode_fw
{

void FwTrajectoryController::load(const Trajectory &trajectory)
{
	_manager.load(trajectory);
	_acquisition.reset();
	_captured = false;
	_finished = false;
	_lost_time = 0.f;
}

void FwTrajectoryController::clear()
{
	_manager.clear();
	_acquisition.reset();
	_captured = false;
	_finished = false;
	_lost_time = 0.f;
}

FwControllerOutput FwTrajectoryController::update(Vec2 pos, Vec2 vel, float dt, const FwControllerParams &params)
{
	FwControllerOutput out;

	if (!_manager.active()) {
		out.reason = "waiting for a trajectory";
		return out;
	}

	if (_finished) {
		out.state = FwControllerOutput::State::Finished;
		out.reason = "end of the trajectory";
		return out;
	}

	const float r_star = params.guidance.r_star;
	const TrajectoryPoint &current = _manager.point(_manager.currentIndex());

	if (!_captured) {
		const PathAcquisition::Output acq = _acquisition.update(pos, vel, current, r_star, params.acq_a_max);

		if (!acq.captured) {
			out.state = FwControllerOutput::State::Acquiring;
			out.a_m = acq.a_m;
			out.v_t = std::hypot(current.vx, current.vy);
			out.r = acq.distance;
			out.index = _manager.currentIndex();
			return out;
		}

		_captured = true;
		_lost_time = 0.f;
	}

	// The internal mode: TrajectoryManager -> continuous_trajectory_output -> FlightTaskTraj guidance
	const uint16_t previous_index = _manager.currentIndex();
	const ManagerOutput target = _manager.update(pos, vel, r_star, params.search_window);
	const GuidanceOutput guidance = computeGuidance(target, params.guidance);

	if (!std::isfinite(target.r) || !guidance.finite) {
		// No point of the search window at >= r*: past the end of the trajectory (PX4 falls back to index 0 here)
		const TrajectoryPoint &last = _manager.point(_manager.size() - 1);
		const TrajectoryPoint &was = _manager.point(previous_index);

		if (std::hypot(last.x - was.x, last.y - was.y) <= 2.f * r_star) {
			_finished = true;
			out.state = FwControllerOutput::State::Finished;
			out.reason = "end of the trajectory";

		} else {
			out.reason = "no virtual target in range";
		}

		return out;
	}

	const float speed = std::max(std::hypot(vel.x, vel.y), 5.f);
	const float lost_distance = std::max(3.f * r_star, 2.f * speed * speed / std::max(params.acq_a_max, 0.5f));
	// Lost: far from the virtual target, or flying away from it (course more than 90 deg off the line of sight)
	const bool lost = target.r > lost_distance || std::cos(target.lambda - target.gamma_m) < 0.f;
	_lost_time = lost ? _lost_time + dt : 0.f;

	if (_lost_time > 3.f) {
		// Lost the virtual target for 3 s: acquire the current target point again
		_captured = false;
		_acquisition.reset();
	}

	out.state = FwControllerOutput::State::Tracking;
	out.a_m = guidance.a_m;
	out.a_long = guidance.a_long;
	out.v_t = target.v_t;
	out.r = target.r;
	out.index = target.index;
	return out;
}

} // namespace trajectory_mode_fw
