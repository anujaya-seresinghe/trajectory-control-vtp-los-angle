#pragma once

// Fixed-wing path acquisition, used before (and after losing) the virtual-target guidance.
//
// The VTP selection only advances once the vehicle is within r* of the current target point, and the sliding-mode
// law only steers well near the path. A multicopter can slow down and reach the point; a fixed-wing at a constant
// speed with a limited turn radius R = V^2 / a_max circles it instead. Acquisition brings the vehicle onto the
// target point along the path direction:
//   LeadIn:   pursue a lead-in point L = P - d * t (d = max(3R, 4 r*), t = path tangent at P)
//   Approach: follow the line through P along t (L1-style lookahead of R) until |pos - P| <= r* while flying
//             along t (within 45 deg)
// Passing P without capturing it, or drifting far off the approach line, starts the lead-in again.

#include "trajectory_mode_fw/vtp_guidance.hpp"

namespace trajectory_mode_fw
{

class PathAcquisition
{
public:
	enum class Phase { LeadIn, Approach };

	struct Output {
		bool captured{false}; // within r* of the target point: hand over to the VTP guidance
		float a_m{0.f};       // lateral acceleration command, flight-path y (right positive) [m/s^2]
		float distance{0.f};  // to the target point [m]
		Phase phase{Phase::LeadIn};
	};

	void reset() { _phase = Phase::LeadIn; }

	// target: point to acquire (x, y, direction vx/vy); speed: ground speed used for the turn radius [m/s]
	Output update(Vec2 pos, Vec2 vel, const TrajectoryPoint &target, float r_star, float a_max);

private:
	Phase _phase{Phase::LeadIn};
};

} // namespace trajectory_mode_fw
