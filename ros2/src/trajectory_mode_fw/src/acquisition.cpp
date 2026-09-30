#include "trajectory_mode_fw/acquisition.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_mode_fw
{

namespace
{

float wrapPi(float a)
{
	while (a > static_cast<float>(M_PI)) { a -= 2.f * static_cast<float>(M_PI); }

	while (a < -static_cast<float>(M_PI)) { a += 2.f * static_cast<float>(M_PI); }

	return a;
}

// Pursuit of a point with an L1-style law, a = 2 V^2 sin(eta) / L1, with L1 >= R so it does not orbit the point.
// Turns at full rate while the point is behind.
float pursue(Vec2 pos, float course, float speed, Vec2 aim, float turn_radius, float a_max)
{
	const float dx = aim.x - pos.x;
	const float dy = aim.y - pos.y;
	const float eta = wrapPi(std::atan2(dy, dx) - course); // positive: aim is to the right (NED)

	if (std::abs(eta) > static_cast<float>(M_PI_2)) {
		return eta > 0.f ? a_max : -a_max;
	}

	const float l1 = std::max(std::hypot(dx, dy), turn_radius);
	return std::clamp(2.f * speed * speed * std::sin(eta) / l1, -a_max, a_max);
}

} // namespace

PathAcquisition::Output PathAcquisition::update(Vec2 pos, Vec2 vel, const TrajectoryPoint &target, float r_star,
		float a_max)
{
	Output out;
	const float speed = std::max(std::hypot(vel.x, vel.y), 5.f);
	const float course = std::atan2(vel.y, vel.x);
	a_max = std::max(a_max, 0.5f);
	const float turn_radius = speed * speed / a_max;
	const float lead = std::max(3.f * turn_radius, 4.f * r_star);

	// Path direction at the target point
	float tx = target.vx;
	float ty = target.vy;
	const float tn = std::hypot(tx, ty);

	if (tn > 1e-3f) {
		tx /= tn;
		ty /= tn;

	} else {
		tx = std::cos(course);
		ty = std::sin(course);
	}

	const Vec2 p{target.x, target.y};
	const float rx = pos.x - p.x;
	const float ry = pos.y - p.y;
	const float along = rx * tx + ry * ty;  // < 0: before the point
	const float cross = rx * ty - ry * tx;  // distance from the approach line
	out.distance = std::hypot(rx, ry);

	// Capture only when also flying along the path (within 45 deg): the sliding-mode law does not recover from a
	// capture while flying across or against it
	const bool aligned = std::cos(wrapPi(course - std::atan2(ty, tx))) > 0.7f;

	if (out.distance <= r_star && aligned) {
		out.captured = true;
		out.phase = _phase;
		return out;
	}

	const Vec2 lead_in{p.x - lead * tx, p.y - lead * ty};

	if (_phase == Phase::LeadIn) {
		const bool near_lead_in = std::hypot(lead_in.x - pos.x, lead_in.y - pos.y) < turn_radius;
		const bool on_line = along < -turn_radius && along > -2.f * lead && std::abs(cross) < 0.5f * turn_radius
				     && std::cos(wrapPi(course - std::atan2(ty, tx))) > 0.9f;

		if (near_lead_in || on_line) {
			_phase = Phase::Approach;
		}

	} else if (along > r_star || std::abs(cross) > 2.f * turn_radius || along < -(lead + 3.f * turn_radius)) {
		// Passed the point without capturing it, or pushed far off the approach line: go around again
		_phase = Phase::LeadIn;
	}

	Vec2 aim = lead_in;

	if (_phase == Phase::Approach) {
		// Lookahead point on the approach line, never beyond the target point
		const float s = std::min(along + turn_radius, 0.f);
		aim = {p.x + s * tx, p.y + s * ty};
	}

	out.phase = _phase;
	out.a_m = pursue(pos, course, speed, aim, turn_radius, a_max);
	return out;
}

} // namespace trajectory_mode_fw
