#pragma once

// Minimal fixed-wing model for the unit tests: constant ground speed, lateral acceleration through a bank angle
// with a first-order roll response (like PX4's roll controller, a = g tan(roll)), bank limited to roll_limit.

#include <algorithm>
#include <cmath>
#include <vector>

#include "trajectory_mode_fw/fw_controller.hpp"

namespace trajectory_mode_fw::sim
{

struct Aircraft {
	float x{0.f}, y{0.f}, course{0.f}, roll{0.f};
	float speed{16.f};
	float roll_tau{0.6f};                 // [s]
	float roll_limit{45.f * M_PI / 180.f};

	Vec2 pos() const { return {x, y}; }
	Vec2 vel() const { return {speed * std::cos(course), speed * std::sin(course)}; }

	void step(float a_cmd, float dt)
	{
		constexpr float g = 9.81f;
		const float roll_cmd = std::clamp(std::atan(a_cmd / g), -roll_limit, roll_limit);
		roll += (roll_cmd - roll) * std::min(dt / roll_tau, 1.f);
		course += g * std::tan(roll) / speed * dt;
		x += speed * std::cos(course) * dt;
		y += speed * std::sin(course) * dt;
	}
};

// Path like the web planner: 0.5 m spacing, constant speed, signed at = V^2 kappa (positive = right turn)
inline Trajectory makePath(float x0, float y0, float heading, float speed = 16.f)
{
	struct Segment { float length; float curvature; };
	const Segment segments[] = {{300.f, 0.f}, {float(M_PI) * 200.f / 2.f, 1.f / 200.f}, {200.f, 0.f},
		{float(M_PI) * 150.f / 2.f, -1.f / 150.f}, {300.f, 0.f}};
	Trajectory t;
	t.id = 1;
	float x = x0, y = y0, h = heading, s = 0.f;

	for (const Segment &seg : segments) {
		for (float d = 0.f; d < seg.length; d += 0.5f) {
			t.points.push_back({x, y, speed * std::cos(h), speed * std::sin(h), speed * speed * seg.curvature, 0.f,
					    s / speed});
			x += 0.5f * std::cos(h);
			y += 0.5f * std::sin(h);
			h += 0.5f * seg.curvature;
			s += 0.5f;
		}
	}

	return t;
}

inline float distanceToPath(const Trajectory &t, Vec2 p)
{
	float d = INFINITY;

	for (const TrajectoryPoint &q : t.points) {
		d = std::min(d, std::hypot(q.x - p.x, q.y - p.y));
	}

	return d;
}

struct Result {
	bool captured{false};
	float capture_time{NAN};
	bool finished{false};
	float max_error{0.f};   // distance to the path while tracking, after settling [m]
	float mean_error{0.f};
	float max_index_fraction{0.f};
};

inline Result fly(Aircraft ac, const Trajectory &path, const FwControllerParams &params, float duration = 400.f,
		  float settle = 25.f)
{
	FwTrajectoryController controller;
	controller.load(path);
	Result res;
	constexpr float dt = 0.02f;
	double error_sum = 0.0;
	int error_n = 0;

	for (float t = 0.f; t < duration; t += dt) {
		const FwControllerOutput out = controller.update(ac.pos(), ac.vel(), dt, params);

		if (out.state == FwControllerOutput::State::Finished) {
			res.finished = true;
			break;
		}

		if (out.state == FwControllerOutput::State::Tracking && !res.captured) {
			res.captured = true;
			res.capture_time = t;
		}

		if (out.state == FwControllerOutput::State::Tracking) {
			res.max_index_fraction = std::max(res.max_index_fraction, float(out.index) / float(path.points.size()));
		}

		if (res.captured && t > res.capture_time + settle && int(t / dt) % 25 == 0) {
			const float e = distanceToPath(path, ac.pos());
			res.max_error = std::max(res.max_error, e);
			error_sum += e;
			++error_n;
		}

		const bool steering = out.state == FwControllerOutput::State::Tracking
				      || out.state == FwControllerOutput::State::Acquiring;
		ac.step(steering ? out.a_m : 0.f, dt);
	}

	res.mean_error = error_n ? float(error_sum / error_n) : 0.f;
	return res;
}

} // namespace trajectory_mode_fw::sim
