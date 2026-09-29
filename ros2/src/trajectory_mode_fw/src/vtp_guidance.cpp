#include "trajectory_mode_fw/vtp_guidance.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_mode_fw
{

namespace
{

float norm(Vec2 v) { return std::sqrt(v.x * v.x + v.y * v.y); }
Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
float signNoZero(float v) { return v < 0.f ? -1.f : 1.f; } // math::signNoZero
float constrain(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

} // namespace

void TrajectoryManager::load(const Trajectory &trajectory)
{
	// reset() + INITIATE (id, number of points) + every UPLOAD, indexed 0..N-1
	_points = trajectory.points;
	_id = trajectory.id;
	_current_wp_index = 0;
}

void TrajectoryManager::clear()
{
	_points.clear();
	_current_wp_index = 0;
}

ManagerOutput TrajectoryManager::update(Vec2 pos_vehicle, Vec2 vel_vehicle, float r_star, int search_window)
{
	ManagerOutput out;

	if (_points.empty()) {
		out.r = INFINITY;
		return out;
	}

	float r = INFINITY;
	float d = INFINITY;
	uint16_t tmp_index = 0;

	// PX4 dereferences the current point without a check; the index always exists here
	const TrajectoryPoint &current_point = _points[std::min<size_t>(_current_wp_index, _points.size() - 1)];
	const Vec2 current_wp{current_point.x, current_point.y};
	const float current_distance = norm(sub(current_wp, pos_vehicle));

	if (current_distance <= r_star) {
		for (size_t i = 0; i < _points.size(); ++i) {
			const TrajectoryPoint &point = _points[i];
			const Vec2 wp{point.x, point.y};
			const float distance = norm(sub(wp, pos_vehicle));

			if (distance < d) {
				d = distance;
				_pos_d = wp;
			}

			if (i < _current_wp_index || i > static_cast<size_t>(_current_wp_index) + search_window) {
				continue;
			}

			if (distance >= r_star && distance < r) {
				r = distance;
				tmp_index = static_cast<uint16_t>(i);
				_a_t = point.at;
				_j_t = point.jt;
				_v_t = {point.vx, point.vy};
				_pos_vtp = wp;
			}
		}

	} else {
		r = current_distance;
		tmp_index = _current_wp_index;
		_a_t = current_point.at;
		_j_t = current_point.jt;
		_v_t = {current_point.vx, current_point.vy};
		_pos_vtp = current_wp;
	}

	_current_wp_index = tmp_index;

	const Vec2 los = sub(_pos_vtp, pos_vehicle);
	const Vec2 los_d = sub(_pos_vtp, _pos_d);
	out.lambda = std::atan2(los.y, los.x);
	out.lambda_d = std::atan2(los_d.y, los_d.x);
	out.v_t = norm(_v_t);
	out.v_m = norm(vel_vehicle);
	out.gamma_t = std::atan2(_v_t.y, _v_t.x);
	out.gamma_m = std::atan2(vel_vehicle.y, vel_vehicle.x);
	out.r = r;
	out.a_t = _a_t;
	out.j_t = _j_t;
	out.index = _current_wp_index;
	return out;
}

GuidanceOutput computeGuidance(const ManagerOutput &in, const GuidanceParams &p)
{
	const float alpha = p.alpha;
	const float beta = p.beta;
	const float epsilon = p.epsilon;
	const float r_star = p.r_star;
	const float k_long = p.k_long;

	const float lambda = in.lambda;
	const float gamma_t = in.gamma_t;
	const float gamma_m = in.gamma_m;
	const float v_t = std::max(in.v_t, 0.1f); // Protect against division by zero
	const float v_m = std::max(in.v_m, 0.1f);
	const float r = std::max(in.r, 0.1f);
	const float a_t = in.a_t;
	const float j_t = in.j_t;

	// 1. Desired LOS angle
	const float lambda_d = gamma_t - std::asin((a_t / (2.0f * v_t * v_t)) * r_star);

	// 2. Kinematic rates
	const float lambda_dot = (1.0f / r) * ((-v_t * std::sin(lambda - gamma_t)) + (v_m * std::sin(lambda - gamma_m)));
	const float lambda_d_dot = a_t / v_t;
	float r_dot = (v_t * std::cos(lambda - gamma_t)) - (v_m * std::cos(lambda - gamma_m));

	if (v_t * std::cos(lambda - gamma_t) < 0) {
		r_dot = -(v_m * std::cos(lambda - gamma_m));
	}

	// 3. Sliding mode states
	const float x_1 = lambda - lambda_d;
	const float x_2 = lambda_dot - lambda_d_dot;

	const float sign_x2 = signNoZero(x_2);
	const float x_2_pow_alpha = sign_x2 * std::pow(std::abs(x_2), alpha);
	const float x_2_pow_two_minus_alpha = sign_x2 * std::pow(std::abs(x_2), 2.0f - alpha);

	const float s = x_1 + (1.0f / beta) * x_2_pow_alpha;

	// 4. Equivalent lateral acceleration
	float cos_m = std::cos(lambda - gamma_m);

	if (std::abs(cos_m) < 0.05f) {
		cos_m = signNoZero(cos_m) * 0.05f; // Prevent singularity division
	}

	float a_m_eq = 0.0f;

	if (r_dot <= 0.0f) {
		a_m_eq = (1.0f / cos_m) * (-2.0f * r_dot * lambda_dot + a_t * std::cos(lambda - gamma_t)
					   + ((r_star * v_m) / (r * r)) * r_dot * std::sin(lambda - gamma_t)
					   - (j_t / v_t) * r - ((r_star * v_m) / (r * v_t * v_t)) * a_t * r_dot
					   + ((r * beta) / alpha) * x_2_pow_two_minus_alpha);

	} else {
		a_m_eq = (1.0f / std::abs(cos_m)) * (2.0f * std::abs(r_dot) * lambda_dot + a_t * std::cos(lambda - gamma_t)
						     + ((r_star * v_m) / (r * r)) * std::abs(r_dot) * std::sin(lambda - gamma_t)
						     - (j_t / v_t) * r + ((r_star * v_m) / (r * v_t * v_t)) * a_t * std::abs(r_dot)
						     + ((r * beta) / alpha) * x_2_pow_two_minus_alpha);
	}

	const float a_m_disc = (1.0f / cos_m) * epsilon * signNoZero(s);

	GuidanceOutput out;
	out.a_m = constrain(a_m_eq + a_m_disc, -p.a_m_max, p.a_m_max);

	// 5. Longitudinal acceleration
	out.a_long = constrain((v_t - v_m) * k_long, -p.a_long_max, p.a_long_max);
	out.s = s;
	out.finite = std::isfinite(out.a_m) && std::isfinite(out.a_long);
	return out;
}

} // namespace trajectory_mode_fw
