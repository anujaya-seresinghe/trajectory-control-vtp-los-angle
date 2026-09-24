#include "FlightTaskTraj.hpp"



FlightTaskTraj::FlightTaskTraj()
{
  _trajectory_manager.Start();
  PX4_INFO("FlightTaskTraj called!"); // report if activation was successful
}

FlightTaskTraj::~FlightTaskTraj()
{
  _trajectory_manager.Stop();
}

bool FlightTaskTraj::activate(const trajectory_setpoint_s &last_setpoint)
{
  bool ret = FlightTask::activate(last_setpoint);
//       if (!_continuous_trajectory_initiate_sub.registerCallback()) {
// 		PX4_ERR("target_estimator callback registration failed");
// 	}
  PX4_INFO("FlightTaskTraj activate was called! ret: %d", ret); // report if activation was successful
  return ret;
}

bool FlightTaskTraj::update()
{
     if (_continuous_trajectory_output_sub.updated()) {



        _vehicle_attitude_sub.update(&_vehicle_attitude);
        _continuous_trajectory_output_sub.update(&_continuous_trajectory_output);

        float lambda = _continuous_trajectory_output.lambda;
        float gamma_t = _continuous_trajectory_output.gamma_t;
        float gamma_m = _continuous_trajectory_output.gamma_m;
        float v_t = math::max(_continuous_trajectory_output.v_t, 0.1f); // Protect against division by zero
        float v_m = math::max(_continuous_trajectory_output.v_m, 0.1f);
        float r = math::max(_continuous_trajectory_output.r, 0.1f);
        float a_t = _continuous_trajectory_output.a_t;
        float j_t = _continuous_trajectory_output.j_t;

        // 1. Calculate desired LOS angle with safety clamping
       // float sin_arg = (a_t / (2.0f * v_t * v_t)) * _r_star;
        //sin_arg = math::constrain(sin_arg, -1.0f, 1.0f);
        //float lambda_d = gamma_t - asinf(sin_arg);
	float lambda_d = gamma_t - asinf((a_t / (2.0f * v_t * v_t)) * _r_star);

        // 2. Kinematic rates
        float lambda_dot = (1.0f / r) * ((-v_t * sinf(lambda - gamma_t)) + (v_m * sinf(lambda - gamma_m)));
        float lambda_d_dot = a_t / v_t;
        float r_dot = (v_t * cosf(lambda - gamma_t)) - (v_m * cosf(lambda - gamma_m));
	if (v_t * cosf(lambda - gamma_t) < 0) {
		r_dot = -(v_m * cosf(lambda - gamma_m));
	}
        // 3. Sliding mode states
        //_x_1 = matrix::wrap_pi(lambda - lambda_d); // Wrap angle differences to [-pi, pi]
	_x_1 = lambda - lambda_d;
        _x_2 = lambda_dot - lambda_d_dot;

        float sign_x2 = math::signNoZero(_x_2);
        float x_2_pow_alpha = sign_x2 * powf(std::abs(_x_2), _alpha);
        float x_2_pow_two_minus_alpha = sign_x2 * powf(std::abs(_x_2), 2.0f - _alpha);

        _s = _x_1 + (1.0f / _beta) * x_2_pow_alpha;

        // 4. Equivalent lateral acceleration (a_m_eq)
        float cos_m = cosf(lambda - gamma_m);
        if (std::abs(cos_m) < 0.05f) {
            cos_m = math::signNoZero(cos_m) * 0.05f; // Prevent singularity division
        }

        float a_m_eq = 0.0f;
        // if (r_dot <= 0.0f) {
        //     a_m_eq = (1.0f / cos_m) * (-2.0f * r_dot * lambda_dot + a_t * cosf(lambda - gamma_t) 
        //              + ((_r_star * v_m) / (r * r)) * r_dot * sinf(lambda - gamma_t) 
        //              - (j_t / v_t) * r - ((_r_star * v_m) / (r * v_t * v_t)) * a_t * r_dot 
        //              + ((r * _beta) / _alpha) * x_2_pow_two_minus_alpha);
        // } else {
        //     a_m_eq = (1.0f / std::abs(cos_m)) * (2.0f * std::abs(r_dot) * lambda_dot + a_t * cosf(lambda - gamma_t) 
        //              + ((_r_star * v_m) / (r * r)) * std::abs(r_dot) * sinf(lambda - gamma_t) 
        //              - (j_t / v_t) * r + ((_r_star * v_m) / (r * v_t * v_t)) * a_t * std::abs(r_dot) 
        //              + ((r * _beta) / _alpha) * x_2_pow_two_minus_alpha);
        // }


	if (r_dot <= 0.0f) {
            a_m_eq = (1.0f / cos_m) * (-2.0f * r_dot * lambda_dot + a_t * cosf(lambda - gamma_t) 
                     + ((_r_star * v_m) / (r * r)) * r_dot * sinf(lambda - gamma_t) 
                     - (j_t / v_t) * r - ((j_t * _r_star * r) / (2 * v_t * v_t)) 
                     + ((r * _beta) / _alpha) * x_2_pow_two_minus_alpha);
        } else {
            a_m_eq = (1.0f / std::abs(cos_m)) * (2.0f * std::abs(r_dot) * lambda_dot + a_t * cosf(lambda - gamma_t) 
                     + ((_r_star * v_m) / (r * r)) * std::abs(r_dot) * sinf(lambda - gamma_t) 
                     - (j_t / v_t) * r + ((j_t * _r_star * r) / (2 * v_t * v_t)) 
                     + ((r * _beta) / _alpha) * x_2_pow_two_minus_alpha);
        }

        float a_m_disc = (1.0f / cos_m) * _epsilon * math::signNoZero(_s);
	float a_m = a_m_eq + a_m_disc;
        //float a_m = math::constrain(a_m_eq + a_m_disc, -2.0f, 2.0f);

        // 5. Longitudinal acceleration
        //_v_m_setpoint = v_t * (r / static_cast<float>(_r_star));
	float a_long = (v_t - v_m) * _k_long;
	//float a_long = ((v_t * r/_r_star) - v_m) * _k_long;
        //float a_long = math::constrain((v_t - v_m) * _k_long, -0.5f, 0.5f);
        // FlightTaskTraj.hpp
        float _z_hold{NAN};

        // FlightTaskTraj::activate(), after FlightTask::activate(last_setpoint)
        _z_hold = _position(2);
        _position_setpoint(2) = _z_hold;

        // 6. Project acceleration vector directly into inertial NED frame
        float ax_ned = a_long * cosf(gamma_m) - a_m * sinf(gamma_m);
        float ay_ned = a_long * sinf(gamma_m) + a_m * cosf(gamma_m);

        _acceleration_setpoint(0) = ax_ned;
        _acceleration_setpoint(1) = ay_ned;
        _acceleration_setpoint(2) = 0.0f;

        _position_setpoint(2) = _z_hold;

	//_velocity_setpoint(3) = 0.0f;













	




    }

    return true;
}
