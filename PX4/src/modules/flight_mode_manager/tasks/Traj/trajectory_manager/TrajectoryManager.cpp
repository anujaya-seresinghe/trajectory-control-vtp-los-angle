#include "TrajectoryManager.hpp"
#include <px4_platform_common/defines.h>


#include <mathlib/mathlib.h>

TrajectoryManager::TrajectoryManager() :

	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

}


TrajectoryManager::~TrajectoryManager() 
{
	Stop();
}



bool TrajectoryManager::Start()
{
	ScheduleDelayed(10_ms);
    if (!_continuous_trajectory_initiate_sub.registerCallback()) {
		PX4_ERR("target_estimator callback registration failed");
	}

	if (!_continuous_trajectory_setpoint_sub.registerCallback()) {
		PX4_ERR("target_estimator callback registration failed");
	}

    if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("target_estimator callback registration failed");
	}


	return true;
}

void TrajectoryManager::Stop()
{
    _vehicle_local_position_sub.unregisterCallback();

	Deinit();
}

void TrajectoryManager::Run()
{

	update();
}



void TrajectoryManager::update()
{
    // initiation

    _vehicle_local_position_sub.update(&_vehicle_local_position);


//     continuous_trajectory_initiate_s continuous_trajectory_initiate;
//     if (_continuous_trajectory_initiate_sub.update(&continuous_trajectory_initiate)) {
//             if (_traj_id != continuous_trajectory_initiate.id) {
//                 reset();
//             }
//             _no_of_wps = continuous_trajectory_initiate.no_of_waypoints;
//             _traj_id = continuous_trajectory_initiate.id;
//     }

    continuous_trajectory_setpoint_s continuous_trajectory_setpoint;
    if (_continuous_trajectory_setpoint_sub.update(&continuous_trajectory_setpoint)) {
        //if (_traj_id == continuous_trajectory_setpoint.id && !index_exists(continuous_trajectory_setpoint.index)) {
            Point *point = new Point();

            point->x = continuous_trajectory_setpoint.x;
            point->y = continuous_trajectory_setpoint.y;
            point->vx = continuous_trajectory_setpoint.vx;
            point->vy = continuous_trajectory_setpoint.vy;
            point->at = continuous_trajectory_setpoint.at;
            point->jt = continuous_trajectory_setpoint.jt;
            point->index = continuous_trajectory_setpoint.index;
	    point->t = continuous_trajectory_setpoint.t;
            wp_list.add(point);
            _current_wp_count++;
	    //PX4_INFO("handle_message_trajectory_setpoint_upload trajectory manager count: %u , index: %u", _current_wp_count,  point->index);

       // }

    }

    if (_current_wp_count == _no_of_wps) {
	if (!_trajectory_started) {
            _traj_start_time = hrt_absolute_time();
            _trajectory_started = true;
            PX4_INFO("Trajectory started! Start time: %" PRIu64, _traj_start_time);
        }
	//float elapsed_time_sec = hrt_elapsed_time(&_traj_start_time) / 1e6f;


        // find pvtp pd d r
        float r = INFINITY;
	float d = INFINITY;
	_pos_vehicle(0) = _vehicle_local_position.x;
	_pos_vehicle(1) = _vehicle_local_position.y;
	_vel_vehicle(0) = _vehicle_local_position.vx;
	_vel_vehicle(1) = _vehicle_local_position.vy;






	uint16_t tmp_index = 0;
	//selecting VTP and determing d
	Point *current_point = nullptr;
	for (Point *point : wp_list) {
		if (point-> index == _current_wp_index) {
			current_point = point;
		}
	}
	matrix::Vector2f current_wp{current_point->x, current_point->y};
	float current_distance = (current_wp - _pos_vehicle).norm();

	if (current_distance <= _r_star) {


        for (Point *point : wp_list) {
		matrix::Vector2f wp{point->x, point->y};
		float distance = (wp - _pos_vehicle).norm();
		//&&  point->index < _current_wp_index - 10
		if (distance < d ) {
			d = distance;
			_pos_d = wp;
		}



		if (point->index < _current_wp_index || point->index > _current_wp_index + 20) {
			//r = point->index;
			continue;
		}



		if (distance >= _r_star && distance < r) {
			r = distance;
			tmp_index = point->index;
			_a_t = point->at;
			_j_t = point->jt;
			_v_t(0) = point->vx;
			_v_t(1) = point->vy;
			_pos_vtp = wp;
			//PX4_INFO("VTP updated! Chosen WP index: %u, dist: %.3f", point->index, (double)distance);

		}



          	//PX4_INFO("Index: %d, N: %.2f, E: %.2f",
         	//point->index,
         	//(double)index->x,
         	//(double)index->y);
        }


	} else{
			r = current_distance;
			tmp_index = current_point->index;
			_a_t = current_point->at;
			_j_t = current_point->jt;
			_v_t(0) = current_point->vx;
			_v_t(1) = current_point->vy;
			_pos_vtp = current_wp;
	}

	_current_wp_index = tmp_index;







	//--------------------------------------- time based VTP-----------------------------------------------------------




// 	float dt = 0.026197f; // Fixed time step between waypoints
// 	//float t_start = wp_list.first()->t; // Initial trajectory time offset
// 	float t_start = 0.0f;
// 	// Compute exact index based on elapsed time
// 	int target_idx = static_cast<int>(roundf((elapsed_time_sec - t_start) / dt));

// 	// Clamp target_idx within valid array bounds
// 	if (target_idx < 0) {
// 	target_idx = 0;
// 	} else if (target_idx >= static_cast<int>(_no_of_wps)) {
// 	target_idx = static_cast<int>(_no_of_wps) - 1;
// 	}

// 	Point *vtp_point = nullptr;
// int current_idx = 0;

// for (Point *point : wp_list) {
//     if (current_idx == target_idx) {
//         vtp_point = point;
//         break;
//     }
//     current_idx++;
// }
// 	if (vtp_point != nullptr) {
// 	//tmp_index = vtp_point->index;
// 	_pos_vtp = matrix::Vector2f{vtp_point->x, vtp_point->y};
// 	_a_t = vtp_point->at;
// 	_j_t = vtp_point->jt;
// 	_v_t(0) = vtp_point->vx;
// 	_v_t(1) = vtp_point->vy;
// 	r = (_pos_vtp - _pos_vehicle).norm();
// 	_current_wp_index = vtp_point->index;
// 	} else{
// 		//PX4_INFO("vtp null");
// 	}





	//--------------------------------------- time based VTP-----------------------------------------------------------








	// calculate auxiliary variables
	// if (r < _r_star * 2) {
	// 	_current_wp_index = tmp_index;
	// }

	matrix::Vector2f los = _pos_vtp - _pos_vehicle;
	_lambda = atan2(los(1), los(0));
	_v_t_norm = _v_t.norm();
	_v_m_norm = _vel_vehicle.norm();
	_gamma_t = atan2(_v_t(1), _v_t(0));
	_gamma_m = atan2(_vel_vehicle(1), _vel_vehicle(0));
	matrix::Vector2f los_d = _pos_vtp - _pos_d;
	_lambda_d = atan2(los_d(1), los_d(0));
	//float lambda_dot = (1/r) * ((-_v_t_norm * sin(_lambda - _gamma_t)) + (_v_m_norm * sin(_lambda - _gamma_m)));
	//float r_dot = (_v_t_norm * cos(_lambda - _gamma_t)) - (_v_m_norm * cos(_lambda - _gamma_m));
	continuous_trajectory_output_s continuous_trajectory_output{};
	continuous_trajectory_output.timestamp =  hrt_absolute_time();
	continuous_trajectory_output.lambda = _lambda;
	continuous_trajectory_output.lambda_d = _lambda_d;
	continuous_trajectory_output.gamma_t = _gamma_t;
	continuous_trajectory_output.gamma_m = _gamma_m;
	continuous_trajectory_output.v_t = _v_t_norm;
	continuous_trajectory_output.v_m = _v_m_norm;
	continuous_trajectory_output.r = r;
	continuous_trajectory_output.a_t = _a_t;
	continuous_trajectory_output.j_t = _j_t;
	_continuous_trajectory_output_pub.publish(continuous_trajectory_output);
	// PX4_INFO("sending continuous_trajectory_output: lambda=%.3f, lambda_d=%.3f, gamma_t=%.3f, gamma_m=%.3f, v_t=%.3f, v_m=%.3f, r=%.3f, a_t=%.3f, j_t=%.3f",
        //          (double)_lambda,
        //          (double)_lambda_d,
        //          (double)_gamma_t,
        //          (double)_gamma_m,
        //          (double)_v_t_norm,
        //          (double)_v_m_norm,
        //          (double)r,
        //          (double)_a_t,
        //          (double)_j_t);
	PX4_INFO("r = %.3f, index = %u", (double)r, _current_wp_index);

// uint64 timestamp			# time since system start (microseconds)
// float32 lambda
// float32 lambda_d
// float32 gamma_t
// float32 gamma_m
// float32 v_t
// float32 v_m
// float32 r
// float32 a_t
// float32 j_t



    }



}

void TrajectoryManager::update_wp_list(Point wp) {

}

void TrajectoryManager::reset() {
    wp_list.clear();
    _current_wp_count = 0;
    _current_wp_index = 0;
}

bool TrajectoryManager::index_exists(uint16_t index)
{
    for (auto point : wp_list) {

        if (point->index == index) {
            return true;
        }
    }

    return false;
}
