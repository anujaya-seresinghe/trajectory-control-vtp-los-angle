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
	continuous_trajectory_initiate_s init;
if (_continuous_trajectory_initiate_sub.update(&init)) {
    reset();
    _traj_id = init.id;
    _no_of_wps = init.no_of_waypoints;
	PX4_INFO("trajectory initiated, no_of_wps:%u", _no_of_wps);
}



    continuous_trajectory_setpoint_s continuous_trajectory_setpoint;
    if (_continuous_trajectory_setpoint_sub.update(&continuous_trajectory_setpoint)) {
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
			PX4_INFO("waypoint received, count; %u", _current_wp_count);


    }

    if (_no_of_wps > 0 && _current_wp_count == _no_of_wps) {
	if (!_trajectory_started) {
            _traj_start_time = hrt_absolute_time();
            _trajectory_started = true;
            PX4_INFO("Trajectory started! Start time: %" PRIu64, _traj_start_time);
        }

        float r = INFINITY;
	float d = INFINITY;
	_pos_vehicle(0) = _vehicle_local_position.x;
	_pos_vehicle(1) = _vehicle_local_position.y;
	_vel_vehicle(0) = _vehicle_local_position.vx;
	_vel_vehicle(1) = _vehicle_local_position.vy;






	uint16_t tmp_index = 0;
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
		if (distance < d ) {
			d = distance;
			_pos_d = wp;
		}



		if (point->index < _current_wp_index || point->index > _current_wp_index + 100) {
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
		

		}



          	
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








	matrix::Vector2f los = _pos_vtp - _pos_vehicle;
	_lambda = atan2(los(1), los(0));
	_v_t_norm = _v_t.norm();
	_v_m_norm = _vel_vehicle.norm();
	_gamma_t = atan2(_v_t(1), _v_t(0));
	_gamma_m = atan2(_vel_vehicle(1), _vel_vehicle(0));
	matrix::Vector2f los_d = _pos_vtp - _pos_d;
	_lambda_d = atan2(los_d(1), los_d(0));
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
	PX4_INFO("r = %.3f, index = %u", (double)r, _current_wp_index);





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
