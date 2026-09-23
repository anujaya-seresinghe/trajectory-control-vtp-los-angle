#pragma once
#include <px4_platform_common/module_params.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/continuous_trajectory_initiate.h>
#include <uORB/topics/continuous_trajectory_setpoint.h>
#include <uORB/topics/continuous_trajectory_output.h>
#include <uORB/topics/vehicle_local_position.h>
#include <containers/IntrusiveSortedList.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <drivers/drv_hrt.h>

#
using namespace time_literals;
class Point : public IntrusiveSortedListNode<Point *>
{
public:
    float x;
    float y;
    float vx;
    float vy;
    float at;
    float jt;
    float t;
    int index;

    bool operator<=(const Point &other) const
    {
        return index <= other.index;
    }
};

class TrajectoryManager : public px4::ScheduledWorkItem
{

    public:
        TrajectoryManager();
	~TrajectoryManager();
        bool Start();
        void Stop();

    private:
        void Run() override;
        void update();
	hrt_abstime _traj_start_time{0};
	bool _trajectory_started{false};
	float _min_time_diff = 0.012;
        IntrusiveSortedList<Point *> wp_list;
        float _origin_z{0.f};
        uint8_t _traj_id = 0;
        uint16_t _no_of_wps = 1500;
        uint16_t _current_wp_count = 0;
        uint16_t _current_wp_index = 1;
        continuous_trajectory_initiate_s _continuous_trajectory_initiate{};
        continuous_trajectory_setpoint_s _follow_target_estimator{};
        vehicle_local_position_s _vehicle_local_position{};

	matrix::Vector2f _pos_vehicle;
	matrix::Vector2f _pos_vtp;
	matrix::Vector2f _pos_d;
	matrix::Vector2f _vel_vehicle;
	float _lambda;
	//float _lambda_dot;
	float _lambda_d;
	//float _lambda_d_dot;
	float _a_t;
	float _j_t;
	float _gamma_t;
	float _gamma_m;
	matrix::Vector2f _v_t;
	float _v_t_norm;
	float _v_m_norm;


        //variables for sliding mode control
        float x_m;
        float y_m;


        uORB::SubscriptionCallbackWorkItem _continuous_trajectory_initiate_sub{this, ORB_ID(continuous_trajectory_initiate)};
        uORB::SubscriptionCallbackWorkItem _continuous_trajectory_setpoint_sub{this, ORB_ID(continuous_trajectory_setpoint)};
        uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Publication<continuous_trajectory_output_s> _continuous_trajectory_output_pub{ORB_ID(continuous_trajectory_output)};

        void update_wp_list(Point wp);
        void reset();
        bool index_exists(uint16_t index);


	uint16_t _r_star = 30;


};
