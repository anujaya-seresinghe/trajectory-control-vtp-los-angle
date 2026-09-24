#pragma once

#include "FlightTask.hpp"

#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <containers/IntrusiveSortedList.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include "trajectory_manager/TrajectoryManager.hpp"


#include <uORB/topics/continuous_trajectory_output.h>
#include <uORB/topics/continuous_trajectory_initiate.h>
#include <uORB/topics/continuous_trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>





class FlightTaskTraj : public FlightTask
{
public:
  	FlightTaskTraj();
  	virtual ~FlightTaskTraj();
  	bool activate(const trajectory_setpoint_s &last_setpoint) override;
  	bool update() override;
  //bool activate(const trajectory_setpoint_s &last_setpoint) override;
//subcribe to trajectory initiator, trajectory
private:
	float _alpha = 1.4f;
	float _beta = 0.2f;
	float _epsilon = 0.1f;


	//sliding mode
	float _x_1 = 0;
	float _x_2 = 0;
	float _s = 0;
	float _k_long = 0.5;
	float _v_m_setpoint = 1.0f;

	continuous_trajectory_output_s _continuous_trajectory_output;
	vehicle_attitude_s _vehicle_attitude;
  	TrajectoryManager _trajectory_manager;
  	//uORB::Subscription _continuous_trajectory_output_sub{ORB_ID(continuous_trajectory_output)};
  	uORB::Subscription _continuous_trajectory_output_sub{ORB_ID(continuous_trajectory_output)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};



	uint16_t _r_star = 15;
};


