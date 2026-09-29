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
	//sliding mode
	float _x_1 = 0;
	float _x_2 = 0;
	float _s = 0;
	float _v_m_setpoint = 1.0f;

	continuous_trajectory_output_s _continuous_trajectory_output;
	vehicle_attitude_s _vehicle_attitude;
  	TrajectoryManager _trajectory_manager;
  	//uORB::Subscription _continuous_trajectory_output_sub{ORB_ID(continuous_trajectory_output)};
  	uORB::Subscription _continuous_trajectory_output_sub{ORB_ID(continuous_trajectory_output)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};



	// Tunable at runtime (param set ...), see flight_task_traj_params.yaml
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
		(ParamFloat<px4::params::TRAJ_SMC_ALPHA>) _param_traj_smc_alpha,
		(ParamFloat<px4::params::TRAJ_SMC_BETA>) _param_traj_smc_beta,
		(ParamFloat<px4::params::TRAJ_SMC_EPS>) _param_traj_smc_eps,
		(ParamFloat<px4::params::TRAJ_R_STAR>) _param_traj_r_star,
		(ParamFloat<px4::params::TRAJ_K_LONG>) _param_traj_k_long,
		(ParamFloat<px4::params::TRAJ_A_M_MAX>) _param_traj_a_m_max,
		(ParamFloat<px4::params::TRAJ_A_LONG_MAX>) _param_traj_a_long_max
	)
};


