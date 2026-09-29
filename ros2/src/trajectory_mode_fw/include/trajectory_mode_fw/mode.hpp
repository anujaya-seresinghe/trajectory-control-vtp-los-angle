#pragma once

// External PX4 flight mode "Trajectory" for fixed-wing flight (VTOL in FW mode).
//
// Runs the same virtual-target guidance as the PX4-internal Trajectory mode (TrajectoryManager +
// FlightTaskTraj) and converts its outputs into fixed-wing lateral/longitudinal setpoints:
//   a_m    (lateral acceleration, flight-path y, right positive) -> FixedWingLateralSetpoint.lateral_acceleration
//   a_long (longitudinal acceleration)                           -> speed reference -> FixedWingLongitudinalSetpoint.equivalent_airspeed
//   altitude                                                     -> held at the value on activation

#include <memory>
#include <optional>

#include <px4_msgs/msg/wind.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/odometry/airspeed.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/subscription.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "trajectory_mode_fw/mqtt_trajectory_source.hpp"
#include "trajectory_mode_fw/vtp_guidance.hpp"

namespace trajectory_mode_fw
{

class TrajectoryModeFw : public px4_ros2::ModeBase
{
public:
	static constexpr const char *kName = "Trajectory";

	explicit TrajectoryModeFw(rclcpp::Node &node);

	// True once a VTOL has finished the transition to fixed-wing (or for a plain fixed-wing).
	// PX4 only accepts fixed-wing setpoints while vehicle_type == FIXED_WING, and px4_ros2 checks the
	// setpoint type once at registration, so the mode can only be registered in fixed-wing flight.
	bool inFixedWingFlight() const;

	void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter &reporter) override;
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	GuidanceParams readParams();
	void declareParameters();
	void handleMqttCommand(const std::string &command, const std::string &payload); // MQTT thread
	void publishParam(const std::string &px4_name, const std::string &error = "");
	void persistParameters(const std::vector<rclcpp::Parameter> &parameters);
	void loadPendingTrajectory();
	float equivalentAirspeedFor(float ground_speed_sp, float course) const;
	void holdCourse(const char *reason);
	// Guidance state for the web app (<prefix>/<sysid>/ros_guidance), at most every 200 ms unless forced
	void publishGuidance(const char *state, const ManagerOutput *target, const GuidanceOutput *guidance,
			     bool force = false);

	rclcpp::Node &_node;

	std::shared_ptr<px4_ros2::FwLateralLongitudinalSetpointType> _fw_setpoint;
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
	std::shared_ptr<px4_ros2::OdometryAirspeed> _airspeed;
	std::shared_ptr<px4_ros2::VehicleStatus> _vehicle_status;
	std::shared_ptr<px4_ros2::Subscription<px4_msgs::msg::Wind>> _wind;

	std::unique_ptr<MqttTrajectorySource> _mqtt;
	std::string _status_topic; // <prefix>/<sysid>/ros_trajectory_status
	std::string _param_topic;  // <prefix>/<sysid>/ros_param
	std::string _guidance_topic; // <prefix>/<sysid>/ros_guidance
	std::string _params_file;  // YAML that runtime parameter changes are written back to ("" = don't persist)
	rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr _post_set_handle;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _trajectory_json_sub;
	std::optional<Trajectory> _pending_ros; // from the ROS topic (executor thread only)

	TrajectoryManager _manager;

	float _altitude_hold_amsl{NAN}; // captured on activation
	float _course_hold{NAN};        // used while there is no valid virtual target
	float _speed_ref{NAN};          // ground speed reference driven by the internal a_long law
	const char *_hold_reason{nullptr};
	rclcpp::Time _last_log_time{0, 0, RCL_ROS_TIME};
	rclcpp::Time _last_guidance_pub{0, 0, RCL_ROS_TIME};
};

} // namespace trajectory_mode_fw
