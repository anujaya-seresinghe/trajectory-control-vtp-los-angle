#include "trajectory_mode_fw/mode.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <px4_ros2/components/events.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include "trajectory_mode_fw/yaml_params.hpp"

using namespace std::chrono_literals;

namespace trajectory_mode_fw
{

namespace
{

// Guidance parameters: PX4 name (used by the web app's parameter panel), ROS name, default and range.
// Same meaning, defaults and ranges as PX4/.../Traj/flight_task_traj_params.yaml.
struct GuidanceParamDef {
	const char *px4_name;
	const char *ros_name;
	double default_value;
	double min;
	double max;
	const char *description;
};

constexpr GuidanceParamDef kGuidanceParams[] = {
	{"TRAJ_SMC_ALPHA", "traj_smc_alpha", 1.4, 1.01, 1.99, "Sliding mode exponent alpha"},
	{"TRAJ_SMC_BETA", "traj_smc_beta", 0.12, 0.001, 10.0, "Sliding mode gain beta"},
	{"TRAJ_SMC_EPS", "traj_smc_eps", 0.05, 0.0, 10.0, "Sliding mode switching gain epsilon"},
	{"TRAJ_R_STAR", "traj_r_star", 15.0, 1.0, 200.0, "Virtual target distance r* [m]"},
	{"TRAJ_K_LONG", "traj_k_long", 0.5, 0.0, 10.0, "Longitudinal speed gain"},
	{"TRAJ_A_M_MAX", "traj_a_m_max", 2.0, 0.0, 20.0, "Lateral acceleration limit [m/s^2]"},
	{"TRAJ_A_LONG_MAX", "traj_a_long_max", 0.3, 0.0, 10.0, "Longitudinal acceleration limit [m/s^2]"},
	// Fixed-wing only (no PX4 equivalent): lateral acceleration limit while acquiring the path, sets the turn radius
	{"TRAJ_ACQ_A_MAX", "traj_acq_a_max", 4.0, 0.5, 20.0, "Lateral acceleration limit while acquiring the path [m/s^2]"},
};

const GuidanceParamDef *findByPx4Name(const std::string &name)
{
	for (const auto &def : kGuidanceParams) {
		if (name == def.px4_name) {
			return &def;
		}
	}

	return nullptr;
}

const GuidanceParamDef *findByRosName(const std::string &name)
{
	for (const auto &def : kGuidanceParams) {
		if (name == def.ros_name) {
			return &def;
		}
	}

	return nullptr;
}

} // namespace

TrajectoryModeFw::TrajectoryModeFw(rclcpp::Node &node)
	: ModeBase(node, Settings(kName)), _node(node)
{
	_fw_setpoint = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
	_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_airspeed = std::make_shared<px4_ros2::OdometryAirspeed>(*this);
	_vehicle_status = std::make_shared<px4_ros2::VehicleStatus>(*this);
	_wind = std::make_shared<px4_ros2::Subscription<px4_msgs::msg::Wind>>(
			*this, "fmu/out/wind" + px4_ros2::getMessageNameVersion<px4_msgs::msg::Wind>());

	declareParameters();

	node.declare_parameter("sysid", 1);
	node.declare_parameter("mavlink_compid", 191); // MAV_COMP_ID_ONBOARD_COMPUTER
	// Own PX4 MAVLink instance with forwarding (see config/params.yaml): PX4 sends here...
	node.declare_parameter("mavlink_local_port", 14590);
	node.declare_parameter("mavlink_remote_host", "127.0.0.1");
	node.declare_parameter("mavlink_remote_port", 14591); // ...and listens here

	MavlinkLink::Config link;
	link.sysid = static_cast<uint8_t>(node.get_parameter("sysid").as_int());
	link.compid = static_cast<uint8_t>(node.get_parameter("mavlink_compid").as_int());
	link.local_port = static_cast<int>(node.get_parameter("mavlink_local_port").as_int());
	link.remote_host = node.get_parameter("mavlink_remote_host").as_string();
	link.remote_port = static_cast<int>(node.get_parameter("mavlink_remote_port").as_int());

	std::vector<std::string> names;

	for (const auto &def : kGuidanceParams) {
		names.emplace_back(def.px4_name);
	}

	// Parameter access from the MAVLink thread (rclcpp parameter access is thread-safe)
	_mavlink = std::make_unique<MavlinkLink>(
			   link, names,
	[this](const std::string & name) -> std::optional<float> {
		const GuidanceParamDef *def = findByPx4Name(name);
		return def ? std::optional<float>(static_cast<float>(_node.get_parameter(def->ros_name).as_double())) : std::nullopt;
	},
	[this](const std::string & name, float value) -> std::string {
		const GuidanceParamDef *def = findByPx4Name(name);

		if (value < def->min || value > def->max) {
			// Short enough for a STATUSTEXT (50 characters with the name)
			return "range " + yamlDouble(def->min) + ".." + yamlDouble(def->max);
		}

		// MAVLink parameters are float: store the shortest decimal of that float (1.4, not 1.399999976)
		char text[32];
		snprintf(text, sizeof(text), "%.7g", static_cast<double>(value));
		const auto result = _node.set_parameter(rclcpp::Parameter(def->ros_name, std::strtod(text, nullptr)));
		return result.successful ? "" : result.reason;
	},
	[this](const std::string & s) { RCLCPP_INFO(_node.get_logger(), "%s", s.c_str()); },
	[this](const std::string & s) { RCLCPP_WARN(_node.get_logger(), "%s", s.c_str()); });

	_guidance_timer = node.create_wall_timer(200ms, [this]() { publishGuidance(); });

	// Alternative input for ROS-only setups: the same JSON as a std_msgs/String
	_trajectory_json_sub = node.create_subscription<std_msgs::msg::String>(
				       "~/trajectory_json", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
		Trajectory trajectory;
		std::string error;

		if (parseTrajectoryJson(msg->data, trajectory, error)) {
			_pending_ros = std::move(trajectory);

		} else {
			RCLCPP_WARN(_node.get_logger(), "Rejected trajectory from ROS topic: %s", error.c_str());
		}
	});
}


void TrajectoryModeFw::declareParameters()
{
	// All of them can be changed at runtime (web app parameter panel, or
	// ros2 param set /trajectory_mode_fw traj_r_star 30.0); changes are written back to params_file.
	for (const auto &def : kGuidanceParams) {
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		descriptor.description = def.description;
		rcl_interfaces::msg::FloatingPointRange range;
		range.from_value = def.min;
		range.to_value = def.max;
		range.step = 0.0;
		descriptor.floating_point_range.push_back(range);
		_node.declare_parameter(def.ros_name, def.default_value, descriptor);
	}

	_node.declare_parameter("traj_search_window", 100); // indices ahead of the current target (TrajectoryManager)
	_node.declare_parameter("params_file", "");
	_params_file = _node.get_parameter("params_file").as_string();

	_post_set_handle = _node.add_post_set_parameters_callback(
	[this](const std::vector<rclcpp::Parameter> &parameters) { persistParameters(parameters); });
}

void TrajectoryModeFw::persistParameters(const std::vector<rclcpp::Parameter> &parameters)
{
	std::vector<std::pair<std::string, std::string>> values;

	for (const rclcpp::Parameter &parameter : parameters) {
		if (const GuidanceParamDef *def = findByRosName(parameter.get_name())) {
			values.emplace_back(parameter.get_name(), yamlDouble(parameter.as_double()));

			// Confirm to the ground station (also for changes made with ros2 param set), like PX4 does
			if (_mavlink) {
				_mavlink->sendParamValue(def->px4_name);
			}

		} else if (parameter.get_name() == "traj_search_window") {
			values.emplace_back(parameter.get_name(), std::to_string(parameter.as_int()));
		}
	}

	if (values.empty() || _params_file.empty()) {
		return;
	}

	std::string error;

	if (updateYamlFile(_params_file, values, error)) {
		RCLCPP_INFO(_node.get_logger(), "Saved %s = %s to %s", values.front().first.c_str(), values.front().second.c_str(),
			    _params_file.c_str());

	} else {
		RCLCPP_WARN(_node.get_logger(), "Could not save parameters: %s", error.c_str());
	}
}

bool TrajectoryModeFw::inFixedWingFlight() const
{
	return _vehicle_status->lastValid(3s)
	       && _vehicle_status->last().vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING
	       && !_vehicle_status->last().in_transition_mode;
}

void TrajectoryModeFw::checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter &reporter)
{
	// Fixed-wing setpoints only make sense once a VTOL has finished the transition to fixed-wing flight
	if (!inFixedWingFlight()) {
		reporter.armingCheckFailureExt(px4_ros2::events::ID("trajectory_fw_not_fixed_wing"),
					       px4_ros2::events::Log::Error,
					       "Trajectory (fixed-wing) is only available in fixed-wing flight");
	}
}

void TrajectoryModeFw::onActivate()
{
	// Like the internal mode, the trajectory is followed from its first point after every activation.
	// A trajectory that arrived while the mode was inactive is picked up now.
	_controller.clear();
	_hold_reason = nullptr;

	const auto &pos = _local_position->last();
	_altitude_hold_amsl = (pos.z_global && _local_position->positionZValid()) ? pos.ref_alt - pos.z : NAN;

	const Eigen::Vector3f vel = _local_position->velocityNed();
	_course_hold = vel.head<2>().norm() > 1.f ? std::atan2(vel.y(), vel.x()) : _local_position->heading();
	_speed_ref = vel.head<2>().norm();

	RCLCPP_INFO(_node.get_logger(), "Trajectory (fixed-wing) active, holding %.1f m AMSL", _altitude_hold_amsl);
}

void TrajectoryModeFw::onDeactivate()
{
	_controller.clear();
	_guidance_state = GuidanceState::Inactive;
}

void TrajectoryModeFw::publishGuidance()
{
	// While inactive only the state and the number of loaded points, once per second
	if (_guidance_state == GuidanceState::Inactive && _guidance_ticks++ % 5 != 0) {
		return;
	}

	_mavlink->sendNamedValue("TRAJ_STATE", static_cast<float>(_guidance_state));
	_mavlink->sendNamedValue("TRAJ_N", static_cast<float>(_controller.size()));

	// Tracking: r to the virtual target. Acquiring: distance to the point being acquired.
	if (_guidance_state == GuidanceState::Tracking || _guidance_state == GuidanceState::Acquiring) {
		_mavlink->sendNamedValue("TRAJ_R", _last_output.r);
		_mavlink->sendNamedValue("TRAJ_IDX", static_cast<float>(_last_output.index));
		_mavlink->sendNamedValue("TRAJ_A_M", _last_output.a_m);
		_mavlink->sendNamedValue("TRAJ_ALONG", _last_output.a_long);
	}
}

FwControllerParams TrajectoryModeFw::readParams()
{
	FwControllerParams p;
	p.guidance.alpha = static_cast<float>(_node.get_parameter("traj_smc_alpha").as_double());
	p.guidance.beta = static_cast<float>(_node.get_parameter("traj_smc_beta").as_double());
	p.guidance.epsilon = static_cast<float>(_node.get_parameter("traj_smc_eps").as_double());
	p.guidance.r_star = static_cast<float>(_node.get_parameter("traj_r_star").as_double());
	p.guidance.k_long = static_cast<float>(_node.get_parameter("traj_k_long").as_double());
	p.guidance.a_m_max = static_cast<float>(_node.get_parameter("traj_a_m_max").as_double());
	p.guidance.a_long_max = static_cast<float>(_node.get_parameter("traj_a_long_max").as_double());
	p.search_window = static_cast<int>(_node.get_parameter("traj_search_window").as_int());
	p.acq_a_max = static_cast<float>(_node.get_parameter("traj_acq_a_max").as_double());
	return p;
}

void TrajectoryModeFw::loadPendingTrajectory()
{
	std::optional<Trajectory> trajectory = _mavlink->take();

	if (_pending_ros) {
		trajectory = std::move(_pending_ros);
		_pending_ros.reset();
	}

	if (trajectory) {
		_controller.load(*trajectory);
		_hold_reason = nullptr;
		RCLCPP_INFO(_node.get_logger(), "Trajectory %u loaded (%zu points)", trajectory->id, trajectory->points.size());
	}
}

float TrajectoryModeFw::equivalentAirspeedFor(float ground_speed_sp, float course) const
{
	// Ground velocity setpoint along the current flight path -> air-relative velocity (wind triangle)
	float air_n = ground_speed_sp * std::cos(course);
	float air_e = ground_speed_sp * std::sin(course);

	if (_wind->lastValid(2s)) {
		air_n -= _wind->last().windspeed_north;
		air_e -= _wind->last().windspeed_east;
	}

	const float tas = std::hypot(air_n, air_e);

	// TAS -> EAS with the ratio PX4 currently measures (~1 near sea level)
	float eas_per_tas = 1.f;

	if (_airspeed->lastValid(1s)) {
		const float cas = _airspeed->calibratedAirspeed();
		const float tas_measured = _airspeed->trueAirspeed();

		if (std::isfinite(cas) && std::isfinite(tas_measured) && tas_measured > 1.f) {
			eas_per_tas = cas / tas_measured;
		}
	}

	return tas * eas_per_tas;
}

void TrajectoryModeFw::holdCourse(const char *reason)
{
	// A fixed-wing cannot stop and wait: fly straight on the last course at the held altitude
	if (reason != _hold_reason) {
		RCLCPP_WARN(_node.get_logger(), "Holding course %.0f deg: %s", _course_hold * 180.0 / M_PI, reason);
		_hold_reason = reason;
	}

	_guidance_state = GuidanceState::Holding;

	px4_ros2::FwLateralLongitudinalSetpoint sp;
	sp.withCourse(_course_hold);

	if (std::isfinite(_altitude_hold_amsl)) {
		sp.withAltitude(_altitude_hold_amsl);
	}

	_fw_setpoint->update(sp);
}

void TrajectoryModeFw::updateSetpoint(float dt_s)
{
	loadPendingTrajectory();

	if (!_local_position->positionXYValid() || !_local_position->velocityXYValid()) {
		holdCourse("no valid local position");
		return;
	}

	if (!_controller.active()) {
		holdCourse("waiting for a trajectory");
		return;
	}

	const FwControllerParams params = readParams();
	const Eigen::Vector3f pos = _local_position->positionNed();
	const Eigen::Vector3f vel = _local_position->velocityNed();
	const float dt = (dt_s > 0.f && dt_s < 0.5f) ? dt_s : 0.f;

	// Acquisition until the vehicle is on the path, then TrajectoryManager -> FlightTaskTraj guidance
	const FwControllerOutput out = _controller.update({pos.x(), pos.y()}, {vel.x(), vel.y()}, dt, params);
	const GuidanceState state = out.state == FwControllerOutput::State::Tracking ? GuidanceState::Tracking
				    : out.state == FwControllerOutput::State::Acquiring ? GuidanceState::Acquiring
				    : out.state == FwControllerOutput::State::Finished ? GuidanceState::Finished
				    : GuidanceState::Holding;

	if (state != _guidance_state) {
		const char *text = state == GuidanceState::Tracking ? "Trajectory: on the path, tracking"
				   : state == GuidanceState::Acquiring ? "Trajectory: acquiring the path"
				   : state == GuidanceState::Finished ? "Trajectory: finished, holding course" : nullptr;

		if (text) {
			RCLCPP_INFO(_node.get_logger(), "%s (point %u)", text, out.index);
			_mavlink->sendStatusText(6 /* MAV_SEVERITY_INFO */, text);
		}
	}

	if (state == GuidanceState::Holding || state == GuidanceState::Finished) {
		holdCourse(out.reason ? out.reason : "no virtual target");
		_guidance_state = state;
		return;
	}

	_hold_reason = nullptr;
	_course_hold = std::atan2(vel.y(), vel.x());

	// Longitudinal: the internal law a_long = clamp(k_long (v_t - v), +-a_long_max) is applied to a speed
	// reference (integrated here) that TECS tracks through the airspeed setpoint. The vehicle therefore
	// approaches v_t with the same first-order, acceleration-limited response as the internal mode.
	if (!std::isfinite(_speed_ref)) {
		_speed_ref = vel.head<2>().norm();
	}

	const float ref_accel = std::min(std::max(params.guidance.k_long * (out.v_t - _speed_ref),
					 -params.guidance.a_long_max), params.guidance.a_long_max);
	_speed_ref += ref_accel * dt;
	const float ground_speed_sp = _speed_ref;

	px4_ros2::FwLateralLongitudinalSetpoint sp;
	// a_m is the flight-path y (right-positive) acceleration; FixedWingLateralSetpoint.lateral_acceleration
	// is FRD y, i.e. the same axis in coordinated flight. PX4 maps it to roll = atan(a / g).
	sp.withLateralAcceleration(out.a_m);
	sp.withEquivalentAirspeed(equivalentAirspeedFor(ground_speed_sp, _course_hold));

	if (std::isfinite(_altitude_hold_amsl)) {
		sp.withAltitude(_altitude_hold_amsl);
	}

	_fw_setpoint->update(sp);
	_guidance_state = state;
	_last_output = out;

	const rclcpp::Time now = _node.now();

	if ((now - _last_log_time).seconds() >= 1.0) {
		_last_log_time = now;
		RCLCPP_INFO(_node.get_logger(), "%s idx %u/%zu  r %.1f m  a_m %.2f  a_long %.2f  v %.1f -> %.1f m/s",
			    state == GuidanceState::Tracking ? "tracking " : "acquiring", out.index, _controller.size(), out.r,
			    out.a_m, out.a_long, vel.head<2>().norm(), ground_speed_sp);
	}
}

} // namespace trajectory_mode_fw
