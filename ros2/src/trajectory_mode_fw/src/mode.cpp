#include "trajectory_mode_fw/mode.hpp"

#include <cmath>
#include <cstdlib>

#include <px4_ros2/components/events.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include "trajectory_mode_fw/yaml_params.hpp"

using namespace std::chrono_literals;

namespace trajectory_mode_fw
{

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

	node.declare_parameter("mqtt_host", "127.0.0.1"); // not "localhost": libmosquitto may try IPv6 ::1 first
	node.declare_parameter("mqtt_port", 1883);
	node.declare_parameter("topic_prefix", "uav");
	node.declare_parameter("sysid", 1);

	const std::string vehicle_prefix = node.get_parameter("topic_prefix").as_string() + "/"
					   + std::to_string(node.get_parameter("sysid").as_int());
	_status_topic = vehicle_prefix + "/ros_trajectory_status";
	_param_topic = vehicle_prefix + "/ros_param";
	_guidance_topic = vehicle_prefix + "/ros_guidance";
	_mqtt = std::make_unique<MqttTrajectorySource>(
			node.get_parameter("mqtt_host").as_string(), static_cast<int>(node.get_parameter("mqtt_port").as_int()),
			vehicle_prefix,
	[this](const std::string & s) { RCLCPP_INFO(_node.get_logger(), "%s", s.c_str()); },
	[this](const std::string & s) { RCLCPP_WARN(_node.get_logger(), "%s", s.c_str()); },
	[this](const std::string & command, const std::string & payload) { handleMqttCommand(command, payload); });

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

// Minimal extraction from the fixed JSON the web app sends
std::vector<std::string> jsonStringArray(const std::string &json, const std::string &key)
{
	std::vector<std::string> out;
	size_t pos = json.find("\"" + key + "\"");
	pos = pos == std::string::npos ? pos : json.find('[', pos);
	const size_t end = pos == std::string::npos ? pos : json.find(']', pos);

	while (pos != std::string::npos && end != std::string::npos) {
		const size_t open = json.find('"', pos + 1);

		if (open == std::string::npos || open > end) {
			break;
		}

		const size_t close = json.find('"', open + 1);

		if (close == std::string::npos || close > end) {
			break;
		}

		out.push_back(json.substr(open + 1, close - open - 1));
		pos = close;
	}

	return out;
}

std::optional<std::string> jsonString(const std::string &json, const std::string &key)
{
	size_t pos = json.find("\"" + key + "\"");
	pos = pos == std::string::npos ? pos : json.find(':', pos);
	const size_t open = pos == std::string::npos ? pos : json.find('"', pos);
	const size_t close = open == std::string::npos ? open : json.find('"', open + 1);

	if (close == std::string::npos) {
		return std::nullopt;
	}

	return json.substr(open + 1, close - open - 1);
}

std::optional<double> jsonNumber(const std::string &json, const std::string &key)
{
	const size_t key_pos = json.find("\"" + key + "\"");
	const size_t colon = key_pos == std::string::npos ? key_pos : json.find(':', key_pos);

	if (colon == std::string::npos) {
		return std::nullopt;
	}

	const char *start = json.c_str() + colon + 1;
	char *end = nullptr;
	const double value = std::strtod(start, &end);

	if (end == start || !std::isfinite(value)) {
		return std::nullopt;
	}

	return value;
}

} // namespace

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
		if (findByRosName(parameter.get_name())) {
			values.emplace_back(parameter.get_name(), yamlDouble(parameter.as_double()));

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

void TrajectoryModeFw::publishParam(const std::string &px4_name, const std::string &error)
{
	const GuidanceParamDef *def = findByPx4Name(px4_name);

	if (!def) {
		return;
	}

	std::string json = "{\"name\":\"" + px4_name + "\",\"value\":"
			   + yamlDouble(_node.get_parameter(def->ros_name).as_double()) + ",\"type\":\"float\"";

	if (!error.empty()) {
		std::string escaped;

		for (const char c : error) {
			if (c == '"' || c == '\\') {
				escaped += '\\';
			}

			if (static_cast<unsigned char>(c) >= 0x20) {
				escaped += c;
			}
		}

		json += ",\"error\":\"" + escaped + "\"";
	}

	_mqtt->publish(_param_topic, json + "}");
}

void TrajectoryModeFw::handleMqttCommand(const std::string &command, const std::string &payload)
{
	// Parameter panel of the web app for VTOLs (MQTT thread; rclcpp parameter access is thread-safe)
	if (command == "ros_param_get") {
		for (const std::string &name : jsonStringArray(payload, "names")) {
			publishParam(name);
		}

	} else if (command == "ros_param_set") {
		const std::optional<std::string> name = jsonString(payload, "name");
		const std::optional<double> value = jsonNumber(payload, "value");
		const GuidanceParamDef *def = name ? findByPx4Name(*name) : nullptr;

		if (!def || !value) {
			RCLCPP_WARN(_node.get_logger(), "Rejected ros_param_set: %s", payload.c_str());
			return;
		}

		const rcl_interfaces::msg::SetParametersResult result = _node.set_parameter(rclcpp::Parameter(def->ros_name,
				*value));
		publishParam(*name, result.successful ? "" : result.reason);
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
	_manager.clear();
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
	_manager.clear();
	publishGuidance("inactive", nullptr, nullptr, true);
}

void TrajectoryModeFw::publishGuidance(const char *state, const ManagerOutput *target, const GuidanceOutput *guidance,
				       bool force)
{
	const rclcpp::Time now = _node.now();

	if (!force && (now - _last_guidance_pub).seconds() < 0.2) {
		return;
	}

	_last_guidance_pub = now;

	const auto number = [](float v) { return std::isfinite(v) ? yamlDouble(v) : std::string("null"); };
	std::string json = std::string("{\"state\":\"") + state + "\",\"points\":" + std::to_string(_manager.size());

	if (target) {
		json += ",\"r\":" + number(target->r) + ",\"index\":" + std::to_string(target->index)
			+ ",\"v_t\":" + number(target->v_t) + ",\"v_m\":" + number(target->v_m);
	}

	if (guidance) {
		json += ",\"a_m\":" + number(guidance->a_m) + ",\"a_long\":" + number(guidance->a_long);
	}

	_mqtt->publish(_guidance_topic, json + "}", 0);
}

GuidanceParams TrajectoryModeFw::readParams()
{
	GuidanceParams p;
	p.alpha = static_cast<float>(_node.get_parameter("traj_smc_alpha").as_double());
	p.beta = static_cast<float>(_node.get_parameter("traj_smc_beta").as_double());
	p.epsilon = static_cast<float>(_node.get_parameter("traj_smc_eps").as_double());
	p.r_star = static_cast<float>(_node.get_parameter("traj_r_star").as_double());
	p.k_long = static_cast<float>(_node.get_parameter("traj_k_long").as_double());
	p.a_m_max = static_cast<float>(_node.get_parameter("traj_a_m_max").as_double());
	p.a_long_max = static_cast<float>(_node.get_parameter("traj_a_long_max").as_double());
	return p;
}

void TrajectoryModeFw::loadPendingTrajectory()
{
	std::optional<Trajectory> trajectory = _mqtt->take();

	if (_pending_ros) {
		trajectory = std::move(_pending_ros);
		_pending_ros.reset();
	}

	if (trajectory) {
		_manager.load(*trajectory);
		_hold_reason = nullptr;
		RCLCPP_INFO(_node.get_logger(), "Trajectory %u loaded (%zu points)", trajectory->id, trajectory->points.size());
		_mqtt->publish(_status_topic, "{\"id\":" + std::to_string(trajectory->id) + ",\"points\":"
			       + std::to_string(trajectory->points.size()) + ",\"state\":\"loaded\"}");
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

	publishGuidance("holding", nullptr, nullptr);

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

	if (!_manager.active()) {
		holdCourse("waiting for a trajectory");
		return;
	}

	const GuidanceParams params = readParams();
	const int window = static_cast<int>(_node.get_parameter("traj_search_window").as_int());
	const Eigen::Vector3f pos = _local_position->positionNed();
	const Eigen::Vector3f vel = _local_position->velocityNed();

	// TrajectoryManager -> continuous_trajectory_output -> FlightTaskTraj guidance
	const ManagerOutput target = _manager.update({pos.x(), pos.y()}, {vel.x(), vel.y()}, params.r_star, window);
	const GuidanceOutput guidance = computeGuidance(target, params);

	if (!std::isfinite(target.r) || !guidance.finite) {
		// The internal mode would publish a non-finite setpoint here (e.g. past the end of the trajectory)
		holdCourse("no virtual target in range (end of the trajectory?)");
		return;
	}

	_hold_reason = nullptr;
	_course_hold = target.gamma_m;

	// Longitudinal: the internal law a_long = clamp(k_long (v_t - v), +-a_long_max) is applied to a speed
	// reference (integrated here) that TECS tracks through the airspeed setpoint. The vehicle therefore
	// approaches v_t with the same first-order, acceleration-limited response as the internal mode.
	if (!std::isfinite(_speed_ref)) {
		_speed_ref = target.v_m;
	}

	const float dt = (dt_s > 0.f && dt_s < 0.5f) ? dt_s : 0.f;
	const float ref_accel = std::min(std::max(params.k_long * (target.v_t - _speed_ref), -params.a_long_max),
					 params.a_long_max);
	_speed_ref += ref_accel * dt;
	const float ground_speed_sp = _speed_ref;

	px4_ros2::FwLateralLongitudinalSetpoint sp;
	// a_m is the flight-path y (right-positive) acceleration; FixedWingLateralSetpoint.lateral_acceleration
	// is FRD y, i.e. the same axis in coordinated flight. PX4 maps it to roll = atan(a / g).
	sp.withLateralAcceleration(guidance.a_m);
	sp.withEquivalentAirspeed(equivalentAirspeedFor(ground_speed_sp, target.gamma_m));

	if (std::isfinite(_altitude_hold_amsl)) {
		sp.withAltitude(_altitude_hold_amsl);
	}

	_fw_setpoint->update(sp);
	publishGuidance("tracking", &target, &guidance);

	const rclcpp::Time now = _node.now();

	if ((now - _last_log_time).seconds() >= 1.0) {
		_last_log_time = now;
		RCLCPP_INFO(_node.get_logger(), "idx %u/%zu  r %.1f m  a_m %.2f  a_long %.2f  v %.1f -> %.1f m/s",
			    target.index, _manager.size(), target.r, guidance.a_m, guidance.a_long, target.v_m, ground_speed_sp);
	}
}

} // namespace trajectory_mode_fw
