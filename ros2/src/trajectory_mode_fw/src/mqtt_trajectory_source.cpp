#include "trajectory_mode_fw/mqtt_trajectory_source.hpp"

#include <chrono>

#include <mosquitto.h>

namespace trajectory_mode_fw
{

MqttTrajectorySource::MqttTrajectorySource(std::string host, int port, std::string vehicle_prefix, LogFn log_info,
		LogFn log_error, CommandFn on_command)
	: _host(std::move(host)), _port(port), _vehicle_prefix(std::move(vehicle_prefix)),
	  _trajectory_topic(_vehicle_prefix + "/cmd/trajectory"), _log_info(std::move(log_info)),
	  _log_error(std::move(log_error)), _on_command(std::move(on_command))
{
	mosquitto_lib_init();
	_mosq = mosquitto_new(nullptr, true, this);

	if (!_mosq) {
		_log_error("mosquitto_new failed; trajectories can only arrive on the ROS topic");
		return;
	}

	mosquitto_connect_callback_set(_mosq, &MqttTrajectorySource::onConnect);
	mosquitto_message_callback_set(_mosq, &MqttTrajectorySource::onMessage);
	mosquitto_reconnect_delay_set(_mosq, 1, 10, true);

	// connect_async fails immediately if the broker is not up yet; keep retrying until it is,
	// after that the mosquitto loop thread reconnects on its own
	_connect_thread = std::thread(&MqttTrajectorySource::connectLoop, this);
}

MqttTrajectorySource::~MqttTrajectorySource()
{
	_running = false;

	if (_connect_thread.joinable()) {
		_connect_thread.join();
	}

	if (_mosq) {
		mosquitto_disconnect(_mosq);
		mosquitto_loop_stop(_mosq, true);
		mosquitto_destroy(_mosq);
	}

	mosquitto_lib_cleanup();
}

void MqttTrajectorySource::connectLoop()
{
	bool reported = false;

	while (_running) {
		if (mosquitto_connect_async(_mosq, _host.c_str(), _port, 10) == MOSQ_ERR_SUCCESS
		    && mosquitto_loop_start(_mosq) == MOSQ_ERR_SUCCESS) {
			return;
		}

		if (!reported) {
			_log_error("MQTT broker " + _host + ":" + std::to_string(_port) + " not reachable, retrying...");
			reported = true;
		}

		for (int i = 0; i < 20 && _running; ++i) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}

void MqttTrajectorySource::onConnect(mosquitto *mosq, void *obj, int rc)
{
	auto *self = static_cast<MqttTrajectorySource *>(obj);

	if (rc != 0) {
		self->_log_error(std::string("MQTT connection refused: ") + mosquitto_connack_string(rc));
		return;
	}

	mosquitto_subscribe(mosq, nullptr, self->_trajectory_topic.c_str(), 1);
	mosquitto_subscribe(mosq, nullptr, (self->_vehicle_prefix + "/cmd/ros_param_get").c_str(), 1);
	mosquitto_subscribe(mosq, nullptr, (self->_vehicle_prefix + "/cmd/ros_param_set").c_str(), 1);
	self->_log_info("MQTT connected, waiting for trajectories on " + self->_trajectory_topic);
}

void MqttTrajectorySource::onMessage(mosquitto *, void *obj, const mosquitto_message *msg)
{
	auto *self = static_cast<MqttTrajectorySource *>(obj);
	const std::string topic = msg->topic;
	const std::string command_prefix = self->_vehicle_prefix + "/cmd/";

	if (topic != self->_trajectory_topic) {
		if (self->_on_command && topic.rfind(command_prefix, 0) == 0) {
			self->_on_command(topic.substr(command_prefix.size()),
					  std::string(static_cast<const char *>(msg->payload), msg->payloadlen));
		}

		return;
	}

	Trajectory trajectory;
	std::string error;

	if (!parseTrajectoryJson(std::string(static_cast<const char *>(msg->payload), msg->payloadlen), trajectory,
				 error)) {
		self->_log_error("Rejected trajectory from MQTT: " + error);
		return;
	}

	std::lock_guard<std::mutex> lock(self->_mutex);
	self->_pending = std::move(trajectory);
}

void MqttTrajectorySource::publish(const std::string &topic, const std::string &payload, int qos)
{
	if (_mosq) {
		mosquitto_publish(_mosq, nullptr, topic.c_str(), static_cast<int>(payload.size()), payload.data(), qos, false);
	}
}

std::optional<Trajectory> MqttTrajectorySource::take()
{
	std::lock_guard<std::mutex> lock(_mutex);
	std::optional<Trajectory> out = std::move(_pending);
	_pending.reset();
	return out;
}

} // namespace trajectory_mode_fw
