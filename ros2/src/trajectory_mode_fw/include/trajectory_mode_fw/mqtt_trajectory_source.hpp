#pragma once

// MQTT link to the ground station (see ground_station/README.md), under <prefix>/<sysid>/:
//   cmd/trajectory     the trajectory JSON the web app publishes. PX4 does not export the continuous_trajectory_*
//                      uORB topics over uXRCE-DDS, so the external mode cannot read the MAVLink upload.
//   cmd/ros_param_get  {"names": [...]}           -> handled by the mode (parameter panel of the web app)
//   cmd/ros_param_set  {"name": ..., "value": ...}

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "trajectory_mode_fw/trajectory_json.hpp"

struct mosquitto;
struct mosquitto_message;

namespace trajectory_mode_fw
{

class MqttTrajectorySource
{
public:
	using LogFn = std::function<void(const std::string &)>;
	// Called on the MQTT thread with "ros_param_get" / "ros_param_set" and the JSON payload
	using CommandFn = std::function<void(const std::string &command, const std::string &payload)>;

	MqttTrajectorySource(std::string host, int port, std::string vehicle_prefix, LogFn log_info, LogFn log_error,
			     CommandFn on_command);
	~MqttTrajectorySource();

	MqttTrajectorySource(const MqttTrajectorySource &) = delete;
	MqttTrajectorySource &operator=(const MqttTrajectorySource &) = delete;

	// Latest trajectory received since the last call, if any (thread-safe)
	std::optional<Trajectory> take();

	// Publish a status message (e.g. "trajectory loaded") for the ground station; dropped if not connected.
	// qos 0 for high-rate telemetry that the next message replaces anyway.
	void publish(const std::string &topic, const std::string &payload, int qos = 1);

private:
	static void onConnect(mosquitto *mosq, void *obj, int rc);
	static void onMessage(mosquitto *mosq, void *obj, const mosquitto_message *msg);
	void connectLoop();

	std::string _host;
	int _port;
	std::string _vehicle_prefix;
	std::string _trajectory_topic;
	LogFn _log_info;
	LogFn _log_error;
	CommandFn _on_command;

	mosquitto *_mosq{nullptr};
	std::atomic<bool> _running{true};
	std::thread _connect_thread;

	std::mutex _mutex;
	std::optional<Trajectory> _pending;
};

} // namespace trajectory_mode_fw
