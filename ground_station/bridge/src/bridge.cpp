#include "bridge.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <mosquitto.h>

static constexpr const char *TOPIC_RAW_RX = "mavlink/rx";
static constexpr const char *TOPIC_RAW_TX = "mavlink/tx";
static constexpr const char *TOPIC_STATUS = "bridge/status";

// Separate parser channels: UDP is parsed on the main thread, MQTT tx on the mosquitto thread.
static constexpr mavlink_channel_t CHAN_UDP = MAVLINK_COMM_0;
static constexpr mavlink_channel_t CHAN_MQTT_TX = MAVLINK_COMM_1;

// Packing for trajectory uploads happens on the MQTT thread
static constexpr mavlink_channel_t CHAN_UPLOAD = MAVLINK_COMM_2;

// These commands are only in the "development" dialect of the bundled headers
static constexpr uint16_t CMD_DO_SET_MODE = 176;
static constexpr uint16_t CMD_SET_MESSAGE_INTERVAL = 511;

// Integer value of "key" in a flat JSON object, e.g. jsonInt(R"({"a": 4})", "a", v)
static bool jsonInt(const std::string &json, const char *key, long &value)
{
	const std::string quoted = std::string("\"") + key + "\"";
	size_t pos = json.find(quoted);

	if (pos == std::string::npos) {
		return false;
	}

	pos = json.find(':', pos + quoted.size());

	if (pos == std::string::npos) {
		return false;
	}

	const char *start = json.c_str() + pos + 1;
	char *end = nullptr;
	value = std::strtol(start, &end, 10);
	return end != start;
}

MavlinkMqttBridge::MavlinkMqttBridge(const BridgeConfig &config) : _config(config)
{
	for (auto &compid : _autopilot_compid) {
		compid = MAV_COMP_ID_AUTOPILOT1;
	}
}

MavlinkMqttBridge::~MavlinkMqttBridge()
{
	if (_mosq) {
		if (_mqtt_loop_started) {
			publish(TOPIC_STATUS, std::string(R"({"online":false})"), true);
			mosquitto_disconnect(_mosq);
			mosquitto_loop_stop(_mosq, false);
		}

		mosquitto_destroy(_mosq);
	}

	mosquitto_lib_cleanup();

	if (_sock >= 0) {
		close(_sock);
	}
}

bool MavlinkMqttBridge::init()
{
	// --- UDP ---
	_sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (_sock < 0) {
		perror("socket");
		return false;
	}

	int reuse = 1;
	setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	local.sin_port = htons(_config.udp_listen_port);

	if (bind(_sock, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
		perror("bind");
		return false;
	}

	_target_addr.sin_family = AF_INET;
	_target_addr.sin_port = htons(_config.udp_target_port);

	if (inet_pton(AF_INET, _config.udp_target_host.c_str(), &_target_addr.sin_addr) != 1) {
		std::cerr << "Invalid UDP target host: " << _config.udp_target_host << std::endl;
		return false;
	}

	// --- MQTT ---
	mosquitto_lib_init();
	_mosq = mosquitto_new("mavlink_mqtt_bridge", true, this);

	if (!_mosq) {
		std::cerr << "mosquitto_new failed" << std::endl;
		return false;
	}

	mosquitto_connect_callback_set(_mosq, &MavlinkMqttBridge::onConnect);
	mosquitto_message_callback_set(_mosq, &MavlinkMqttBridge::onMessage);
	mosquitto_reconnect_delay_set(_mosq, 1, 10, true);

	const std::string will = R"({"online":false})";
	mosquitto_will_set(_mosq, TOPIC_STATUS, will.size(), will.data(), 1, true);

	if (!tryConnectMqtt()) {
		std::cerr << "MQTT broker " << _config.mqtt_host << ":" << _config.mqtt_port
			  << " not reachable, retrying..." << std::endl;
	}

	std::cout << "MAVLink UDP: listening on " << _config.udp_listen_port
		  << ", default target " << _config.udp_target_host << ":" << _config.udp_target_port << std::endl
		  << "MQTT broker: " << _config.mqtt_host << ":" << _config.mqtt_port << std::endl;

	return true;
}

bool MavlinkMqttBridge::tryConnectMqtt()
{
	// The loop thread handles reconnects after a successful first connect, but it is only started
	// once connect_async succeeds (it fails immediately if the broker is not up yet).
	if (mosquitto_connect_async(_mosq, _config.mqtt_host.c_str(), _config.mqtt_port, 10) != MOSQ_ERR_SUCCESS) {
		return false;
	}

	const int rc = mosquitto_loop_start(_mosq);

	if (rc != MOSQ_ERR_SUCCESS) {
		std::cerr << "mosquitto_loop_start failed: " << mosquitto_strerror(rc) << std::endl;
		return false;
	}

	_mqtt_loop_started = true;
	return true;
}

void MavlinkMqttBridge::run()
{
	using clock = std::chrono::steady_clock;

	_running = true;
	auto next_heartbeat = clock::now();
	auto next_mqtt_retry = clock::now() + std::chrono::seconds(2);
	auto next_upload = clock::now();
	uint8_t buf[2048];

	while (_running) {
		const auto now = clock::now();

		if (!_mqtt_loop_started && now >= next_mqtt_retry) {
			tryConnectMqtt();
			next_mqtt_retry = now + std::chrono::seconds(2);
		}

		if (now >= next_heartbeat) {
			sendHeartbeat();
			next_heartbeat = now + std::chrono::seconds(1);
		}

		bool uploading = false;
		{
			std::lock_guard<std::mutex> lock(_upload_mutex);
			uploading = !_upload.queue.empty();
		}

		if (uploading && now >= next_upload) {
			serviceUploadQueue();
			next_upload = now + std::chrono::milliseconds(_config.upload_interval_ms);
		}

		pollfd pfd{_sock, POLLIN, 0};
		const int ret = poll(&pfd, 1, uploading ? std::max(1, _config.upload_interval_ms) : 100);

		if (ret > 0 && (pfd.revents & POLLIN)) {
			sockaddr_in from{};
			socklen_t from_len = sizeof(from);
			const ssize_t n = recvfrom(_sock, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &from_len);

			if (n > 0) {
				handleUdpBytes(buf, static_cast<size_t>(n), from);
			}
		}
	}
}

void MavlinkMqttBridge::onConnect(mosquitto *mosq, void *obj, int rc)
{
	auto *self = static_cast<MavlinkMqttBridge *>(obj);

	if (rc != 0) {
		std::cerr << "MQTT connection refused: " << mosquitto_connack_string(rc) << std::endl;
		return;
	}

	std::cout << "MQTT connected" << std::endl;
	mosquitto_subscribe(mosq, nullptr, TOPIC_RAW_TX, 0);
	mosquitto_subscribe(mosq, nullptr, (self->_config.topic_prefix + "/+/cmd/trajectory").c_str(), 1);
	mosquitto_subscribe(mosq, nullptr, (self->_config.topic_prefix + "/+/cmd/set_mode").c_str(), 1);
	self->publish(TOPIC_STATUS, std::string(R"({"online":true})"), true);
}

void MavlinkMqttBridge::onMessage(mosquitto *, void *obj, const mosquitto_message *msg)
{
	auto *self = static_cast<MavlinkMqttBridge *>(obj);

	if (strcmp(msg->topic, TOPIC_RAW_TX) == 0 && msg->payloadlen > 0) {
		self->handleMqttTx(static_cast<const uint8_t *>(msg->payload), static_cast<size_t>(msg->payloadlen));
		return;
	}

	// <prefix>/<sysid>/cmd/...
	const std::string &prefix = self->_config.topic_prefix;
	const uint8_t sysid = static_cast<uint8_t>(std::atoi(msg->topic + prefix.size() + 1));
	const std::string payload(static_cast<const char *>(msg->payload), msg->payloadlen);
	bool match = false;

	mosquitto_topic_matches_sub((prefix + "/+/cmd/trajectory").c_str(), msg->topic, &match);

	if (match) {
		self->handleTrajectoryCommand(sysid, payload);
		return;
	}

	mosquitto_topic_matches_sub((prefix + "/+/cmd/set_mode").c_str(), msg->topic, &match);

	if (match) {
		self->handleSetModeCommand(sysid, payload);
	}
}

void MavlinkMqttBridge::handleSetModeCommand(uint8_t sysid, const std::string &payload)
{
	long main_mode = 0;
	long sub_mode = 0;

	if (!jsonInt(payload, "main_mode", main_mode) || main_mode < 0 || main_mode > 255) {
		std::cerr << "Rejected set_mode for system " << int(sysid) << ": bad main_mode" << std::endl;
		return;
	}

	jsonInt(payload, "sub_mode", sub_mode); // optional, 0 if absent

	// Same as PX4's convention and pymavlink's set_mode: param1 = base mode flags,
	// param2 = custom main mode, param3 = custom sub mode
	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
					   sysid, _autopilot_compid[sysid], CMD_DO_SET_MODE, 0,
					   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, float(main_mode), float(sub_mode),
					   0, 0, 0, 0);
	sendMavlink(msg);

	std::cout << "Sent DO_SET_MODE main=" << main_mode << " sub=" << sub_mode << " to system "
		  << int(sysid) << std::endl;
}

void MavlinkMqttBridge::handleTrajectoryCommand(uint8_t sysid, const std::string &payload)
{
	Trajectory trajectory;
	std::string error;

	if (!parseTrajectoryJson(payload, trajectory, error)) {
		std::cerr << "Rejected trajectory for system " << int(sysid) << ": " << error << std::endl;
		publishUploadStatus(sysid, trajectory.id, "error", 0, 0, error);
		return;
	}

	std::deque<mavlink_message_t> queue;
	mavlink_message_t msg;

	mavlink_msg_trajectory_setpoint_initiate_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
			static_cast<uint16_t>(trajectory.points.size()), trajectory.id);
	queue.push_back(msg);

	for (size_t i = 0; i < trajectory.points.size(); ++i) {
		const TrajectoryPoint &p = trajectory.points[i];
		mavlink_msg_trajectory_setpoint_upload_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
				static_cast<uint16_t>(i), trajectory.id, p.x, p.y, p.vx, p.vy, p.at, p.jt, p.t);
		queue.push_back(msg);
	}

	{
		std::lock_guard<std::mutex> lock(_upload_mutex);

		if (!_upload.queue.empty()) {
			std::cout << "Aborting unfinished upload of trajectory " << int(_upload.id) << std::endl;
		}

		_upload.sysid = sysid;
		_upload.id = trajectory.id;
		_upload.total = trajectory.points.size();
		_upload.sent = 0;
		_upload.queue = std::move(queue);
	}

	std::cout << "Uploading trajectory " << int(trajectory.id) << " (" << trajectory.points.size()
		  << " points) to system " << int(sysid) << std::endl;
	publishUploadStatus(sysid, trajectory.id, "uploading", 0, trajectory.points.size());
}

void MavlinkMqttBridge::serviceUploadQueue()
{
	mavlink_message_t msg;
	uint8_t sysid, id;
	size_t sent, total;
	bool done;

	{
		std::lock_guard<std::mutex> lock(_upload_mutex);

		if (_upload.queue.empty()) {
			return;
		}

		msg = _upload.queue.front();
		_upload.queue.pop_front();

		if (msg.msgid == MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD) {
			++_upload.sent;
		}

		sysid = _upload.sysid;
		id = _upload.id;
		sent = _upload.sent;
		total = _upload.total;
		done = _upload.queue.empty();
	}

	sendMavlink(msg);

	if (done) {
		std::cout << "Trajectory " << int(id) << " uploaded (" << total << " points)" << std::endl;
		publishUploadStatus(sysid, id, "done", sent, total);

	} else if (sent % 100 == 0 && msg.msgid == MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD) {
		publishUploadStatus(sysid, id, "uploading", sent, total);
	}
}

void MavlinkMqttBridge::publishUploadStatus(uint8_t sysid, uint8_t id, const char *state, size_t sent,
		size_t total, const std::string &error)
{
	std::string json = "{\"id\":" + std::to_string(id) + ",\"state\":\"" + state + "\",\"sent\":"
			   + std::to_string(sent) + ",\"total\":" + std::to_string(total);

	if (!error.empty()) {
		std::string escaped;

		for (const char c : error) {
			if (c == '"' || c == '\\') {
				escaped += '\\';
			}

			escaped += c;
		}

		json += ",\"error\":\"" + escaped + "\"";
	}

	json += "}";
	publish(vehicleTopic(sysid, "trajectory_status"), json);
}

void MavlinkMqttBridge::handleUdpBytes(const uint8_t *data, size_t len, const sockaddr_in &from)
{
	// Reply to wherever the vehicle is actually sending from
	{
		std::lock_guard<std::mutex> lock(_target_mutex);

		if (!_target_learned || _target_addr.sin_addr.s_addr != from.sin_addr.s_addr
		    || _target_addr.sin_port != from.sin_port) {
			char ip[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
			std::cout << "Vehicle link at " << ip << ":" << ntohs(from.sin_port) << std::endl;
			_target_addr = from;
			_target_learned = true;
		}
	}

	mavlink_message_t msg;
	mavlink_status_t status;

	for (size_t i = 0; i < len; ++i) {
		if (mavlink_parse_char(CHAN_UDP, data[i], &msg, &status)) {
			handleMavlinkMessage(msg);
		}
	}
}

void MavlinkMqttBridge::handleMavlinkMessage(const mavlink_message_t &msg)
{
	char json[512];

	switch (msg.msgid) {
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
			mavlink_local_position_ned_t p;
			mavlink_msg_local_position_ned_decode(&msg, &p);
			snprintf(json, sizeof(json),
				 R"({"time_boot_ms":%u,"x":%.4f,"y":%.4f,"z":%.4f,"vx":%.4f,"vy":%.4f,"vz":%.4f})",
				 p.time_boot_ms, p.x, p.y, p.z, p.vx, p.vy, p.vz);
			publish(vehicleTopic(msg.sysid, "local_position_ned"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_ATTITUDE: {
			mavlink_attitude_t a;
			mavlink_msg_attitude_decode(&msg, &a);
			snprintf(json, sizeof(json),
				 R"({"time_boot_ms":%u,"roll":%.5f,"pitch":%.5f,"yaw":%.5f,"rollspeed":%.5f,"pitchspeed":%.5f,"yawspeed":%.5f})",
				 a.time_boot_ms, a.roll, a.pitch, a.yaw, a.rollspeed, a.pitchspeed, a.yawspeed);
			publish(vehicleTopic(msg.sysid, "attitude"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_HEARTBEAT: {
			mavlink_heartbeat_t hb;
			mavlink_msg_heartbeat_decode(&msg, &hb);

			// Only forward the autopilot's heartbeat (ignore cameras, GCSs, ourselves, ...)
			if (hb.autopilot == MAV_AUTOPILOT_INVALID || msg.sysid == _config.sysid) {
				break;
			}

			_autopilot_compid[msg.sysid] = msg.compid;

			const bool armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
			snprintf(json, sizeof(json),
				 R"({"type":%u,"autopilot":%u,"base_mode":%u,"custom_mode":%u,"system_status":%u,"armed":%s})",
				 hb.type, hb.autopilot, hb.base_mode, hb.custom_mode, hb.system_status, armed ? "true" : "false");
			publish(vehicleTopic(msg.sysid, "heartbeat"), std::string(json));

			if (_config.stream_rate_hz > 0.f && _streams_requested.insert(msg.sysid).second) {
				requestStreams(msg.sysid, msg.compid);
			}

			break;
		}

	case MAVLINK_MSG_ID_COMMAND_ACK: {
			mavlink_command_ack_t ack;
			mavlink_msg_command_ack_decode(&msg, &ack);

			// Only acks addressed to this bridge (or broadcast)
			if (ack.target_system != 0 && ack.target_system != _config.sysid) {
				break;
			}

			snprintf(json, sizeof(json), R"({"command":%u,"result":%u})", ack.command, ack.result);
			publish(vehicleTopic(msg.sysid, "command_ack"), std::string(json));
			break;
		}

	default:
		break;
	}

	if (_config.publish_raw) {
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		publish(TOPIC_RAW_RX, buf, len);
	}
}

void MavlinkMqttBridge::handleMqttTx(const uint8_t *data, size_t len)
{
	// Validate framing/CRC before forwarding so garbage never reaches the vehicle
	mavlink_message_t msg;
	mavlink_status_t status;

	for (size_t i = 0; i < len; ++i) {
		if (mavlink_parse_char(CHAN_MQTT_TX, data[i], &msg, &status)) {
			sendMavlink(msg);
		}
	}
}

void MavlinkMqttBridge::sendMavlink(const mavlink_message_t &msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	sendUdp(buf, len);
}

void MavlinkMqttBridge::sendUdp(const uint8_t *data, size_t len)
{
	std::lock_guard<std::mutex> lock(_target_mutex);
	sendto(_sock, data, len, 0, reinterpret_cast<const sockaddr *>(&_target_addr), sizeof(_target_addr));
}

void MavlinkMqttBridge::sendHeartbeat()
{
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack_chan(_config.sysid, _config.compid, CHAN_UDP, &msg,
					MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
	sendMavlink(msg);
}

void MavlinkMqttBridge::requestStreams(uint8_t target_sys, uint8_t target_comp)
{
	const float interval_us = 1e6f / _config.stream_rate_hz;

	for (const uint32_t msg_id : {MAVLINK_MSG_ID_LOCAL_POSITION_NED, MAVLINK_MSG_ID_ATTITUDE}) {
		mavlink_message_t msg;
		mavlink_msg_command_long_pack_chan(_config.sysid, _config.compid, CHAN_UDP, &msg,
						   target_sys, target_comp, CMD_SET_MESSAGE_INTERVAL, 0,
						   static_cast<float>(msg_id), interval_us, 0, 0, 0, 0, 0);
		sendMavlink(msg);
	}

	std::cout << "Requested LOCAL_POSITION_NED/ATTITUDE at " << _config.stream_rate_hz
		  << " Hz from system " << int(target_sys) << std::endl;
}

void MavlinkMqttBridge::publish(const std::string &topic, const std::string &payload, bool retain)
{
	publish(topic, payload.data(), payload.size(), retain);
}

void MavlinkMqttBridge::publish(const std::string &topic, const void *data, size_t len, bool retain)
{
	mosquitto_publish(_mosq, nullptr, topic.c_str(), static_cast<int>(len), data, 0, retain);
}

std::string MavlinkMqttBridge::vehicleTopic(uint8_t sysid, const char *name) const
{
	return _config.topic_prefix + "/" + std::to_string(sysid) + "/" + name;
}
