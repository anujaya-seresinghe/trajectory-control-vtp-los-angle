#include "bridge.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <climits>
#include <cmath>
#include <cstring>
#include <vector>
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
	const long parsed = std::strtol(start, &end, 10);

	if (end == start) {
		return false; // e.g. null: leave the caller's default (such as INT32_MAX = "not set") in place
	}

	value = parsed;
	return true;
}

// Position just after `"key":` in a flat JSON object, or npos
static size_t jsonValuePos(const std::string &json, const char *key)
{
	const std::string quoted = std::string("\"") + key + "\"";
	size_t pos = json.find(quoted);

	if (pos == std::string::npos) {
		return pos;
	}

	pos = json.find(':', pos + quoted.size());
	return pos == std::string::npos ? pos : pos + 1;
}

static bool jsonNumber(const std::string &json, const char *key, double &value)
{
	const size_t pos = jsonValuePos(json, key);

	if (pos == std::string::npos) {
		return false;
	}

	const char *start = json.c_str() + pos;
	char *end = nullptr;
	const double parsed = std::strtod(start, &end);

	// Leave `value` untouched unless a finite number was actually parsed: `"z": null` must stay NaN
	// (e.g. DO_REPOSITION altitude "keep current"), not become 0 m AMSL.
	if (end == start || !std::isfinite(parsed)) {
		return false;
	}

	value = parsed;
	return true;
}

// Quoted strings in order, starting at `from` and stopping at `stop` (e.g. ']' or ',')
static std::vector<std::string> jsonStrings(const std::string &json, size_t from, char stop, size_t max_count)
{
	std::vector<std::string> out;

	for (size_t i = from; i < json.size() && out.size() < max_count; ++i) {
		if (json[i] == stop) {
			break;
		}

		if (json[i] == '"') {
			const size_t close = json.find('"', i + 1);

			if (close == std::string::npos) {
				break;
			}

			out.push_back(json.substr(i + 1, close - i - 1));
			i = close;
		}
	}

	return out;
}

// Up to max_count numbers of a JSON array starting at `open` ('['); null (or anything non-numeric) -> NaN
static std::vector<float> jsonFloatArray(const std::string &json, size_t open, size_t max_count)
{
	std::vector<float> out;
	const size_t close = json.find(']', open);

	if (open == std::string::npos || close == std::string::npos) {
		return out;
	}

	size_t start = open + 1;

	while (start < close && out.size() < max_count) {
		size_t end = json.find(',', start);

		if (end == std::string::npos || end > close) {
			end = close;
		}

		const std::string item = json.substr(start, end - start);
		char *num_end = nullptr;
		const double v = std::strtod(item.c_str(), &num_end);
		out.push_back(num_end != item.c_str() && std::isfinite(v) ? static_cast<float>(v) : NAN);
		start = end + 1;
	}

	return out;
}

// PX4 parameter names: 1-16 chars of A-Z, 0-9, _
static bool validParamName(const std::string &name)
{
	if (name.empty() || name.size() > 16) {
		return false;
	}

	for (const char c : name) {
		if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) {
			return false;
		}
	}

	return true;
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
	mosquitto_subscribe(mosq, nullptr, (self->_config.topic_prefix + "/+/cmd/command_long").c_str(), 1);
	mosquitto_subscribe(mosq, nullptr, (self->_config.topic_prefix + "/+/cmd/command_int").c_str(), 1);
	mosquitto_subscribe(mosq, nullptr, (self->_config.topic_prefix + "/+/cmd/param_get").c_str(), 1);
	mosquitto_subscribe(mosq, nullptr, (self->_config.topic_prefix + "/+/cmd/param_set").c_str(), 1);
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
		return;
	}

	mosquitto_topic_matches_sub((prefix + "/+/cmd/command_long").c_str(), msg->topic, &match);

	if (match) {
		self->handleCommandLong(sysid, payload);
		return;
	}

	mosquitto_topic_matches_sub((prefix + "/+/cmd/command_int").c_str(), msg->topic, &match);

	if (match) {
		self->handleCommandInt(sysid, payload);
		return;
	}

	mosquitto_topic_matches_sub((prefix + "/+/cmd/param_get").c_str(), msg->topic, &match);

	if (match) {
		self->handleParamGetCommand(sysid, payload);
		return;
	}

	mosquitto_topic_matches_sub((prefix + "/+/cmd/param_set").c_str(), msg->topic, &match);

	if (match) {
		self->handleParamSetCommand(sysid, payload);
	}
}

void MavlinkMqttBridge::handleCommandLong(uint8_t sysid, const std::string &payload)
{
	long command = 0;
	const size_t pos = jsonValuePos(payload, "params");
	std::vector<float> p = pos == std::string::npos ? std::vector<float> {}
			       : jsonFloatArray(payload, payload.find('[', pos), 7);

	if (!jsonInt(payload, "command", command) || command < 0 || command > 65535) {
		std::cerr << "Rejected command_long for system " << int(sysid) << ": bad \"command\"" << std::endl;
		return;
	}

	p.resize(7, NAN); // missing params are NaN ("not set" for most PX4 commands)
	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
					   sysid, _autopilot_compid[sysid], static_cast<uint16_t>(command), 0,
					   p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
	sendMavlink(msg);

	std::cout << "Sent COMMAND_LONG " << command << " to system " << int(sysid) << std::endl;
}

void MavlinkMqttBridge::handleCommandInt(uint8_t sysid, const std::string &payload)
{
	long command = 0;
	long frame = 0;
	const size_t pos = jsonValuePos(payload, "params");
	std::vector<float> p = pos == std::string::npos ? std::vector<float> {}
			       : jsonFloatArray(payload, payload.find('[', pos), 4);

	if (!jsonInt(payload, "command", command) || command < 0 || command > 65535
	    || !jsonInt(payload, "frame", frame) || frame < 0 || frame > 255) {
		std::cerr << "Rejected command_int for system " << int(sysid) << ": bad \"command\" or \"frame\"" << std::endl;
		return;
	}

	p.resize(4, NAN);
	// x/y: integers (degE7 in global frames); missing -> INT32_MAX, which PX4 treats as "not set"
	long x = INT32_MAX;
	long y = INT32_MAX;
	double z = NAN;
	jsonInt(payload, "x", x);
	jsonInt(payload, "y", y);
	jsonNumber(payload, "z", z);

	mavlink_message_t msg;
	mavlink_msg_command_int_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
					  sysid, _autopilot_compid[sysid], static_cast<uint8_t>(frame),
					  static_cast<uint16_t>(command), 0, 0, p[0], p[1], p[2], p[3],
					  static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<float>(z));
	sendMavlink(msg);

	std::cout << "Sent COMMAND_INT " << command << " (x=" << x << " y=" << y << ") to system " << int(sysid) << std::endl;
}

void MavlinkMqttBridge::handleParamGetCommand(uint8_t sysid, const std::string &payload)
{
	const size_t pos = jsonValuePos(payload, "names");
	const size_t open = pos == std::string::npos ? pos : payload.find('[', pos);

	if (open == std::string::npos) {
		std::cerr << "Rejected param_get for system " << int(sysid) << ": expected {\"names\": [...]}" << std::endl;
		return;
	}

	for (const std::string &name : jsonStrings(payload, open + 1, ']', 64)) {
		if (!validParamName(name)) {
			std::cerr << "Rejected param_get: bad parameter name \"" << name << "\"" << std::endl;
			continue;
		}

		char id[17] {};
		strncpy(id, name.c_str(), 16);
		mavlink_message_t msg;
		// param_index -1: look the parameter up by name
		mavlink_msg_param_request_read_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
				sysid, _autopilot_compid[sysid], id, -1);
		sendMavlink(msg);
	}
}

void MavlinkMqttBridge::handleParamSetCommand(uint8_t sysid, const std::string &payload)
{
	const size_t pos = jsonValuePos(payload, "name");
	const std::vector<std::string> names = pos == std::string::npos ? std::vector<std::string> {}
					       : jsonStrings(payload, pos, ',', 1);
	double value = 0.0;

	if (names.empty() || !validParamName(names[0]) || !jsonNumber(payload, "value", value)) {
		std::cerr << "Rejected param_set for system " << int(sysid) << ": expected {\"name\": \"...\", \"value\": n}"
			  << std::endl;
		return;
	}

	char id[17] {};
	strncpy(id, names[0].c_str(), 16);
	mavlink_message_t msg;
	// Float parameters only; PX4 answers with PARAM_VALUE carrying the value it actually stored
	mavlink_msg_param_set_pack_chan(_config.sysid, _config.compid, CHAN_UPLOAD, &msg,
					sysid, _autopilot_compid[sysid], id, static_cast<float>(value), MAV_PARAM_TYPE_REAL32);
	sendMavlink(msg);

	std::cout << "Sent PARAM_SET " << id << " = " << value << " to system " << int(sysid) << std::endl;
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

	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
			mavlink_global_position_int_t g;
			mavlink_msg_global_position_int_decode(&msg, &g);
			snprintf(json, sizeof(json),
				 R"({"time_boot_ms":%u,"lat":%.7f,"lon":%.7f,"alt":%.3f,"relative_alt":%.3f})",
				 g.time_boot_ms, g.lat * 1e-7, g.lon * 1e-7, g.alt * 1e-3, g.relative_alt * 1e-3);
			publish(vehicleTopic(msg.sysid, "global_position"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
			mavlink_extended_sys_state_t es;
			mavlink_msg_extended_sys_state_decode(&msg, &es);
			snprintf(json, sizeof(json), R"({"vtol_state":%u,"landed_state":%u})", es.vtol_state, es.landed_state);
			publish(vehicleTopic(msg.sysid, "extended_sys_state"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_AVAILABLE_MODES: {
			mavlink_available_modes_t am;
			mavlink_msg_available_modes_decode(&msg, &am);
			char name[36] {};
			memcpy(name, am.mode_name, 35);
			std::string escaped;

			for (const char *c = name; *c; ++c) {
				if (*c == '"' || *c == '\\') {
					escaped += '\\';
				}

				if (static_cast<unsigned char>(*c) >= 0x20) {
					escaped += *c;
				}
			}

			snprintf(json, sizeof(json),
				 R"({"index":%u,"number_modes":%u,"custom_mode":%u,"standard_mode":%u,"properties":%u,"name":"%s"})",
				 am.mode_index, am.number_modes, am.custom_mode, am.standard_mode, am.properties, escaped.c_str());
			publish(vehicleTopic(msg.sysid, "available_mode"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR: {
			mavlink_available_modes_monitor_t mon;
			mavlink_msg_available_modes_monitor_decode(&msg, &mon);
			snprintf(json, sizeof(json), R"({"seq":%u})", mon.seq);
			publish(vehicleTopic(msg.sysid, "available_modes_monitor"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_STATUSTEXT: {
			mavlink_statustext_t st;
			mavlink_msg_statustext_decode(&msg, &st);
			char text[51] {};
			memcpy(text, st.text, 50); // not NUL-terminated when 50 chars long
			std::string escaped;

			for (const char *c = text; *c; ++c) {
				if (*c == '"' || *c == '\\') {
					escaped += '\\';
				}

				if (static_cast<unsigned char>(*c) >= 0x20) {
					escaped += *c;
				}
			}

			snprintf(json, sizeof(json), R"({"severity":%u,"text":"%s"})", st.severity, escaped.c_str());
			publish(vehicleTopic(msg.sysid, "statustext"), std::string(json));
			break;
		}

	case MAVLINK_MSG_ID_PARAM_VALUE: {
			mavlink_param_value_t pv;
			mavlink_msg_param_value_decode(&msg, &pv);
			char id[17] {};
			memcpy(id, pv.param_id, 16); // not NUL-terminated when the name is 16 chars

			if (pv.param_type == MAV_PARAM_TYPE_REAL32) {
				snprintf(json, sizeof(json), R"({"name":"%s","value":%.9g,"type":"float","index":%u,"count":%u})",
					 id, static_cast<double>(pv.param_value), pv.param_index, pv.param_count);

			} else {
				// PX4 encodes integer parameters bytewise in the float field
				int32_t ivalue = 0;
				memcpy(&ivalue, &pv.param_value, sizeof(ivalue));
				snprintf(json, sizeof(json), R"({"name":"%s","value":%d,"type":"int","index":%u,"count":%u})",
					 id, static_cast<int>(ivalue), pv.param_index, pv.param_count);
			}

			publish(vehicleTopic(msg.sysid, "param"), std::string(json));
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

	for (const uint32_t msg_id : {MAVLINK_MSG_ID_LOCAL_POSITION_NED, MAVLINK_MSG_ID_ATTITUDE,
				      MAVLINK_MSG_ID_GLOBAL_POSITION_INT}) {
		mavlink_message_t msg;
		mavlink_msg_command_long_pack_chan(_config.sysid, _config.compid, CHAN_UDP, &msg,
						   target_sys, target_comp, CMD_SET_MESSAGE_INTERVAL, 0,
						   static_cast<float>(msg_id), interval_us, 0, 0, 0, 0, 0);
		sendMavlink(msg);
	}

	std::cout << "Requested LOCAL_POSITION_NED/ATTITUDE/GLOBAL_POSITION_INT at " << _config.stream_rate_hz
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
