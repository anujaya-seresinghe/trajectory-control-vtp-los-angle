#include "trajectory_mode_fw/mavlink_link.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <mavlink/common/mavlink.h>

namespace trajectory_mode_fw
{

namespace
{

constexpr mavlink_channel_t CHAN_RX = MAVLINK_COMM_0;
constexpr mavlink_channel_t CHAN_TX = MAVLINK_COMM_1;
constexpr uint8_t COMP_ID_ALL = 0;
constexpr uint8_t SEVERITY_WARNING = 4; // MAV_SEVERITY_WARNING
constexpr uint8_t SEVERITY_INFO = 6;    // MAV_SEVERITY_INFO

const auto kStart = std::chrono::steady_clock::now();

std::string paramId(const char *id)
{
	char name[17] {};
	memcpy(name, id, 16); // not NUL-terminated when 16 chars long
	return name;
}

} // namespace

MavlinkLink::MavlinkLink(Config config, std::vector<std::string> param_names, ParamGetFn get, ParamSetFn set,
			 LogFn log_info, LogFn log_error)
	: _config(std::move(config)), _param_names(std::move(param_names)), _param_get(std::move(get)),
	  _param_set(std::move(set)), _log_info(std::move(log_info)), _log_error(std::move(log_error))
{
	_sock = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	local.sin_port = htons(static_cast<uint16_t>(_config.local_port));

	if (_sock < 0 || bind(_sock, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0) {
		_log_error("MAVLink: cannot bind UDP port " + std::to_string(_config.local_port) + ": " + strerror(errno)
			   + ". Is another program using it?");

	} else {
		timeval timeout{0, 100000}; // wake up for heartbeats and shutdown
		setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	}

	_remote.sin_family = AF_INET;
	_remote.sin_port = htons(static_cast<uint16_t>(_config.remote_port));
	inet_pton(AF_INET, _config.remote_host.c_str(), &_remote.sin_addr);

	_log_info("MAVLink: component " + std::to_string(_config.sysid) + "/" + std::to_string(_config.compid)
		  + " on UDP " + std::to_string(_config.local_port) + " -> " + _config.remote_host + ":"
		  + std::to_string(_config.remote_port));

	_thread = std::thread(&MavlinkLink::run, this);
}

MavlinkLink::~MavlinkLink()
{
	_running = false;

	if (_thread.joinable()) {
		_thread.join();
	}

	if (_sock >= 0) {
		close(_sock);
	}
}

uint32_t MavlinkLink::timeBootMs() const
{
	return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
					     std::chrono::steady_clock::now() - kStart).count());
}

void MavlinkLink::run()
{
	auto last_heartbeat = std::chrono::steady_clock::time_point{};
	bool px4_seen = false;
	uint8_t buf[2048];

	while (_running) {
		const auto now = std::chrono::steady_clock::now();

		// The heartbeat makes PX4's onboard link "see" this component, so PX4 forwards messages addressed to it
		if (now - last_heartbeat >= std::chrono::seconds(1)) {
			last_heartbeat = now;
			send([this](mavlink_message_t &msg) {
				mavlink_msg_heartbeat_pack_chan(_config.sysid, _config.compid, CHAN_TX, &msg, 18 /* ONBOARD_CONTROLLER */,
								8 /* MAV_AUTOPILOT_INVALID */, 0, 0, 4 /* MAV_STATE_ACTIVE */);
			});
		}

		if (_sock < 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		sockaddr_in from{};
		socklen_t from_len = sizeof(from);
		const ssize_t len = recvfrom(_sock, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &from_len);

		if (len <= 0) {
			continue;
		}

		if (!px4_seen) {
			px4_seen = true;
			char ip[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
			_log_info(std::string("MAVLink: receiving from PX4 at ") + ip + ":" + std::to_string(ntohs(from.sin_port)));
		}

		mavlink_message_t msg;
		mavlink_status_t status;

		for (ssize_t i = 0; i < len; ++i) {
			if (mavlink_parse_char(CHAN_RX, buf[i], &msg, &status)) {
				handleMessage(msg);
			}
		}
	}
}

void MavlinkLink::handleMessage(const mavlink_message_t &msg)
{
	if (msg.sysid == _config.sysid && msg.compid == _config.compid) {
		return; // our own messages, e.g. echoed by a router
	}

	const auto for_us = [this](uint8_t target_system, uint8_t target_component, bool allow_all) {
		return target_system == _config.sysid
		       && (target_component == _config.compid || (allow_all && target_component == COMP_ID_ALL));
	};

	switch (msg.msgid) {
	case MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE: {
			mavlink_trajectory_setpoint_initiate_t init;
			mavlink_msg_trajectory_setpoint_initiate_decode(&msg, &init);
			_upload.initiate(init.id, init.no_of_waypoints);
			_log_info("Trajectory " + std::to_string(init.id) + " initiated, " + std::to_string(init.no_of_waypoints)
				  + " waypoints");
			break;
		}

	case MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD: {
			mavlink_trajectory_setpoint_upload_t up;
			mavlink_msg_trajectory_setpoint_upload_decode(&msg, &up);
			std::optional<Trajectory> complete = _upload.add(up.index, {up.x, up.y, up.vx, up.vy, up.at, up.jt, up.t});

			if (complete) {
				const std::string text = "Trajectory " + std::to_string(complete->id) + " received ("
							 + std::to_string(complete->points.size()) + " points)";
				_log_info(text);
				sendStatusText(SEVERITY_INFO, text);
				std::lock_guard<std::mutex> lock(_mutex);
				_pending = std::move(complete);
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			mavlink_param_request_list_t req;
			mavlink_msg_param_request_list_decode(&msg, &req);

			if (for_us(req.target_system, req.target_component, true)) {
				for (const std::string &name : _param_names) {
					sendParamValue(name);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			mavlink_param_request_read_t req;
			mavlink_msg_param_request_read_decode(&msg, &req);

			if (!for_us(req.target_system, req.target_component, true)) {
				break;
			}

			if (req.param_index >= 0) {
				if (static_cast<size_t>(req.param_index) < _param_names.size()) {
					sendParamValue(_param_names[req.param_index]);
				}

			} else {
				sendParamValue(paramId(req.param_id)); // unknown names are not answered, like PX4
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET: {
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(&msg, &set);

			if (!for_us(set.target_system, set.target_component, false)) {
				break;
			}

			const std::string name = paramId(set.param_id);

			if (!_param_get(name)) {
				break;
			}

			// On success the mode's parameter callback broadcasts the new PARAM_VALUE
			const std::string error = std::isfinite(set.param_value) ? _param_set(name, set.param_value)
						  : "not a finite number";

			if (!error.empty()) {
				// The reason first, then the unchanged value (the MAVLink parameter protocol has no error reply)
				sendStatusText(SEVERITY_WARNING, (name + " rejected: " + error).substr(0, 50));
				sendParamValue(name);
			}

			break;
		}

	default:
		break;
	}
}

std::optional<Trajectory> MavlinkLink::take()
{
	std::lock_guard<std::mutex> lock(_mutex);
	std::optional<Trajectory> out = std::move(_pending);
	_pending.reset();
	return out;
}

void MavlinkLink::sendParamValue(const std::string &name)
{
	const std::optional<float> value = _param_get(name);

	if (!value) {
		return;
	}

	uint16_t index = 0;

	while (index < _param_names.size() && _param_names[index] != name) {
		++index;
	}

	char id[17] {};
	strncpy(id, name.c_str(), 16);
	send([&](mavlink_message_t &msg) {
		mavlink_msg_param_value_pack_chan(_config.sysid, _config.compid, CHAN_TX, &msg, id, *value,
						  MAV_PARAM_TYPE_REAL32, static_cast<uint16_t>(_param_names.size()), index);
	});
}

void MavlinkLink::sendNamedValue(const char *name, float value)
{
	char id[11] {};
	strncpy(id, name, 10);
	const uint32_t time_boot_ms = timeBootMs();
	send([&](mavlink_message_t &msg) {
		mavlink_msg_named_value_float_pack_chan(_config.sysid, _config.compid, CHAN_TX, &msg, time_boot_ms, id, value);
	});
}

void MavlinkLink::sendStatusText(uint8_t severity, const std::string &text)
{
	char buf[51] {};
	strncpy(buf, text.c_str(), 50);
	send([&](mavlink_message_t &msg) {
		mavlink_msg_statustext_pack_chan(_config.sysid, _config.compid, CHAN_TX, &msg, severity, buf, 0, 0);
	});
}

void MavlinkLink::send(const std::function<void(mavlink_message_t &msg)> &pack)
{
	std::lock_guard<std::mutex> lock(_tx_mutex);
	mavlink_message_t msg;
	pack(msg);
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	if (_sock >= 0) {
		sendto(_sock, buf, len, 0, reinterpret_cast<const sockaddr *>(&_remote), sizeof(_remote));
	}
}

} // namespace trajectory_mode_fw
