#pragma once

// MAVLink link of the fixed-wing Trajectory mode: a MAVLink component of the vehicle's system (default compid 191,
// onboard computer) on its own PX4 MAVLink instance, like a companion computer. PX4 forwards between instances that
// have forwarding on (mavlink start -f / MAV_n_FORWARD), so the ground station reaches this component through PX4
// exactly like the internal multicopter mode, with no PX4 changes. It needs its own instance: PX4 never forwards a
// message back to the instance it came from, and the ground station bridge already uses the SITL onboard link.
//   TRAJECTORY_SETPOINT_INITIATE / _UPLOAD  the same broadcast upload PX4's TrajectoryManager receives
//   PARAM_REQUEST_READ / _LIST / PARAM_SET  the TRAJ_* parameters (addressed to this component)
//   -> PARAM_VALUE, STATUSTEXT, NAMED_VALUE_FLOAT (guidance state), HEARTBEAT

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>

#include "trajectory_mode_fw/trajectory_upload.hpp"

struct __mavlink_message; // mavlink_message_t (mavlink_types.h), kept out of this header

namespace trajectory_mode_fw
{

class MavlinkLink
{
public:
	struct Config {
		std::string remote_host{"127.0.0.1"};
		int remote_port{14591}; // PX4 instance: mavlink start -u 14591 -o 14590 -f ...
		int local_port{14590};
		uint8_t sysid{1};
		uint8_t compid{191};
	};

	using LogFn = std::function<void(const std::string &)>;
	// Parameter access, called on the link thread. get: nullopt if unknown. set: "" or the reason it was rejected.
	using ParamGetFn = std::function<std::optional<float>(const std::string &name)>;
	using ParamSetFn = std::function<std::string(const std::string &name, float value)>;

	MavlinkLink(Config config, std::vector<std::string> param_names, ParamGetFn get, ParamSetFn set, LogFn log_info,
		    LogFn log_error);
	~MavlinkLink();

	MavlinkLink(const MavlinkLink &) = delete;
	MavlinkLink &operator=(const MavlinkLink &) = delete;

	// Latest complete trajectory received since the last call, if any (thread-safe)
	std::optional<Trajectory> take();

	// Thread-safe senders
	void sendParamValue(const std::string &name);
	void sendNamedValue(const char *name, float value);
	void sendStatusText(uint8_t severity, const std::string &text);

private:
	void run();
	void handleMessage(const ::__mavlink_message &msg);
	// Packs (under the TX lock: packing advances the channel's sequence number) and sends one message
	void send(const std::function<void(::__mavlink_message &msg)> &pack);
	uint32_t timeBootMs() const;

	Config _config;
	std::vector<std::string> _param_names; // the parameter index is the position in this list
	ParamGetFn _param_get;
	ParamSetFn _param_set;
	LogFn _log_info;
	LogFn _log_error;

	int _sock{-1};
	sockaddr_in _remote{};
	std::mutex _tx_mutex;
	std::atomic<bool> _running{true};
	std::thread _thread;

	TrajectoryUpload _upload; // link thread only
	std::mutex _mutex;
	std::optional<Trajectory> _pending;
};

} // namespace trajectory_mode_fw
