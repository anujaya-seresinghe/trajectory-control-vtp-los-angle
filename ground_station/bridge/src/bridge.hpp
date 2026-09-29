#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>

#include <netinet/in.h>

#include <mavlink/common/mavlink.h>

#include "trajectory.hpp"

struct mosquitto;
struct mosquitto_message;

struct BridgeConfig {
	// MQTT broker (see configs/mqtt/mosquitto.conf)
	std::string mqtt_host{"127.0.0.1"}; // not "localhost": libmosquitto may try IPv6 ::1 first
	int mqtt_port{1883};
	std::string topic_prefix{"uav"};

	// MAVLink UDP link. Defaults match the PX4 SITL "API/offboard" link:
	// PX4 listens on 14580 and sends to 14540.
	int udp_listen_port{14540};
	std::string udp_target_host{"127.0.0.1"};
	int udp_target_port{14580};

	// Identity of this bridge on the MAVLink network (GCS-like)
	uint8_t sysid{255};
	uint8_t compid{MAV_COMP_ID_MISSIONPLANNER};

	// Requested stream rate for LOCAL_POSITION_NED and ATTITUDE [Hz]; <= 0 disables the request
	float stream_rate_hz{30.0f};

	// Publish every received MAVLink frame as raw bytes on mavlink/rx
	bool publish_raw{true};

	// Spacing between TRAJECTORY_SETPOINT_* messages. PX4's TrajectoryManager reads the
	// continuous_trajectory_setpoint uORB topic (queue depth 1), so bursts can drop points.
	int upload_interval_ms{2};
};

/**
 * Bidirectional MAVLink (UDP) <-> MQTT bridge.
 *
 * Vehicle -> MQTT
 *   <prefix>/<sysid>/local_position_ned   JSON  (LOCAL_POSITION_NED)
 *   <prefix>/<sysid>/attitude             JSON  (ATTITUDE)
 *   <prefix>/<sysid>/heartbeat            JSON  (HEARTBEAT, autopilot component only)
 *   mavlink/rx                            raw MAVLink frame bytes (every message)
 *
 * MQTT -> Vehicle
 *   mavlink/tx                            raw MAVLink frame bytes, forwarded to UDP unchanged
 *   <prefix>/<sysid>/cmd/trajectory       JSON {"id": n, "points": [[x, y, vx, vy, at, jt, t], ...]}
 *                                         -> TRAJECTORY_SETPOINT_INITIATE, then one
 *                                            TRAJECTORY_SETPOINT_UPLOAD per point (index 0..N-1)
 *   <prefix>/<sysid>/trajectory_status    JSON upload progress {"id","state","sent","total"[,"error"]}
 *   <prefix>/<sysid>/cmd/set_mode         JSON {"main_mode": n, "sub_mode": n} (PX4 custom mode)
 *                                         -> COMMAND_LONG MAV_CMD_DO_SET_MODE
 *   <prefix>/<sysid>/command_ack          JSON {"command","result"} from COMMAND_ACK
 *   <prefix>/<sysid>/cmd/command_long     JSON {"command": n, "params": [p1..p7]} (null = NaN) -> COMMAND_LONG
 *   <prefix>/<sysid>/cmd/command_int      JSON {"command": n, "frame": n, "params": [p1..p4], "x": int, "y": int, "z": f}
 *                                         -> COMMAND_INT (x/y as degE7 for global frames; null = NaN / INT32_MAX)
 *   <prefix>/<sysid>/global_position      JSON {"lat","lon","alt","relative_alt"} (deg, m AMSL, m) from GLOBAL_POSITION_INT
 *   <prefix>/<sysid>/extended_sys_state   JSON {"vtol_state","landed_state"} from EXTENDED_SYS_STATE
 *   <prefix>/<sysid>/available_mode       JSON {"index","number_modes","custom_mode","standard_mode","properties","name"}
 *                                         from AVAILABLE_MODES (one per mode)
 *   <prefix>/<sysid>/available_modes_monitor JSON {"seq"}: changes when modes are added/removed
 *   <prefix>/<sysid>/statustext           JSON {"severity","text"} from STATUSTEXT (e.g. why arming was denied)
 *   <prefix>/<sysid>/cmd/param_get        JSON {"names": ["TRAJ_R_STAR", ...]} -> PARAM_REQUEST_READ per name
 *   <prefix>/<sysid>/cmd/param_set        JSON {"name": "TRAJ_R_STAR", "value": 30.0} -> PARAM_SET (float)
 *   <prefix>/<sysid>/param                JSON {"name","value","type","index","count"} from PARAM_VALUE
 *
 * Bridge state
 *   bridge/status                         JSON, retained; last-will sets it offline
 */
class MavlinkMqttBridge
{
public:
	explicit MavlinkMqttBridge(const BridgeConfig &config);
	~MavlinkMqttBridge();

	MavlinkMqttBridge(const MavlinkMqttBridge &) = delete;
	MavlinkMqttBridge &operator=(const MavlinkMqttBridge &) = delete;

	bool init();
	void run();  // blocks until stop() is called
	void stop() { _running = false; }

private:
	static void onConnect(mosquitto *mosq, void *obj, int rc);
	static void onMessage(mosquitto *mosq, void *obj, const mosquitto_message *msg);

	bool tryConnectMqtt();
	void handleUdpBytes(const uint8_t *data, size_t len, const sockaddr_in &from);
	void handleMavlinkMessage(const mavlink_message_t &msg);
	void handleMqttTx(const uint8_t *data, size_t len);
	void handleTrajectoryCommand(uint8_t sysid, const std::string &payload);
	void handleSetModeCommand(uint8_t sysid, const std::string &payload);
	void handleCommandLong(uint8_t sysid, const std::string &payload);
	void handleCommandInt(uint8_t sysid, const std::string &payload);
	void handleParamGetCommand(uint8_t sysid, const std::string &payload);
	void handleParamSetCommand(uint8_t sysid, const std::string &payload);
	void serviceUploadQueue();
	void publishUploadStatus(uint8_t sysid, uint8_t id, const char *state, size_t sent, size_t total,
				 const std::string &error = "");

	void sendMavlink(const mavlink_message_t &msg);
	void sendUdp(const uint8_t *data, size_t len);
	void sendHeartbeat();
	void requestStreams(uint8_t target_sys, uint8_t target_comp);

	void publish(const std::string &topic, const std::string &payload, bool retain = false);
	void publish(const std::string &topic, const void *data, size_t len, bool retain = false);
	std::string vehicleTopic(uint8_t sysid, const char *name) const;

	BridgeConfig _config;
	std::atomic<bool> _running{false};

	mosquitto *_mosq{nullptr};
	bool _mqtt_loop_started{false};

	int _sock{-1};
	std::mutex _target_mutex;
	sockaddr_in _target_addr{};
	bool _target_learned{false};

	std::set<uint8_t> _streams_requested;

	// Autopilot component id per system, learned from HEARTBEAT (PX4 uses 1)
	std::array<std::atomic<uint8_t>, 256> _autopilot_compid{};

	// Trajectory upload, filled on the MQTT thread and drained (paced) on the main thread
	struct Upload {
		uint8_t sysid{0};
		uint8_t id{0};
		size_t total{0};
		size_t sent{0};
		std::deque<mavlink_message_t> queue;
	};
	std::mutex _upload_mutex;
	Upload _upload;
};

