// Minimal stand-in for PX4 SITL: flies a lemniscate at 10 m altitude and streams
// HEARTBEAT, LOCAL_POSITION_NED and ATTITUDE over UDP using the same ports as the
// PX4 SITL API link (listens on 14580, sends to 14540).
//
//   ./fake_vehicle [target_host] [target_port] [listen_port]

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <iostream>

#include <mavlink/common/mavlink.h>

static volatile bool g_running = true;

int main(int argc, char *argv[])
{
	const char *target_host = argc > 1 ? argv[1] : "127.0.0.1";
	const int target_port = argc > 2 ? std::atoi(argv[2]) : 14540;
	const int listen_port = argc > 3 ? std::atoi(argv[3]) : 14580;

	std::signal(SIGINT, [](int) { g_running = false; });

	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in local{};
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	local.sin_port = htons(listen_port);

	if (bind(sock, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
		perror("bind");
		return 1;
	}

	sockaddr_in target{};
	target.sin_family = AF_INET;
	target.sin_port = htons(target_port);
	inet_pton(AF_INET, target_host, &target.sin_addr);

	auto send = [&](const mavlink_message_t &msg) {
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, reinterpret_cast<sockaddr *>(&target), sizeof(target));
	};

	std::cout << "Fake vehicle sending to " << target_host << ":" << target_port
		  << ", listening on " << listen_port << std::endl;

	constexpr uint8_t sysid = 1, compid = MAV_COMP_ID_AUTOPILOT1;
	// PX4 custom_mode = main_mode << 16 | sub_mode << 24. Start in AUTO (4) / LOITER "Hold" (3);
	// MAV_CMD_DO_SET_MODE (176) switches it, e.g. to AUTO / TRAJ (9).
	uint32_t custom_mode = (4u << 16) | (3u << 24);

	const auto start = std::chrono::steady_clock::now();
	auto last_hb = start - std::chrono::seconds(1);
	mavlink_message_t rx;
	mavlink_status_t rx_status;
	unsigned traj_expected = 0, traj_received = 0, traj_next_index = 0;
	bool traj_in_order = true;

	while (g_running) {
		const auto now = std::chrono::steady_clock::now();
		const double t = std::chrono::duration<double>(now - start).count();
		const uint32_t t_ms = static_cast<uint32_t>(t * 1000.0);

		if (now - last_hb >= std::chrono::seconds(1)) {
			mavlink_message_t msg;
			mavlink_msg_heartbeat_pack(sysid, compid, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
						   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
						   custom_mode, MAV_STATE_ACTIVE);
			send(msg);
			last_hb = now;
		}

		// Lemniscate of Gerono (figure eight), 40 m x 20 m, with slow altitude oscillation
		const double w = 2.0 * M_PI / 40.0;
		const double s = w * t;
		const double x = 20.0 * std::sin(s);
		const double y = 20.0 * std::sin(s) * std::cos(s);
		const double z = -10.0 - 2.0 * std::sin(0.5 * s);
		const double vx = 20.0 * w * std::cos(s);
		const double vy = 20.0 * w * std::cos(2.0 * s);
		const double vz = -1.0 * w * std::cos(0.5 * s);
		const double yaw = std::atan2(vy, vx);

		mavlink_message_t msg;
		mavlink_msg_local_position_ned_pack(sysid, compid, &msg, t_ms, x, y, z, vx, vy, vz);
		send(msg);

		mavlink_msg_attitude_pack(sysid, compid, &msg, t_ms, 0.08f * std::cos(s), -0.1f, yaw, 0, 0, 0);
		send(msg);

		// Print anything received (e.g. SET_MESSAGE_INTERVAL from the bridge, or mavlink/tx traffic)
		pollfd pfd{sock, POLLIN, 0};

		while (poll(&pfd, 1, 0) > 0) {
			uint8_t buf[2048];
			const ssize_t n = recv(sock, buf, sizeof(buf), 0);

			for (ssize_t i = 0; i < n; ++i) {
				if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx, &rx_status)
				    || rx.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
					continue;
				}

				if (rx.msgid == MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE) {
					mavlink_trajectory_setpoint_initiate_t init;
					mavlink_msg_trajectory_setpoint_initiate_decode(&rx, &init);
					traj_expected = init.no_of_waypoints;
					traj_received = 0;
					traj_next_index = 0;
					traj_in_order = true;
					std::cout << "TRAJECTORY_SETPOINT_INITIATE id=" << int(init.id) << " no_of_waypoints="
						  << init.no_of_waypoints << std::endl;

				} else if (rx.msgid == MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD) {
					mavlink_trajectory_setpoint_upload_t wp;
					mavlink_msg_trajectory_setpoint_upload_decode(&rx, &wp);
					traj_in_order &= wp.index == traj_next_index;
					traj_next_index = wp.index + 1;
					++traj_received;

					if (wp.index < 2 || wp.index + 1u == traj_expected) {
						std::printf("  UPLOAD id=%u index=%u x=%.2f y=%.2f vx=%.2f vy=%.2f at=%.3f jt=%.3f t=%.3f\n",
							    wp.id, wp.index, wp.x, wp.y, wp.vx, wp.vy, wp.at, wp.jt, wp.t);
					}

					if (traj_received == traj_expected) {
						std::cout << "Received " << traj_received << "/" << traj_expected << " waypoints, indices "
							  << (traj_in_order ? "contiguous" : "OUT OF ORDER") << std::endl;
					}

				} else if (rx.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
					mavlink_command_long_t cmd;
					mavlink_msg_command_long_decode(&rx, &cmd);
					uint8_t result = MAV_RESULT_ACCEPTED;

					if (cmd.command == 176) { // MAV_CMD_DO_SET_MODE
						const uint32_t main_mode = static_cast<uint32_t>(cmd.param2);
						const uint32_t sub_mode = static_cast<uint32_t>(cmd.param3);
						custom_mode = (main_mode << 16) | (sub_mode << 24);
						std::cout << "DO_SET_MODE main=" << main_mode << " sub=" << sub_mode << std::endl;
					}

					mavlink_message_t ack;
					mavlink_msg_command_ack_pack(sysid, compid, &ack, cmd.command, result, 0, 0, rx.sysid, rx.compid);
					send(ack);

				} else {
					std::cout << "rx msgid " << rx.msgid << " from " << int(rx.sysid) << "/" << int(rx.compid)
						  << std::endl;
				}
			}
		}

		usleep(20000); // 50 Hz
	}

	close(sock);
	return 0;
}
