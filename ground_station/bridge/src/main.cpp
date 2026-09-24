#include "bridge.hpp"

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

static MavlinkMqttBridge *g_bridge = nullptr;

static void onSignal(int)
{
	if (g_bridge) {
		g_bridge->stop();
	}
}

static void usage(const char *prog)
{
	const BridgeConfig d;
	std::cout << "Usage: " << prog << " [options]\n"
		  << "  --mqtt-host HOST     MQTT broker host            (default " << d.mqtt_host << ")\n"
		  << "  --mqtt-port PORT     MQTT broker TCP port        (default " << d.mqtt_port << ")\n"
		  << "  --prefix STR         Telemetry topic prefix      (default " << d.topic_prefix << ")\n"
		  << "  --udp-listen PORT    Local MAVLink UDP port      (default " << d.udp_listen_port << ")\n"
		  << "  --udp-target H:P     Vehicle MAVLink UDP address (default " << d.udp_target_host << ":"
		  << d.udp_target_port << ")\n"
		  << "                       (replaced by the source address of incoming packets)\n"
		  << "  --sysid N            Bridge MAVLink system id    (default " << int(d.sysid) << ")\n"
		  << "  --rate HZ            Requested position/attitude rate, 0 = don't request (default "
		  << d.stream_rate_hz << ")\n"
		  << "  --no-raw             Don't publish raw frames on mavlink/rx\n"
		  << "  --upload-interval MS Spacing of trajectory upload messages (default " << d.upload_interval_ms
		  << " ms)\n";
}

int main(int argc, char *argv[])
{
	BridgeConfig config;

	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];
		auto next = [&]() -> const char * {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for " << arg << std::endl;
				std::exit(1);
			}

			return argv[++i];
		};

		if (arg == "--mqtt-host") {
			config.mqtt_host = next();

		} else if (arg == "--mqtt-port") {
			config.mqtt_port = std::atoi(next());

		} else if (arg == "--prefix") {
			config.topic_prefix = next();

		} else if (arg == "--udp-listen") {
			config.udp_listen_port = std::atoi(next());

		} else if (arg == "--udp-target") {
			const std::string target = next();
			const auto colon = target.rfind(':');

			if (colon == std::string::npos) {
				std::cerr << "--udp-target expects HOST:PORT" << std::endl;
				return 1;
			}

			config.udp_target_host = target.substr(0, colon);
			config.udp_target_port = std::atoi(target.c_str() + colon + 1);

		} else if (arg == "--sysid") {
			config.sysid = static_cast<uint8_t>(std::atoi(next()));

		} else if (arg == "--rate") {
			config.stream_rate_hz = std::strtof(next(), nullptr);

		} else if (arg == "--upload-interval") {
			config.upload_interval_ms = std::max(0, std::atoi(next()));

		} else if (arg == "--no-raw") {
			config.publish_raw = false;

		} else if (arg == "-h" || arg == "--help") {
			usage(argv[0]);
			return 0;

		} else {
			std::cerr << "Unknown option: " << arg << std::endl;
			usage(argv[0]);
			return 1;
		}
	}

	MavlinkMqttBridge bridge(config);
	g_bridge = &bridge;
	std::signal(SIGINT, onSignal);
	std::signal(SIGTERM, onSignal);

	if (!bridge.init()) {
		return 1;
	}

	bridge.run();
	std::cout << "Shutting down" << std::endl;
	return 0;
}
