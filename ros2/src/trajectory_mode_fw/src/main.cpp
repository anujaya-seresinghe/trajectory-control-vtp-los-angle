#include <chrono>
#include <thread>

#include <px4_ros2/common/exception.hpp>
#include <rclcpp/rclcpp.hpp>

#include "trajectory_mode_fw/mode.hpp"

using namespace std::chrono_literals;

// Like px4_ros2::NodeWithMode, but registration is deferred until the vehicle is in fixed-wing flight:
// PX4 rejects the fixed-wing setpoint type while a VTOL is still a multicopter, and px4_ros2 treats that
// as fatal at registration. Registration runs while the node is not spinning (like NodeWithMode).
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("trajectory_mode_fw");
	auto mode = std::make_unique<trajectory_mode_fw::TrajectoryModeFw>(*node);

	bool registered = false;
	bool announced = false;

	while (rclcpp::ok() && !registered) {
		{
			rclcpp::executors::SingleThreadedExecutor wait_executor;
			wait_executor.add_node(node);

			while (rclcpp::ok() && !mode->inFixedWingFlight()) {
				if (!announced) {
					RCLCPP_INFO(node->get_logger(), "Waiting for fixed-wing flight to register the \"%s\" mode...",
						    trajectory_mode_fw::TrajectoryModeFw::kName);
					announced = true;
				}

				wait_executor.spin_some(100ms);
				std::this_thread::sleep_for(100ms);
			}

			wait_executor.remove_node(node);
		}

		if (!rclcpp::ok()) {
			break;
		}

		try {
			registered = mode->doRegister();

		} catch (const px4_ros2::Exception &e) {
			RCLCPP_WARN(node->get_logger(), "Registration failed: %s", e.what());
		}

		if (!registered) {
			RCLCPP_WARN(node->get_logger(), "Registration failed, retrying once in fixed-wing flight again");
			announced = false;
			std::this_thread::sleep_for(2s);
		}
	}

	if (registered) {
		RCLCPP_INFO(node->get_logger(), "Mode \"%s\" registered", trajectory_mode_fw::TrajectoryModeFw::kName);
		rclcpp::spin(node);
	}

	mode.reset();
	rclcpp::shutdown();
	return 0;
}
