#include "unitree_interface/topology.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("zero_cmd");

    node->declare_parameter("topic", "/position_control");
    node->declare_parameter("rate_hz", 20.0);

    const auto topic = node->get_parameter("topic").as_string();
    const auto rate_hz = node->get_parameter("rate_hz").as_double();

    auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
        topic,
        rclcpp::QoS(10)
    );

    sensor_msgs::msg::JointState msg;

    namespace embodiment = unitree_interface::embodiment;

    for (const auto& joint : embodiment::upper_body) {
        msg.name.emplace_back(embodiment::to_name(joint));
    }

    const auto n = msg.name.size();
    msg.position.resize(n, 0.0);
    msg.velocity.assign(n, 0.0);
    msg.effort.assign(n, 0.0);

    // Declare position setpoint parameters for each arm joint
    for (const auto& joint : embodiment::arms) {
        const auto name = embodiment::to_name(joint);
        node->declare_parameter(name, 0.0);
    }



    const auto period = std::chrono::duration<double>(1.0 / rate_hz);

    auto timer = node->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [&pub, &msg, &node, n]() {
            msg.header.stamp = node->now();
            // Read arm joint position setpoints from parameters
            for (std::size_t i = 0; i < n; ++i) {
                const auto& name = msg.name[i];
                auto idx = embodiment::from_name(name);
                if (idx.has_value() && embodiment::contains(embodiment::arms, *idx)) {
                    msg.position[i] = node->get_parameter(name).as_double();
                }
            }
            pub->publish(msg);
        }
    );

    RCLCPP_INFO(
        node->get_logger(),
        "Publishing zero commands to %s at %.1f Hz (%zu joints)",
        topic.c_str(),
        rate_hz,
        n
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
