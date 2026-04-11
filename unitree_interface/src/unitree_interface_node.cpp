#include "unitree_interface/unitree_interface.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<unitree_interface::UnitreeInterface>(options);
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        4
    );
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}