#include "unitree_interface/unitree_interface.hpp"

#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<unitree_interface::UnitreeInterface>(options);
    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}