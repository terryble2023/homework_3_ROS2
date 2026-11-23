#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "euler_to_quaternion_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EulerToQuaternionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}