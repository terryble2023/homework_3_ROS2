#ifndef EULER_TO_QUATERNION_NODE_HPP_
#define EULER_TO_QUATERNION_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "euler_to_quaternion_interface/msg/my_euler.hpp"
#include "euler_to_quaternion_interface/msg/my_quaternion.hpp"

class EulerToQuaternionNode : public rclcpp::Node
{
public:
    EulerToQuaternionNode();

private:
    using MyEuler = euler_to_quaternion_interface::msg::MyEuler;
    using MyQuaternion = euler_to_quaternion_interface::msg::MyQuaternion;

    void eulerCallback(const MyEuler::SharedPtr msg);
    MyQuaternion eulerToQuaternion(double roll, double pitch, double yaw) const;
    void broadcastTransform(
        const std::string & child_frame,
        double x, double y, double z,
        const geometry_msgs::msg::Quaternion & q);

    rclcpp::Subscription<MyEuler>::SharedPtr subscription_;
    rclcpp::Publisher<MyQuaternion>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // EULER_TO_QUATERNION_NODE_HPP_