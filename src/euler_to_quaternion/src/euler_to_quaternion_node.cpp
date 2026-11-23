#include "euler_to_quaternion_node.hpp"

#include <cmath>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"

EulerToQuaternionNode::EulerToQuaternionNode()
: Node("euler_to_quaternion")
{
    subscription_ = this->create_subscription<MyEuler>(
        "/euler", 10,
        std::bind(&EulerToQuaternionNode::eulerCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<MyQuaternion>("/quaternion", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void EulerToQuaternionNode::eulerCallback(const MyEuler::SharedPtr msg)
{
    const auto q = eulerToQuaternion(msg->roll, msg->pitch, msg->yaw);

    RCLCPP_INFO(
        this->get_logger(),
        "欧拉角: roll=%.4f, pitch=%.4f, yaw=%.4f -> "
        "四元数: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
        msg->roll, msg->pitch, msg->yaw,
        q.x, q.y, q.z, q.w);

    publisher_->publish(q);

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x;
    q_msg.y = q.y;
    q_msg.z = q.z;
    q_msg.w = q.w;

    broadcastTransform("A", 0.0, 0.0, 0.0, q_msg);
    broadcastTransform("B", 1.0, 1.0, 2.0, q_msg);
}

EulerToQuaternionNode::MyQuaternion EulerToQuaternionNode::eulerToQuaternion(
    double roll, double pitch, double yaw) const
{
    const double sr = std::sin(roll / 2.0);
    const double cr = std::cos(roll / 2.0);
    const double sp = std::sin(pitch / 2.0);
    const double cp = std::cos(pitch / 2.0);
    const double sy = std::sin(yaw / 2.0);
    const double cy = std::cos(yaw / 2.0);

    MyQuaternion q;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.w = cr * cp * cy + sr * sp * sy;
    return q;
}

void EulerToQuaternionNode::broadcastTransform(
    const std::string & child_frame,
    double x, double y, double z,
    const geometry_msgs::msg::Quaternion & q)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = child_frame;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    t.transform.rotation = q;

    tf_broadcaster_->sendTransform(t);
}