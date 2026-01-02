#include "rclcpp/rclcpp.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObjectFollower : public rclcpp::Node
{
public:
    ObjectFollower() : Node("object_follower")
    {
        _cosestObjectSubsriber = this->create_subscription<nav_2d_msgs::msg::Twist2D>(
            "closest_object",
            10,
            std::bind(&ObjectFollower::closest_object_callback, this, std::placeholders::_1)
        );
        _cmdVelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_DEBUG(this->get_logger(), "ObjectFollower node has been started.");
    }
private:
    void closest_object_callback(const nav_2d_msgs::msg::Twist2D::SharedPtr msg)
    {
        if (msg->x < 0.0) {
            RCLCPP_DEBUG(this->get_logger(), "No valid closest object data received.");
            cmd.linear.x = 0.0;
            cmd.angular.z = 2.5; // Rotate in place to search for object
        } else {
            cmd.linear.x = std::min(1.5 * (msg->x - 1.0), 2.0);
            cmd.angular.z = msg->theta * 10.0f;
            // RCLCPP_INFO(this->get_logger(), "Following object at distance: %.2f meters and angle: %.2f radians", msg->x, msg->theta);
        }
        _cmdVelPublisher->publish(cmd);
    }
    geometry_msgs::msg::Twist cmd = geometry_msgs::msg::Twist();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmdVelPublisher;
    rclcpp::Subscription<nav_2d_msgs::msg::Twist2D>::SharedPtr _cosestObjectSubsriber;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
