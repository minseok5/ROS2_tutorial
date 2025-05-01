#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class CrazyflieCommandPublisher : public rclcpp::Node
{
public:
    CrazyflieCommandPublisher()
        : Node("crazyflie_command_publisher"), phase_(0)
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd/vel", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cmd/pose", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CrazyflieCommandPublisher::publish_command, this));
    }

private:
    void publish_command()
    {
        if (phase_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Phase 1: Takeoff");
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.z = goal_height_;
            pose_pub_->publish(pose_msg);
        } else if (phase_ == 1) {
            RCLCPP_INFO(this->get_logger(), "Phase 2: Moving to goal");
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.x = goal_x_;
            pose_msg.pose.position.y = goal_y_;
            pose_msg.pose.position.z = goal_height_;
            pose_msg.pose.orientation.w = 1.0; // Adjust as needed
            pose_pub_->publish(pose_msg);
        } else if (phase_ == 2) {
            RCLCPP_INFO(this->get_logger(), "Phase 3: Landing");
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.x = goal_x_;
            pose_msg.pose.position.y = goal_y_;
            pose_msg.pose.position.z = 0.0;
            pose_pub_->publish(pose_msg);
        }

        phase_++;
        if (phase_ > 2)
            phase_ = 0;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int phase_;
    const double goal_x_ = 1.0;
    const double goal_y_ = 1.0;
    const double goal_height_ = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CrazyflieCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
