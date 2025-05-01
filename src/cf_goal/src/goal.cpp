#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalPublisher : public rclcpp::Node
{
    public:
        GoalPublisher() : Node("goal_publisher")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/crazyflie/pose", 10);
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GoalPublisher::publish_goal, this));
        }

    private:
        void publish_goal()
        {
            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "map";
            message.pose.position.x = 1.0;
            message.pose.position.y = 1.0;
            message.pose.position.z = 1.0;
            message.pose.orientation.w = 0.0;

            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing goal: [%.2f, %.2f, %.2f, %.2f]", message.pose.position.x, message.pose.position.y, message.pose.position.z, message.pose.orientation.w);
        
            // rclcpp::shutdown();
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    // rclcpp::spin_some(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}