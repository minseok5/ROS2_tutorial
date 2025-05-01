#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/goal_position", 10);
        timer_ = this->create_wall_timer(
            0.01s,  // Uses chrono_literals for readability
            std::bind(&GoalPublisher::publish_goal, this)
        );
    }

private:
    void publish_goal() {
        auto message = geometry_msgs::msg::Point();
        message.x = 3.0;  
        message.y = 3.0;  
        message.z = 0.0;  

        RCLCPP_INFO(this->get_logger(), "Publishing Goal: x=%.2f, y=%.2f", message.x, message.y);
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
