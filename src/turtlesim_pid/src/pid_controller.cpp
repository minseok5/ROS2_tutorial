#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class PIDController : public rclcpp::Node {
    public:
        PIDController() : Node("pid_controller"), Kp_l_(1.0), Ki_l_(0), Kd_l_(0.1), Kp_a_(4), Ki_a_(0.1), Kd_a_(1), prev_error_l_(0.0), prev_error_a_(0.0), integral_l_(0.0), integral_a_(0.0) {
            pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10, std::bind(&PIDController::pose_callback, this, std::placeholders::_1));
    
            goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/goal_position", 10, std::bind(&PIDController::goal_callback, this, std::placeholders::_1));
    
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        }
    
    private:
        void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
            current_x_ = msg->x;
            current_y_ = msg->y;
            current_theta_ = msg->theta;
            control_loop();
        }
    
        void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
            goal_x_ = msg->x;
            goal_y_ = msg->y;
        }

        void control_loop() {
            double error_x = goal_x_ - current_x_;
            double error_y = goal_y_ - current_y_;
            double distance_error = std::sqrt(error_x * error_x + error_y * error_y);    
            
            double angle_to_goal = std::atan2(error_y, error_x);
            double angle_error = angle_to_goal - current_theta_;
            if(std::abs(error_x + error_y) < 0.001)
                angle_error = 0;

            // PID computations
            integral_l_ += distance_error;
            integral_a_ += angle_error;
            double derivative_l = distance_error - prev_error_l_;
            double derivative_a = angle_error - prev_error_a_;
            prev_error_l_ = distance_error;
            prev_error_a_ = angle_error;
    
            double linear_velocity = Kp_l_ * distance_error + Ki_l_ * integral_l_ + Kd_l_ * derivative_l;
            double angular_velocity = Kp_a_ * angle_error + Ki_a_ * integral_a_ + Kd_a_ * derivative_a;
    
            // Create Twist message
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = linear_velocity;
            cmd_msg.angular.z = angular_velocity;
    
            // Publish velocity command
            cmd_pub_->publish(cmd_msg);
        }
    
        // ROS2 components
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
        // PID parameters
        double Kp_l_, Ki_l_, Kd_l_, Kp_a_, Ki_a_, Kd_a_;
        double prev_error_l_, prev_error_a_, integral_l_, integral_a_;
    
        // Turtle position
        double current_x_, current_y_, current_theta_;
        double goal_x_ = 5.0, goal_y_ = 5.0;
    };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}