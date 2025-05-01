#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <thread>

class PosePublisher : public rclcpp::Node {
public:
    PosePublisher() : Node("pose_publisher"), time_(0.0), can_publish_(false) {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal", 10);
        
        // Wait for 5 seconds before starting to publish
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for 5 seconds before starting to publish poses...");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // Start publishing poses
        can_publish_ = true;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // Fast updates every 100ms
            std::bind(&PosePublisher::publishPose, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 Starting to publish poses...");
    }

private:
    void publishPose() {
        if (!can_publish_) return;

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";  // Reference frame

        // Circular motion with 30 seconds per full circle
        double radius = 1.0;
        double angular_velocity = 2 * M_PI / 30.0;  // One full circle in 30 seconds

        // Calculate the position of the drone in the circular path
        double x = radius * std::sin(time_);
        double y = radius * std::cos(time_) - radius;
        double z = 1.0;  // Constant height

        // Update the time (move slower)
        time_ += angular_velocity * 0.1;  // Slow update step for smooth movement

        // Set position data
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;

        // No rotation (identity quaternion)
        msg.pose.orientation.w = 1.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 1.0;
        msg.pose.orientation.z = 0.0;

        // Publish the position to the topic
        pose_publisher_->publish(msg);

        // Log the current position (for debugging)
        RCLCPP_INFO(this->get_logger(), "📡 Publishing Pose: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_;
    bool can_publish_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
