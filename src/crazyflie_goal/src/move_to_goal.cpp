#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TakeoffNode : public rclcpp::Node
{
public:
    TakeoffNode() : Node("takeoff_node"), reached_height_(false)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cmd/takeoff", 10);
        status_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "drone/status", 10,
            std::bind(&TakeoffNode::status_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TakeoffNode::publish_takeoff, this));
    }

private:
    void publish_takeoff()
    {
        if (!reached_height_)
        {
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.z = goal_height_;
            pose_pub_->publish(pose_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing takeoff command");
        }
    }

    void status_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg->pose.position.z >= goal_height_ - 0.05)
        {
            reached_height_ = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff complete, ready to move");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool reached_height_;
    const double goal_height_ = 1.0;
};

class MoveNode : public rclcpp::Node
{
public:
    MoveNode() : Node("move_node"), reached_goal_(false)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cmd/move", 10);
        status_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "drone/status", 10,
            std::bind(&MoveNode::status_callback, this, std::placeholders::_1));
    }

private:
    void status_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!reached_goal_ && msg->pose.position.z >= 0.9)
        {
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.x = goal_x_;
            pose_msg.pose.position.y = goal_y_;
            pose_msg.pose.position.z = goal_height_;
            pose_pub_->publish(pose_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing move command");
        }
        if (msg->pose.position.x == goal_x_ && msg->pose.position.y == goal_y_)
        {
            reached_goal_ = true;
            RCLCPP_INFO(this->get_logger(), "Move complete, ready to land");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr status_sub_;
    bool reached_goal_;
    const double goal_x_ = 1.0;
    const double goal_y_ = 1.0;
    const double goal_height_ = 1.0;
};

class LandingNode : public rclcpp::Node
{
public:
    LandingNode() : Node("landing_node"), landed_(false)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cmd/land", 10);
        status_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "drone/status", 10,
            std::bind(&LandingNode::status_callback, this, std::placeholders::_1));
    }

private:
    void status_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!landed_ && msg->pose.position.x == goal_x_ && msg->pose.position.y == goal_y_)
        {
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.z = 0.0;
            pose_pub_->publish(pose_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing landing command");
        }
        if (msg->pose.position.z <= 0.05)
        {
            landed_ = true;
            RCLCPP_INFO(this->get_logger(), "Landing complete, shutting down");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr status_sub_;
    bool landed_;
    const double goal_x_ = 1.0;
    const double goal_y_ = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto takeoff_node = std::make_shared<TakeoffNode>();
    auto move_node = std::make_shared<MoveNode>();
    auto landing_node = std::make_shared<LandingNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(takeoff_node);
    executor.add_node(move_node);
    executor.add_node(landing_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
