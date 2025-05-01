#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/msg/position.hpp"

class CmdPositionPublisher : public rclcpp::Node {
    public:
    CmdPositionPublisher()
        : Node("cmd_position_publisher") {
            publisher_ = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf231/cmd_position", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CmdPositionPublisher::publish_position, this)); 
    }

    private:
        void publish_position() {
            auto msg = crazyflie_interfaces::msg::Position();
            msg.x = 0.0;
            msg.y = 0.0;
            msg.z = 1.0;
            msg.yaw = 0.0;

            RCLCPP_INFO(this->get_logger(), "Publishing: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", msg.x, msg.y, msg.z, msg.yaw);

            publisher_->publish(msg);
    }

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdPositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
