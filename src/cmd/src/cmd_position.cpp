#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include "crazyflie_interfaces/msg/position.hpp"

using namespace std::chrono_literals;

class Cmd_Position_Publisher : public rclcpp::Node
{
public:

    Cmd_Position_Publisher() : Node("cmd_position_publisher"), time_cnt(0.0)
    {
        cmd_position_publisher_ = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf231/cmd_position", 10);
        command_loop_timer_ = this->create_wall_timer(10ms, std::bind(&Cmd_Position_Publisher::command_loop_callback, this));
        global_xyz_cmd = Eigen::VectorXd::Zero(4);
    }


private:

    void command_loop_callback()
    {
        publish_cmd_position();

        crazyflie_interfaces::msg::Position global_xyz_cmd_msg;
        global_xyz_cmd_msg.x = global_xyz_cmd[0];
        global_xyz_cmd_msg.y = global_xyz_cmd[1];
        global_xyz_cmd_msg.z = global_xyz_cmd[2];
        global_xyz_cmd_msg.yaw = global_xyz_cmd[3];
        cmd_position_publisher_->publish(global_xyz_cmd_msg);
    }

    void publish_cmd_position()
    {

        time_cnt ++;
        time_real = time_cnt * 0.01;

        if (time_real < 2) global_xyz_cmd[2] = -0.05;
        else if (time_real > 2 && time_real < 2.1)
        global_xyz_cmd[2] = 0.1;
        if (time_real > 2.1 && time_real < 2.2)
        global_xyz_cmd[2] = 0.2;
        if (time_real > 2.2 && time_real < 2.3)
        global_xyz_cmd[2] = 0.3;
        if (time_real > 2.3 && time_real < 2.4)
        global_xyz_cmd[2] = 0.4;


        if (time_real > 5 && time_real < 22)
        {
        global_xyz_cmd[1] = 0.4 * sin (2 * M_PI * 0.2 * (time_real - 5));    
        }

        if (time_real == 10) global_xyz_cmd[3] = 15;
        if (time_real == 15) global_xyz_cmd[3] = -15;
        if (time_real == 20) global_xyz_cmd[3] = 30;

        if (time_real > 30 && global_xyz_cmd[2] > -0.3) global_xyz_cmd[2] -= 0.3 * time_real;
    
        RCLCPP_INFO(this->get_logger(), "%lf, %lf %lf %lf", time_real, global_xyz_cmd[0], global_xyz_cmd[1], global_xyz_cmd[2]);
        
    }

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cmd_position_publisher_;
    rclcpp::TimerBase::SharedPtr command_loop_timer_;

    Eigen::VectorXd global_xyz_cmd;

    double time_cnt;
    double time_real;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cmd_Position_Publisher>());
    rclcpp::shutdown();
    return 0;
}
