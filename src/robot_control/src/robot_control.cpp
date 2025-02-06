#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

enum class RobotMovement: char{
    STOP = 'x',
    MOVE_FORWARD = 'w',
    MOVE_BACKWARD = 's',
    MOVE_LEFT = 'l',
    MOVE_RIGHT = 'r',
    ROTATE_LEFT = 'a',
    ROTATE_RIGHT = 'd',
    DIAG_FORWARD_RIGHT = 'e',
    DIAG_BACKWARD_RIGHT = 'c',
    DIAG_FORWARD_LEFT = 'q',
    DIAG_BACKWARD_LEFT = 'z',
    FASTER = '+',
    SLOWER = '-',
    INVALID = '?'
};
static const std::unordered_map<std::string, RobotMovement> command_map = {
    {"x", RobotMovement::STOP},
    {"w", RobotMovement::MOVE_FORWARD},
    {"s", RobotMovement::MOVE_BACKWARD},
    {"a", RobotMovement::ROTATE_LEFT},
    {"d", RobotMovement::ROTATE_RIGHT},
    {"l", RobotMovement::MOVE_LEFT},
    {"r", RobotMovement::MOVE_RIGHT},
    {"q", RobotMovement::DIAG_FORWARD_LEFT},
    {"e", RobotMovement::DIAG_FORWARD_RIGHT},
    {"z", RobotMovement::DIAG_BACKWARD_LEFT},
    {"c", RobotMovement::DIAG_BACKWARD_RIGHT},
    {"=", RobotMovement::FASTER}, // no need to capitalize '+' 
    {"+", RobotMovement::FASTER},
    {"-", RobotMovement::SLOWER}
};

const std::unordered_map<std::string, std::string> command_descriptions = {
    {"x", "Stop the robot"},
    {"w", "Move Forward"},
    {"s", "Move Backward"},
    {"a", "Turn Left"},
    {"d", "Turn Right"},
    {"l", "Move Left"},
    {"r", "Move Right"},
    {"q", "Diagnoal Forward Left"},
    {"e", "Diagnoal Forward Right"},
    {"z", "Diagnoal Backward Left"},
    {"c", "Diagnoal Backward Right"},
    {"=", "Faster"}, // no need to capitalize '+' 
    {"+", "Faster"},
    {"-", "Slower"}
};

RobotMovement parse_command(const std::string &input) {
    auto it = command_map.find(input);
    return (it != command_map.end()) ? it->second : RobotMovement::INVALID;
}

class MotorControllerPublisher : public rclcpp::Node
{
  public:
    MotorControllerPublisher()
    : Node("motor_controller")
    {
        rclcpp::QoS qos_settings(1);  // Depth of 1 for minimal latency
        qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_settings.durability(rclcpp::DurabilityPolicy::Volatile);

        publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", qos_settings);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MotorControllerPublisher::wait_for_user_command_and_publish, this));
    }

  private:
    std::string user_input; 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


    void wait_for_user_command_and_publish()
    {
        std::string valid_user_input = wait_for_user();
        auto message = std_msgs::msg::String();

        if(valid_user_input == "kill"){
            // RCLCPP_INFO(this->get_logger(), "Shutting down the node...");
            rclcpp::shutdown();
            return; 
            
            auto error_message = std_msgs::msg::String();
            error_message.data = "Shutting down the node...";
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", error_message.data.c_str());

        }

        message.data = valid_user_input;
        std::string log_message = "Publishing: '" + command_descriptions.at(valid_user_input);
        RCLCPP_INFO(this->get_logger(), "%s" , log_message.c_str());
        this->publisher_->publish(message);
    }

    std::string wait_for_user(){
        while (true) {
            // std::cout << "\nPress w (UP), s (Down), a (Left), d (right) or x (stop). Enter 'kill' to shutdown node \n";
            std::cin >> user_input;
            // std::cout << "You entered " << user_input << "\n";

            auto it = command_map.find(user_input);
            if (it != command_map.end() || user_input == "kill") {
                return user_input; // Return the valid input
            }
            std::cout << "Press w (UP), s (Down), a (Left), d (right) or x (stop)\n";
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControllerPublisher>());
  rclcpp::shutdown();
  return 0;
}
