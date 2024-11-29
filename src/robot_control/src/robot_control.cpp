#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

enum class RobotCommand {
    STOP = 0,
    MOVE_FORWARD = 1,
    MOVE_BACKWARD = 2,
    TURN_LEFT = 3, //may use in future builds
    TURN_RIGHT = 4,
    TURN_LEFT_OPP = 5, 
    TURN_RIGHT_OPP = 6, 
    INVALID = -1
};

static const std::unordered_map<std::string, RobotCommand> command_map = {
    {"x", RobotCommand::STOP},
    {"w", RobotCommand::MOVE_FORWARD},
    {"s", RobotCommand::MOVE_BACKWARD},
    {"a", RobotCommand::TURN_LEFT_OPP},
    {"d", RobotCommand::TURN_RIGHT_OPP}
};

const std::unordered_map<std::string, std::string> command_descriptions = {
    {"x", "Stop the robot"},
    {"w", "Move forward"},
    {"s", "Move backward"},
    {"a", "Turn left"},
    {"d", "Turn right"}
};
// Convert user input to a RobotCommand
RobotCommand parse_command(const std::string &input) {
    auto it = command_map.find(input);
    return (it != command_map.end()) ? it->second : RobotCommand::INVALID;
}

class MotorControllerPublisher : public rclcpp::Node
{
  public:
    MotorControllerPublisher()
    : Node("motor_controller")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorControllerPublisher::wait_for_user_command_and_publish, this));
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
            RCLCPP_INFO(this->get_logger(), "Shutting down the node...");
            rclcpp::shutdown();
            return; 
            
            auto error_message = std_msgs::msg::String();
            error_message.data = "Shutting down the node...";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", error_message.data.c_str());

        }
        
        message.data = valid_user_input;
        std::string log_message = "Publishing: '" + command_descriptions.at(valid_user_input);
        RCLCPP_INFO(this->get_logger(), "%s" , log_message.c_str());
        this->publisher_->publish(message);
    }

    std::string wait_for_user(){
        while (true) {
            std::cout << "\nPress w (UP), s (Down), a (Left), d (right) or x (stop). Enter 'kill' to shutdown node \n";
            std::cin >> user_input;
            std::cout << "You entered " << user_input << "\n";

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
