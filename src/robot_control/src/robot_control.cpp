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
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
    INVALID = -1
};

static const std::unordered_map<std::string, RobotCommand> command_map = {
    {"0", RobotCommand::STOP},
    {"1", RobotCommand::MOVE_FORWARD},
    {"2", RobotCommand::MOVE_BACKWARD},
    {"3", RobotCommand::TURN_LEFT},
    {"4", RobotCommand::TURN_RIGHT}
};

const std::unordered_map<RobotCommand, std::string> command_descriptions = {
    {RobotCommand::STOP, "Stop the robot"},
    {RobotCommand::MOVE_FORWARD, "Move forward for 5 seconds"},
    {RobotCommand::MOVE_BACKWARD, "Move backward for 5 seconds"},
    {RobotCommand::TURN_LEFT, "Turn left for 5 seconds"},
    {RobotCommand::TURN_RIGHT, "Turn right for 5 seconds"}
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
    void wait_for_user_command_and_publish()
    {
        std::string user_input = wait_for_user();
        auto message = std_msgs::msg::String();
        message.data = user_input;

        RobotCommand command = parse_command(user_input);

        if(command != RobotCommand::INVALID ){
            std::string log_message = "Publishing: '" + command_descriptions.at(command);
            RCLCPP_INFO(this->get_logger(), "%s" , log_message.c_str());
            this->publisher_->publish(message);
        }else{
            if (user_input == "kill") {
                RCLCPP_INFO(this->get_logger(), "Shutting down the node...");
                rclcpp::shutdown();
                return; 
            }
            
            auto error_message = std_msgs::msg::String();
            error_message.data = "invalid command, try again";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", error_message.data.c_str());
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    std::string wait_for_user(){
        std::string input;
        while (true) {
            std::cout << "\nPress 0 to stop the robot, 1 to move forwartd, 2 to move backward, 3 for right turn or 4 for lewft turn ";
            std::getline(std::cin, input);

            auto it = command_map.find(input);
            if (it != command_map.end()) {
                return input; // Return the valid input
            } else {
                std::cout << "Invalid input! Please enter 0, 1, 2, 3 or 4";
            }
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