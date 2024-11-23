#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MotorControllerPublisher : public rclcpp::Node
{
  public:
    MotorControllerPublisher()
    : Node("motor_controller")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", 10);
        timer_ = this->create_wall_timer(
        1000ms, std::bind(&MotorControllerPublisher::timer_callback, this));
    }

  private:
    void wait_for_user_command_and_publish()
    {
        std::string user_input = wait_for_user();
        auto message = std_msgs::msg::String();
        message.data = wait_for_user();
        if(user_input ==0 || user_input == 1 || user_input == 1 ){
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            this->publisher_->publish(message);
        }else{
             auto error_message = std_msgs::msg::String();
             error_message.data = "invalid command, try again";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", error_message.data .c_str());
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    std::string wait_for_user(){
        std::string input;
        while (true) {
            std::cout << "\nPress 0 to stop the robot, 1 to move forward 5 seconds, or 2 to move backward 5 seconds: ";
            std::getline(std::cin, input);
            if (input == "0" || input == "1" || input == "2") {
                return input; // Return the valid input
            } else {
                std::cout << "Invalid input. Please enter 0, 1, or 2.";
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