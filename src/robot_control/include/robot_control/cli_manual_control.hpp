
#ifndef CLI_MANUAL_CONTROL_HPP 
#define CLI_MANUAL_CONTROL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>

#include <condition_variable>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>

// #include <iostream>
// #include <fcntl.h>     
// #include <unistd.h>     
// #include <termios.h>    
// #include <cstring>  

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_control/serial_communication.hpp"

using namespace std::chrono_literals;

constexpr char KILL_COMMAND[] = "kill";

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

// RobotMovement parse_command(const std::string &input);

class ManualControllerPublisher : public rclcpp::Node
{
    public:
        ManualControllerPublisher();
        ~ManualControllerPublisher();
        
        void publishUserCommand();
        void waitForUser();
        void displayControlGuide();

    private:
        SerialCommunication m_serial_;
        std::string m_user_input; 

        rclcpp::TimerBase::SharedPtr m_timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher_;

        std::thread m_input_thread_;
        std::atomic<bool> m_running_;
        std::queue<std::string> m_command_queue_;
        std::mutex m_queue_mutex_;
        std::condition_variable m_queue_cv_;
};


#endif