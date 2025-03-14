#ifndef AUTONOMOUS_CONTROL_HPP
#define AUTONOMOUS_CONTROL_HPP

#include <chrono>
// #include <functional>
#include <string>
#include <iostream>

// #include <unordered_map>
// #include <memory>
// #include <condition_variable>
// #include <thread>
// #include <atomic>
// #include <queue>
// #include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include "serial_communication.hpp"
#include "inverse_kinematics.hpp"




using namespace std::chrono_literals;

class AutonomousControllerPublisher : public rclcpp::Node
{
    public:
        AutonomousControllerPublisher();
        ~AutonomousControllerPublisher();
        
        // void convertTwistToArduinoMSG();
        void updateMotorControl(const geometry_msgs::msg::Twist & twist_msg);

    private:
        SerialCommunication m_serial_;
        InverseKinematics m_inv_kin;
        std::string m_arduino_msg;

        rclcpp::TimerBase::SharedPtr m_timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscriber_;

        // std::thread m_input_thread_;
        // std::atomic<bool> m_running_{true};
        // std::queue<std::string> m_command_queue_;
        // std::mutex m_queue_mutex_;
        // std::condition_variable m_queue_cv_;
};

#endif