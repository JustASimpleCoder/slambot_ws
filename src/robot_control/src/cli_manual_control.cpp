#include "cli_manual_control.hpp"
//#include "robot_control/m_serial_communication.hpp"

using namespace std::chrono_literals;

ManualControllerPublisher::ManualControllerPublisher()
    : Node("motor_controller"), 
    m_serial_("/dev/ttyUSB0", 9600)
{

    if (!m_serial_.open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        throw std::runtime_error("Failed to open serial port");
    }


    rclcpp::QoS qos_settings(1);  // Depth of 1 for minimal latency
    qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_settings.durability(rclcpp::DurabilityPolicy::Volatile);

    m_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", qos_settings);
    m_timer_ = this->create_wall_timer(
        10ms, std::bind(&ManualControllerPublisher::publish_user_command, this));
        m_input_thread_ = std::thread(&ManualControllerPublisher::wait_for_user, this);

}

ManualControllerPublisher::~ManualControllerPublisher() {
    if (m_input_thread_.joinable()) {
        m_input_thread_.join();
    }
}


RobotMovement parse_command(const std::string &input) {
    auto it = command_map.find(input);
    return (it != command_map.end()) ? it->second : RobotMovement::INVALID;
}

void ManualControllerPublisher::publish_user_command()
{
    std::unique_lock<std::mutex> lock(m_queue_mutex_);
    if (m_queue_cv_.wait_for(lock, 10ms, [this] { return !m_command_queue_.empty(); })) {
        // Publish all commands in the queue
        while (!m_command_queue_.empty()) {
            std::string command = m_command_queue_.front();

            m_command_queue_.pop();
            if (m_serial_.write(command + "\n") < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send command over serial: %s", command.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent command over serial: %s", command.c_str());
            }

            auto message = std_msgs::msg::String();
            message.data = command;

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            m_publisher_->publish(message);
        }
    }
}

void ManualControllerPublisher::wait_for_user(){
    while (m_running_) {
        std::string user_input;
        std::cin >> user_input;

        if (user_input == "kill") {
            m_running_ = false;
            rclcpp::shutdown();
            auto error_message = std_msgs::msg::String();
            error_message.data = "Shutting down the node...";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", error_message.data.c_str());
            break;
        }

        // Add valid input to the queue
        auto it = command_map.find(user_input);
        if (it != command_map.end()) {
            std::lock_guard<std::mutex> lock(m_queue_mutex_);
            m_command_queue_.push(user_input);
            m_queue_cv_.notify_one(); // Notify the publisher thread
        } else {
            std::cout << "Invalid input. Press w (UP), s (Down), a (Left), d (right) or x (stop)\n";
        }
    }
}



