#include "robot_control/controller_publisher.hpp"
//#include "robot_control/serial_communication.hpp"

using namespace std::chrono_literals;

MotorControllerPublisher::MotorControllerPublisher()
    : Node("motor_controller"), 
    serial_("/dev/ttyUSB0", 9600)
{

    if (!serial_.open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        throw std::runtime_error("Failed to open serial port");
    }


    rclcpp::QoS qos_settings(1);  // Depth of 1 for minimal latency
    qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_settings.durability(rclcpp::DurabilityPolicy::Volatile);

    publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", qos_settings);
    timer_ = this->create_wall_timer(
        10ms, std::bind(&MotorControllerPublisher::publish_user_command, this));
    input_thread_ = std::thread(&MotorControllerPublisher::wait_for_user, this);

}

MotorControllerPublisher::~MotorControllerPublisher() {
    if (input_thread_.joinable()) {
        input_thread_.join();
    }
}


RobotMovement parse_command(const std::string &input) {
    auto it = command_map.find(input);
    return (it != command_map.end()) ? it->second : RobotMovement::INVALID;
}

void MotorControllerPublisher::publish_user_command()
{
    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (queue_cv_.wait_for(lock, 10ms, [this] { return !command_queue_.empty(); })) {
        // Publish all commands in the queue
        while (!command_queue_.empty()) {
            std::string command = command_queue_.front();

            command_queue_.pop();
            if (serial_.write(command + "\n") < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send command over serial: %s", command.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent command over serial: %s", command.c_str());
            }

            auto message = std_msgs::msg::String();
            message.data = command;

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
    }
}

void MotorControllerPublisher::wait_for_user(){
    while (running_) {
        std::string user_input;
        std::cin >> user_input;

        if (user_input == "kill") {
            running_ = false;
            rclcpp::shutdown();
            auto error_message = std_msgs::msg::String();
            error_message.data = "Shutting down the node...";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", error_message.data.c_str());
            break;
        }

        // Add valid input to the queue
        auto it = command_map.find(user_input);
        if (it != command_map.end()) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            command_queue_.push(user_input);
            queue_cv_.notify_one(); // Notify the publisher thread
        } else {
            std::cout << "Invalid input. Press w (UP), s (Down), a (Left), d (right) or x (stop)\n";
        }
    }
}



