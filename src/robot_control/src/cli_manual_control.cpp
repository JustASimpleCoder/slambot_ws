#include "cli_manual_control.hpp"
//#include "robot_control/m_serial_communication.hpp"

using namespace std::chrono_literals;

ManualControllerPublisher::ManualControllerPublisher()
    : Node("motor_controller"),
    m_serial_("/dev/ttyUSB0", 9600)
{
    // declare_parameter("usb_port", "/dev/ttyUSB0");
    // std::string usb_port = get_parameter("usb_port").as_string();
    // m_serial_ = SerialPort(usb_port, 9600);  // Use ROS parameter

    // if (!m_serial_.open()) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
    //     // throw std::runtime_error("Failed to open serial port");
    // }
    m_running_ = true;
    rclcpp::QoS qos_settings(1);  // Depth of 1 for minimal latency
    qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_settings.durability(rclcpp::DurabilityPolicy::Volatile);

    m_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", qos_settings);
    m_timer_ = this->create_wall_timer(
        10ms, std::bind(&ManualControllerPublisher::publishUserCommand, this));
        m_input_thread_ = std::thread(&ManualControllerPublisher::waitForUser, this);

    displayControlGuide();
}

ManualControllerPublisher::~ManualControllerPublisher() {
    m_running_ = false;
    m_queue_cv_.notify_all(); 

    if (m_input_thread_.joinable()) {
        m_input_thread_.join();
    }
}


RobotMovement parse_command(const std::string &input) {
    auto it = command_map.find(input);
    return (it != command_map.end()) ? it->second : RobotMovement::INVALID;
}

void ManualControllerPublisher::publishUserCommand()
{
    std::unique_lock<std::mutex> lock(m_queue_mutex_);
   if (m_queue_cv_.wait_for(lock, 10ms, [this] { return !m_command_queue_.empty(); })) {
        // Publish all commands in the queue
        while (!m_command_queue_.empty()) {
            std::string command = m_command_queue_.front();

            m_command_queue_.pop();
            // if (m_serial_.write(command + "\n") < 0) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to send command over serial: %s", command.c_str());
            // } else {
            //     RCLCPP_INFO(this->get_logger(), "Sent command over serial: %s", command.c_str());
            // }

            auto message = std_msgs::msg::String();
            message.data = command;

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            m_publisher_->publish(message);
        }
    }

}

void ManualControllerPublisher::waitForUser(){
    while (m_running_) {
        std::string user_input;
        std::cin >> user_input;

        if (user_input == KILL_COMMAND) {
            m_running_ = false;
            m_queue_cv_.notify_all();
            auto shutdown_message = std_msgs::msg::String();
            shutdown_message.data = "Shutting down the node...";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", shutdown_message.data.c_str());
            rclcpp::shutdown();
        
            break;
        }
        // Add valid input to the queue
        auto it = command_map.find(user_input);
        std::lock_guard<std::mutex> lock(m_queue_mutex_);
        if (it != command_map.end()) {
            m_command_queue_.push(user_input);
            m_queue_cv_.notify_all(); // Notify the publisher thread
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid Input, you entered [%s]. Please refer to the control guide for valid inputs: ", user_input.c_str());
            displayControlGuide();
        }
    }

}




void ManualControllerPublisher::displayControlGuide(){
    std::cout << "\n                     ";
    std::cout << "🔹 **Control Guide** 🔹\n\n";

    std::cout << "  ↖  q  - Move Diagonally Left-Up   | ";
    std::cout << "  ↗  e  - Move Diagonally Right-Up    \n\n";
    
    std::cout << "                     ";
    std::cout << "  ↑  w  - Move Up    \n\n";

    std::cout << "  ←  a  - Move Left |";
    std::cout << "  ↓  s  - Move Down |";
    std::cout << "  →  d  - Move Right\n\n\n";
    
    std::cout << "  ↙  z  - Move Diagonally Left-Down | ";
    std::cout << "  ↘  c  - Move Diagonally Right-Down  \n\n\n";
    std::cout << "                     ";
    std::cout << "  ⏹  x  - Stop Moving\n";
    std::cout << "                     ";
    std::cout << "  ❌  KILL - Shutdown Node and Exit Terminal\n";

    std::cout << R"(
             
                         ↖      ↑     ↗
                           (Q) (w) (E)

                         ← (a) (s) (d) → 
                                ↓
           
                           (z) (x) (c)  
                         ↙      ⏹      ↘
                               STOP
  )";
}

