#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>  // Use the serial library for C++
#include <memory>
#include <string>

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        // Initialize the serial connection
        try {
            serial_.setPort("/dev/ttyUSB0");  // Update the port as needed
            serial_.setBaudrate(9600);
            serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // Publisher for odometry data
        odom_publisher_ = this->create_publisher<std_msgs::msg::String>("odometry", 10);

        // Subscriber for motor control commands
        motor_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "motor_control",
            10,
            std::bind(&SerialNode::motorCallback, this, std::placeholders::_1));

        // Timer for reading odometry data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SerialNode::readOdometry, this));

        RCLCPP_INFO(this->get_logger(), "SerialNode initialized and listening on /motor_control");
    }

    ~SerialNode()
    {
        if (serial_.isOpen()) {
            serial_.close();
        }
        RCLCPP_INFO(this->get_logger(), "Serial port closed.");
    }

private:
    void motorCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            const std::string command = msg->data + "\n";
            serial_.write(command);
            RCLCPP_INFO(this->get_logger(), "Sent command to Arduino: %s", command.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command to Arduino: %s", e.what());
        }
    }

    void readOdometry()
    {
        try {
            if (serial_.available() > 0) {
                const std::string data = serial_.readline();
                if (!data.empty()) {
                    auto msg = std_msgs::msg::String();
                    msg.data = data;
                    odom_publisher_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Published odometry data: %s", data.c_str());
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read odometry data: %s", e.what());
        }
    }

    serial::Serial serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odom_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
