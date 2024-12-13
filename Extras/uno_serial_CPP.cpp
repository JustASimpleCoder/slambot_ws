#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>  // Use the serial library for C++
#include <memory>
#include <string>
//Placeholder file for now

// TODO: move this to ros node to handle these odom calcs and make it work with optical odemetry  
//     double robot_x_front = 0.0; 
//     double robot_x_back = 0.0; 

//     double robot_y_front = 0.0; 
//     double robot_y_back = 0.0; 

//     double robot_theta_front = 0.0; 
//     double robot_theta_back = 0.0; 

//     // Estimate the time of travel for this cycle (in seconds)
//     unsigned long timeInterval = 100; // Time interval in milliseconds
//     double timeInSeconds = timeInterval / 1000.0;

//     // Estimate wheel speeds based on PWM
//     double right_front_speed = map(right.pwm_front_pin , 0, 255, 0, 1); 
//     double right_back_speed = map(right.pwm_back_pin , 0, 255, 0, 1);
//     double right_front_speed = map(left.pwm_front_pin , 0, 255, 0, 1); 
//     double right_back_speed = map(left.pwm_back_pin , 0, 255, 0, 1);

  
//     // Calculate the linear distance traveled by each wheel
//     double left_front_dist = right_front_speed * wheel_circum * timeInSeconds;
//     double left_back_dist = right_front_speed * wheel_circum * timeInSeconds;
//     double right_front_dist = right_front_speed * wheel_circum * timeInSeconds;
//     double right_back_dist = right_front_speed * wheel_circum * timeInSeconds;

//     // Calculate robot movement
//     double delta_front_dist = (left_front_dist + right_front_dist) / 2.0;
//     double delta_back_dist = (left_back_dist + right_back_dist) / 2.0;

//     double delta_front_theta = (right_front_dist - left_front_dist) / wheel_base;
//     double delta_back_theta = (right_back_dist - left_back_dist) / wheel_base;
//     // Update robot position and orientation
//     robot_x_front += delta_front_dist * cos(delta_front_theta);
//     robot_x_back += delta_back_dist * cos(delta_back_theta);

//     robot_y_front += delta_front_dist * sin(delta_front_theta);
//     robot_y_back += delta_back_dist * sin(delta_back_theta);


//     robot_theta_front += delta_front_theta; // Update orientation
//     robot_theta_back += delta_back_theta;
//     // Normalize angle (keep between -PI and PI)
//     if (robot_theta_front > PI) robot_theta_front -= 2 * PI;
//     if (robot_theta_front < -PI) robot_theta_front += 2 * PI;

//     if (robot_theta_back > PI) robot_theta_back -= 2 * PI;
//     if (robot_theta_back < -PI) robot_theta_back += 2 * PI;

//   // Print the robot's current position and orientation
//     Serial.print("FrontX: "); Serial.print(robot_x_front); 
//     Serial.print(", FrontY: "); Serial.print(robot_y_front);
//     Serial.print(", FrontTheta: "); Serial.println(robot_theta_front);

//     Serial.print("BackX: "); Serial.print(robot_x_back); 
//     Serial.print(", BackY: "); Serial.print(robot_y_back);
//     Serial.print(", BackTheta: "); Serial.println(robot_theta_back);

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
