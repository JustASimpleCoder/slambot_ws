#include "autonomous_control.hpp"

AutonomousControllerPublisher::AutonomousControllerPublisher() : 
                    Node("Autonomous_control"),
                    m_serial_("/dev/ttyUSB0", 9600), 
                    m_inv_kin(5,5,5) 
{
    m_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&AutonomousControllerPublisher::updateMotorControl,  this, std::placeholders::_1)
        // [this](geometry_msgs::msg::Twist new_twist) 
        // {
        //     // Pass the received twist to the motor control function
        //     this->updateMotorControl(new_twist);
        // }
    );

    if (!m_serial_.open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        throw std::runtime_error("Failed to open serial port");
    }
};

AutonomousControllerPublisher::~AutonomousControllerPublisher(){};

void AutonomousControllerPublisher::updateMotorControl(const geometry_msgs::msg::Twist & twist_msg){

    m_inv_kin.updateDesiredSpeed(twist_msg);
    m_inv_kin.convertToPWMSignal();
    m_arduino_msg =  m_inv_kin.getArduinoUnoMsg();
    if (m_serial_.write(m_arduino_msg) < 0) {
        std::cout << "Failed to send cmd_vel to arduino uno \n" << std::endl;
    }
}
