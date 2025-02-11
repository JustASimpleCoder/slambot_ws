
#include "robot_control/controller_publisher.hpp"
//#include "serial_communication.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControllerPublisher>());
    rclcpp::shutdown();
    return 0;
}

