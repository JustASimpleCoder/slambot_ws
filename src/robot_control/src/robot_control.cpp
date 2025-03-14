
#include "cli_manual_control.hpp"
//#include "serial_communication.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControllerPublisher>());
    rclcpp::shutdown();
    return 0;
}

