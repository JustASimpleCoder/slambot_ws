
#include "inverse_kinematics.hpp"
#include <iostream>

int main(int argc,char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematics>(5, 5, 5));
    rclcpp::shutdown();
}