#ifndef INVERSE_KINEMATICS_TEST_HPP
#define INVERSE_KINEMATICS_TEST_HPP


#include <memory>
#include <cmath>
#include <algorithm>
#include <string>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class InverseKinematics : public rclcpp::Node {
    public:
        InverseKinematics(int wheel_radus, int horizontal_dist_to_wheel, int vertical_dist_to_wheel );
        ~InverseKinematics();

        void updateDesiredSpeed(double v_x, double v_y, double omega);
        void updateDesiredSpeed( const geometry_msgs::msg::Twist & twist );

        void convertToPWMSignal();
        uint8_t computePWM(double omega);
        
        std::string getArduinoUnoMsg();
        std::string msgConstructionHelper(int pwm, int dir, bool is_last);

        void publish_message();

        // m_publisher_ = this->rclcpp::Publisher<std_msgs::msg::String>("arduino_testing", 10);

    private:
        double m_omega_LF, m_omega_RF, m_omega_LB, m_omega_RB;
        double m_r, m_a, m_b;
        int m_pwm_lf, m_pwm_rf, m_pwm_lb, m_pwm_rb;
        int m_dir_lf, m_dir_rf, m_dir_lb, m_dir_rb;

        const double m_max_rpm = 200.0; // From datasheet
        const int m_gear_ratio = 48;
        const int m_min_pwm = 125;      // Minimum PWM to start motion
        const int m_max_pwm = 255;
        std::string m_arduino_uno_msg = "";

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher_;
        rclcpp::TimerBase::SharedPtr m_timer_;
};


#endif