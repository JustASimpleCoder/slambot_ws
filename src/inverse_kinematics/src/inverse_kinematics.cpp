#include <cmath>
#include <algorithm>
#include <string>

#include "geometry_msgs/msg/Twist.hpp"

class InverseKinematics{
    public:
        InverseKinematics();
        ~InverseKinematics();

        void updateDesiredSpeed(double v_x, double v_y, double omega);
        void convertToPWMSignal();
        auto computePWM(double omega);
        
        std::string get_arduino_uno_msg();

    private:
        double m_omega_LF, m_omega_RF, m_omega_LB, m_omega_RB;
        double m_r, m_a, m_b;
        int pwm_lf, pwm_rf, pwm_lb, pwm_rb;
        int dir_lf, dir_rf, dir_lb, dir_rb;

        const double max_rpm = 200.0; // From datasheet
        const int gear_ratio = 48;
        const int min_pwm = 125;      // Minimum PWM to start motion
        const int max_pwm = 255;

        std::string arduino_uno_msg = "";
};


InverseKinematics::InverseKinematics(int wheel_radus, int horizontal_dist_to_wheel, int vertical_dist_to_wheel )                
                                :   m_r(wheel_radus), m_a(horizontal_dist_to_wheel), m_b(vertical_dist_to_wheel) {
}

InverseKinematics::~InverseKinematics(){
}

void InverseKinematics::updateDesiredSpeed(double v_x, double v_y, double omega){
    m_omega_LF = (1 / (m_r)) * (v_x - v_y - (m_a + m_b) * omega);
    m_omega_RF = (1 / (m_r)) * (v_x + v_y + (m_a + m_b) * omega);
    m_omega_LB = (1 / (m_r)) * (v_x + v_y - (m_a + m_b) * omega);
    m_omega_RB = (1 / (m_r)) * (v_x - v_y + (m_a + m_b) * omega);
}

void InverseKinematics::updateDesiredSpeed(geometry_msgs::msg::Twist twist){
    // m_omega_LF = (1 / (m_r)) * (twist.data.linear.x - twist.data.linear.y - (m_a + m_b) * twist.data.linear.z);
    // m_omega_RF = (1 / (m_r)) * (twist.data.linear.x + twist.data.linear.y + (m_a + m_b) * twist.data.linear.z);
    // m_omega_LB = (1 / (m_r)) * (twist.data.linear.x + twist.data.linear.y - (m_a + m_b) * twist.data.linear.z);
    // m_omega_RB = (1 / (m_r)) * (twist.data.linear.x - twist.data.linear.y + (m_a + m_b) * twist.data.linear.z);
    
    updateDesiredSpeed(twist.data.linear.x, twist.data.linear.y, twist.data.angular.z);
}

enum Directions{
    BACkWARD,
    FORWARD
};

void InverseKinematics::convertToPWMSignal(){
    int RPM = 200; // min RPM at 6v
    int n = 48; // gear ratio

    pwm_lf = computePWM(m_omega_LF);
    pwm_rf = computePWM(m_omega_RF);
    pwm_lb = computePWM(m_omega_LB);
    pwm_rb = computePWM(m_omega_RB);

    dir_lf = (m_omega_LF >= 0) ? Directions::FORWARD : Directions::BACkWARD;
    dir_rf = (m_omega_RF >= 0) ? Directions::FORWARD : Directions::BACkWARD;
    dir_lb = (m_omega_LB >= 0) ? Directions::FORWARD : Directions::BACkWARD;
    dir_rb = (m_omega_RB >= 0) ? Directions::FORWARD : Directions::BACkWARD;

    std::string arduino_msg = "";
    arduino_msg += msg_construction_helper(pwm_lf, dir_lf, false);
    arduino_msg += msg_construction_helper(pwm_rf, dir_rf, false);
    arduino_msg += msg_construction_helper(pwm_lb, dir_lb, false);
    arduino_msg += msg_construction_helper(pwm_rb, dir_rb, true);
    arduino_uno_msg = arduino_msg;
}

std::string InverseKinematics::get_arduino_uno_msg(){
    return arduino_uno_msg;
}

//comunication protocol "pwm, dir, pwm, dir, pwm, dir, pwm, dir\r\n"
std::string msg_construction_helper(int pwm, int dir, bool is_last){
    return is_last ? (std::to_string(pwm) + "," std::to_string(dir) + "\r\n") : (std::to_string(pwm) + "," std::to_string(dir) + ",");
}

auto InverseKinematics::computePWM(double omega ){
    if (std::abs(omega) < 1e-3) return 0; // Dead zone to prevent jitter

    double wheel_rpm = (omega * 60.0) / (2.0 * M_PI);
    double motor_rpm = wheel_rpm * gear_ratio;
    motor_rpm = std::fabs(motor_rpm);
    motor_rpm = (motor_rpm > max_rpm) ? max_rpm : motor_rpm;

    int pwm = static_cast<int>((motor_rpm / max_rpm) * (max_pwm - min_pwm) + min_pwm + 0.5);
    return std::clamp(pwm, 0, 255);
}


// struct VelCmdValues
// {
//     uint8_t pwm_values[4] = {0,0,0,0};
//     Direction dir_values[4] = {FORWARD,FORWARD,FORWARD,FORWARD};
// };


