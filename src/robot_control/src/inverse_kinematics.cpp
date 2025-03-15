#include "inverse_kinematics.hpp"

#include <chrono>

InverseKinematics::InverseKinematics(int wheel_radus, int horizontal_dist_to_wheel, int vertical_dist_to_wheel )                
                                :
                                m_r(wheel_radus), m_a(horizontal_dist_to_wheel), m_b(vertical_dist_to_wheel), 
                                m_pwm_lf(0), m_pwm_rf(0), m_pwm_lb(0), m_pwm_rb(0)
                                
{


};

InverseKinematics::~InverseKinematics(){
};

void InverseKinematics::updateDesiredSpeed(double v_x, double v_y, double omega){
    m_omega_lf = (1 / (m_r)) * (v_x - v_y - (m_a + m_b) * omega);
    m_omega_rf = (1 / (m_r)) * (v_x + v_y + (m_a + m_b) * omega);
    m_omega_lb = (1 / (m_r)) * (v_x + v_y - (m_a + m_b) * omega);
    m_omega_rb = (1 / (m_r)) * (v_x - v_y + (m_a + m_b) * omega);
};

void InverseKinematics::updateDesiredSpeed( const geometry_msgs::msg::Twist & twist ){
    updateDesiredSpeed(twist.linear.x, twist.linear.y, twist.angular.z);
};

enum Directions{
    BACkWARD,
    FORWARD
};

void InverseKinematics::convertToPWMSignal(){
    // int RPM = 200; // min RPM at 6v
    // int n = 48; // gear ratio

    m_pwm_lf = computePWM(m_omega_lf);
    m_pwm_rf = computePWM(m_omega_rf);
    m_pwm_lb = computePWM(m_omega_lb);
    m_pwm_rb = computePWM(m_omega_rb);

    m_dir_lf = (m_omega_lf >= 0) ? Directions::FORWARD : Directions::BACkWARD;
    m_dir_rf = (m_omega_rf >= 0) ? Directions::FORWARD : Directions::BACkWARD;
    m_dir_lb = (m_omega_lb >= 0) ? Directions::FORWARD : Directions::BACkWARD;
    m_dir_rb = (m_omega_rb >= 0) ? Directions::FORWARD : Directions::BACkWARD;

    std::string arduino_msg = "";
    arduino_msg += msgConstructionHelper(m_pwm_lf, m_dir_lf, false);
    arduino_msg += msgConstructionHelper(m_pwm_rf, m_dir_rf, false);
    arduino_msg += msgConstructionHelper(m_pwm_lb, m_dir_lb, false);
    arduino_msg += msgConstructionHelper(m_pwm_rb, m_dir_rb, true);
    m_arduino_uno_msg = arduino_msg;
};

std::string InverseKinematics::getArduinoUnoMsg(){
    return m_arduino_uno_msg;
};

//comunication protocol "pwm, dir, pwm, dir, pwm, dir, pwm, dir\r\n"
std::string InverseKinematics::msgConstructionHelper(int pwm, int dir, bool is_last){
    return is_last ? (std::to_string(pwm) + "," + std::to_string(dir) + "\r\n") : (std::to_string(pwm) + "," + std::to_string(dir) + ",");
};

uint8_t InverseKinematics::computePWM(double omega ){
    if (std::abs(omega) < 1e-3) return 0; // Dead zone to prevent jitter

    double wheel_rpm = (omega * 60.0) / (2.0 * M_PI);
    double motor_rpm = wheel_rpm * m_gear_ratio;
    motor_rpm = std::fabs(motor_rpm);
    motor_rpm = (motor_rpm > m_max_rpm) ? m_max_rpm : motor_rpm;

    int pwm = static_cast<int>((motor_rpm / m_max_rpm) * (m_max_pwm - m_min_pwm) + m_min_pwm + 0.5);
    std::clamp(pwm, 0, 255);
    return pwm;
};