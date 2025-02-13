#include <cmath>
#include <algorithm>
#include <string>

class InverseKinematics{
    public:
        InverseKinematics();
        ~InverseKinematics();

        void updateDesiredSpeed(double v_x, double v_y, double omega);
        void convertToPWMSignal();
        auto computePWM(double omega);
    private:
        double m_omega_LF, m_omega_RF, m_omega_LB, m_omega_RB;
        double m_r, m_a, m_b;
        int pwm_lf, pwm_rf, pwm_lb, pwm_rb;
        int dir_lf, dir_rf, dir_lb, dir_rb;

        const double max_rpm = 200.0; // From datasheet
        const int gear_ratio = 48;
        const int min_pwm = 125;      // Minimum PWM to start motion
        const int max_pwm = 255;

};


InverseKinematics::InverseKinematics(){

}
InverseKinematics::~InverseKinematics(){

}


void InverseKinematics::updateDesiredSpeed(double v_x, double v_y, double omega){

    m_omega_LF = (1 / (m_r)) * (v_x - v_y - (m_a + m_b) * omega);
    m_omega_RF = (1 / (m_r)) * (v_x + v_y + (m_a + m_b) * omega);
    m_omega_LB = (1 / (m_r)) * (v_x + v_y - (m_a + m_b) * omega);
    m_omega_RB = (1 / (m_r)) * (v_x - v_y + (m_a + m_b) * omega);

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

    //TO_DO create an approiated schem for sending this data to arduino uno
    // send this to arduino uno
    // std::string arduino_msg = "";
    // arduino_msg += msg_construction_helper(pwm_lf, dir_lf, "LF");
    // arduino_msg += msg_construction_helper(pwm_rf, dir_rf, "RF");
    // arduino_msg += msg_construction_helper(pwm_lb, dir_lb, "LB");
    // arduino_msg += msg_construction_helper(pwm_rb, dir_rb, "RB");

    // publish message 
}
std::string msg_construction_helper(int pwm, int dir, std::string motor){
    return "PWM_" + motor + "<" + std::to_string(pwm) + ">" + "DIR_" + motor + "<" + std::to_string(pwm) + ">";
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