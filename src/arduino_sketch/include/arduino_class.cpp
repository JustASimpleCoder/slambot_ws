#include "arduino_class.hpp"
// uncomment during delopment as having troulbe importing arduino.h class on WSL. 

//ensure to comment out below before uploading sketch Un
void analogWrite(int a, int b){};
void digitalWrite(int a, int b){};
void pinMode(int a, int b){};

class SerialArduino{
    public: 
        void begin(int a){};
        int available(){};
        int parseInt(){};
        void print(std::string);
        void println(std::string );
        std::string read();
};

SerialArduino Serial;

#define OUTPUT 1
#define HIGH 1
#define LOW 0

//Ensure to comment out above before uploading sketch


void Motor::setSpeed(int speed) {
    analogWrite(m_pwm_pin, speed);
}

void Motor::setDirection(bool forward) {
    digitalWrite(m_dir1_pin, forward ? HIGH : LOW);
}

void Motor::pinModeSetup(){
    pinMode(this->m_pwm_pin, OUTPUT);
    pinMode(this->m_dir1_pin, OUTPUT);
    pinMode(this->m_dir2_pin, OUTPUT);
}

void MotorCommands::getMotorData() {
    // int pwm_left_front = left_front.m_pwm_pin;
    // int pwm_left_back = right.pwm_front_pin;

    // int pwm_left_front = right.pwm_front_pin;
    // int pwm_right_back = left.pwm_front_pin;

    // Serial.print("PWM Left: ");
    // Serial.print(pwm_left);
    // Serial.print(", PWM Right: ");
    // Serial.println(pwm_right);
}

void MotorCommands::getOdom(){  
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
}


void MotorCommands::increaseSpeed() {
   changeSpeed(true);
}
void MotorCommands::decreaseSpeed() {
   changeSpeed(false);
}
void MotorCommands::changeSpeed(bool increase) {
    m_wheel_speed = (increase) ? m_wheel_speed += 25 : m_wheel_speed -= 25;
    m_wheel_speed = (m_wheel_speed > static_cast<int>(SpeedLimit::MAX)) ? static_cast<int>(SpeedLimit::MAX) : m_wheel_speed;
    m_wheel_speed = (m_wheel_speed < static_cast<int>(SpeedLimit::MIN)) ? static_cast<int>(SpeedLimit::MIN) : m_wheel_speed;
    
    left_back.setSpeed(m_wheel_speed);
    left_front.setSpeed(m_wheel_speed);
    right_back.setSpeed(m_wheel_speed);
    right_back.setSpeed(m_wheel_speed);
}
void MotorCommands::setStartingSpeed() {
    if (m_wheel_speed == 0){
        m_wheel_speed = static_cast<int>(SpeedLimit::MIN);
    }
}

void MotorCommands::SetMotorDir(bool left_front_wheel_forward, bool left_back_wheel_forward,
                                bool right_front_wheel_forward, bool right_back_wheel_forward){
        right_front.setDirection(right_front_wheel_forward);
        right_back.setDirection(right_back_wheel_forward);

        left_front.setDirection(left_front_wheel_forward);
        left_back.setDirection(left_back_wheel_forward);
}

void MotorCommands::stopMotors() {
    Serial.println("stopping motor");
    m_wheel_speed = 0;
    
    right_back.setSpeed(0);
    right_back.setSpeed(0);

    left_back.setSpeed(0);
    left_front.setSpeed(0);
}


void MotorCommands::SetMotorDirTest(){
    
    // char dir = static_cast<char>(movement);

    // switch (movement){
    //     case RobotMovement::MOVE_FORWARD:
    //         right_front.setDirection(m_forwards);
    //         right_back.setDirection(m_backwards);

    //         left_front.setDirection(m_forwards);
    //         left_back.setDirection(m_backwards);
    // }
}

void MotorCommands::SetMotorSpeed(){
    right_back.setSpeed(m_wheel_speed);
    right_front.setSpeed(m_wheel_speed);

    left_back.setSpeed(m_wheel_speed);
    left_front.setSpeed(m_wheel_speed);
}

void MotorCommands::moveForward() {

    setStartingSpeed();
    SetMotorDir(m_forwards, m_backwards, m_forwards, m_backwards);
    SetMotorSpeed();
}

void MotorCommands::moveBackward() {
    setStartingSpeed();
    SetMotorDir(m_backwards, m_forwards, m_backwards, m_forwards);
    SetMotorSpeed();
}



void MotorCommands::turn(bool rightTurn) {
    setStartingSpeed();
    if(rightTurn){
        //for a right run, right side should move backwards and left side forwards
        SetMotorDir(m_backwards, m_forwards, m_forwards, m_backwards,);
    }else{
        //for a left run, right side should move forwards and left side backwards
        SetMotorDir(m_forwards, m_backwards, m_backwards, m_forwards);
    }
    SetMotorSpeed();
}

void MotorCommands::turnLeft() {
    turn(false);
}

void MotorCommands::turnRight() {
    turn(true);
}

void MotorCommands::moveRight(){
    // move lateral right
    setStartingSpeed();
    SetMotorDir(m_forwards, m_forwards, m_backwards, m_backwards);
    SetMotorSpeed();
}

void MotorCommands::moveLeft(){
    setStartingSpeed();
    SetMotorDir( m_backwards, m_backwards, m_forwards, m_forwards);
    SetMotorSpeed();
}

void MotorCommands::moveForwardRightDiag(){
    //TODO: set some motors on/off
}
void MotorCommands::moveForwardleftDiag(){
    //TODO: set some motors on/off
}

void MotorCommands::moveBackwardRightDiag(){
    //TODO: set some motors on/off
}
void MotorCommands::moveBackwardsleftDiag(){
    //TODO: set some motors on/off
}

void MotorCommands::setupArduino(){

    right_front.pinModeSetup();
    right_back.pinModeSetup();
    left_front.pinModeSetup();
    left_back.pinModeSetup();

    Serial.begin(ARDUINO_SERIAL_BAUD_RATE);
}

void MotorCommands::loopMotorControl() {

    if (Serial.available() > 0) {
        char input = Serial.read();
        RobotMovement move = RobotMovement::INVALID;
        switch (input) {
            case static_cast<char>(RobotMovement::STOP):
                this->stopMotors();
                break;
            case static_cast<char>(RobotMovement::MOVE_FORWARD):
                this->moveForward();
                break;
            case static_cast<char>(RobotMovement::MOVE_BACKWARD):
                this->moveBackward();
                break;
            case static_cast<char>(RobotMovement::TURN_LEFT_OPP):
                this->turnLeft();
                break;
            case static_cast<char>(RobotMovement::TURN_RIGHT_OPP):
                this->turnRight();
                break;
            case static_cast<char>(RobotMovement::SLOWER):
                this->decreaseSpeed();
                break;
            case static_cast<char>(RobotMovement::FASTER):
                this->increaseSpeed();
                break;
            default: 
                //fall through for now, but may auto decrease speed so you must hold down button
                break;
        }
    }
    // this->getMotorData();
}


