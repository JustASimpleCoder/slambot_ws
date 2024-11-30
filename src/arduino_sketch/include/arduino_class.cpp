#include "arduino_class.hpp"

// uncomment during delopment as having troulbe importing arduino.h class on WSL. 

//ensure to comment out below before uploading sketch Un
// void analogWrite(int a, int b){};
// void digitalWrite(int a, int b){};
// void pinMode(int a, int b){};

// class SerialArduino{
//     public: 
//         void begin(int a){};
//         int available(){};
//         int parseInt(){};
// };

// SerialArduino Serial;

// #define OUTPUT 1
// #define HIGH 1
// #define LOW 0

//Ensure to comment out above before uploading sketch

void Motor::setSpeed(int speed) {
    analogWrite(pwm_front_pin, speed);
    analogWrite(pwm_back_pin, speed);
}

void Motor::setDirection(bool forward) {
    digitalWrite(dir1_front_pin, forward ? HIGH : LOW);
    digitalWrite(dir2_front_pin, forward ? LOW : HIGH);

    digitalWrite(dir1_back_pin, forward ? LOW : HIGH);
    digitalWrite(dir2_back_pin, forward ? HIGH : LOW);
}
void MotorCommands::increaseSpeed() {
   changeSpeed(true);
}
void MotorCommands::decreaseSpeed() {
   changeSpeed(false);
}
void MotorCommands::changeSpeed(bool increase) {
    wheel_speed = (increase) ? wheel_speed += 25 : wheel_speed -= 25;
    wheel_speed = (wheel_speed > static_cast<int>(SpeedLimit::MAX)) ? static_cast<int>(SpeedLimit::MAX) : wheel_speed;
    wheel_speed = (wheel_speed < static_cast<int>(SpeedLimit::MIN)) ? static_cast<int>(SpeedLimit::MIN) : wheel_speed;
    
    right.setSpeed(wheel_speed);
    left.setSpeed(wheel_speed);
}
void MotorCommands::setStartingSpeed() {
    if (wheel_speed == 0){
        wheel_speed = static_cast<int>(SpeedLimit::MIN);
    }
}

void MotorCommands::moveForward() {

    setStartingSpeed();

    right.setDirection(forwards);
    right.setSpeed(wheel_speed);

    left.setDirection(forwards);
    left.setSpeed(wheel_speed);
}

void MotorCommands::moveBackward() {
    setStartingSpeed();
    right.setDirection(backwards);
    right.setSpeed(wheel_speed);

    left.setDirection(backwards);
    left.setSpeed(wheel_speed);
}
void MotorCommands::stopMotors() {

    Serial.println("stopping motor");
    wheel_speed = 0;
    right.setSpeed(0);
    left.setSpeed(0);
}

void MotorCommands::turnLeft() {
    setStartingSpeed();
    right.setDirection(forwards);
    right.setSpeed(wheel_speed);

    left.setDirection(backwards);
    left.setSpeed(wheel_speed);
}

void MotorCommands::turnRight() {
    setStartingSpeed();
    right.setDirection(backwards);
    right.setSpeed(wheel_speed);

    left.setDirection(forwards);
    left.setSpeed(wheel_speed);
}

void MotorCommands::motor_control_loop() {

    if (Serial.available() > 0) {
        char input = Serial.read();
        RobotCommand command = RobotCommand::INVALID;
        if (input == 'x') command = RobotCommand::STOP;
        if (input == 'w') command = RobotCommand::MOVE_FORWARD;
        if (input == 's') command = RobotCommand::MOVE_BACKWARD;
        if (input == 'a') command = RobotCommand::TURN_LEFT_OPP;
        if (input == 'd') command = RobotCommand::TURN_RIGHT_OPP;
        if (input == '-') command = RobotCommand::SLOWER;
        if (input == '+') command = RobotCommand::FASTER;

        switch (command) {
            case RobotCommand::MOVE_FORWARD:
                this->moveForward();  // Store function pointer
                break;
            case RobotCommand::MOVE_BACKWARD:
                this->moveBackward();
                break;
            case RobotCommand::TURN_LEFT_OPP:
                this->turnLeft();
                break;
            case RobotCommand::TURN_RIGHT_OPP:
                this->turnRight();
                break;
            case RobotCommand::STOP:
                this->stopMotors();
                break;
            case RobotCommand::FASTER:
                this->increaseSpeed();
                break;
            case RobotCommand::SLOWER:
                this->decreaseSpeed();
                break;
            default:
                //maintain last command();
                break;
        }
    }
}


void MotorCommands::arduino_setup(){
    pinMode(right.pwm_front_pin, OUTPUT);
    pinMode(right.dir1_front_pin, OUTPUT);
    pinMode(right.dir2_back_pin, OUTPUT);
    
    pinMode(right.pwm_back_pin, OUTPUT);
    pinMode(right.dir1_back_pin, OUTPUT);
    pinMode(right.dir2_front_pin, OUTPUT);

    pinMode(left.pwm_front_pin, OUTPUT);
    pinMode(left.dir1_front_pin, OUTPUT);
    pinMode(left.dir2_front_pin, OUTPUT);
    
    pinMode(left.pwm_back_pin, OUTPUT);
    pinMode(left.dir1_back_pin, OUTPUT);
    pinMode(left.dir2_back_pin, OUTPUT);
    Serial.begin(9600);
}