#include "arduino_class.hpp"

// uncomment during delopment as having troulbe importing arduino.h class. 

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


void MotorCommands::moveForward() {
    wheel_speed = 255;
    right.setDirection(forwards);
    right.setSpeed(wheel_speed);

    left.setDirection(forwards);
    left.setSpeed(wheel_speed);
}

void MotorCommands::moveBackward() {
    right.setDirection(backwards);
    right.setSpeed(wheel_speed);

    left.setDirection(backwards);
    left.setSpeed(wheel_speed);
}
void MotorCommands::stopMotors() {
    wheel_speed = 0;
    right.setSpeed(0);
    left.setSpeed(0);
}

void MotorCommands::turnLeft() {
    wheel_speed = 255;
    right.setDirection(forwards);
    right.setSpeed(wheel_speed);

    left.setDirection(backwards);
    left.setSpeed(wheel_speed);
}

void MotorCommands::turnRight() {
    wheel_speed = 255;
    right.setDirection(backwards);
    right.setSpeed(wheel_speed);

    left.setDirection(forwards);
    left.setSpeed(wheel_speed);
}

void MotorCommands::motor_control_loop() {

    if (Serial.available() > 0) {
        int command = Serial.parseInt();
        switch (command) {
            case MOVE_FORWARD:
                this->moveForward();
                lastCommand = &MotorCommands::moveForward;  // Store function pointer
                break;
            case MOVE_BACKWARD:
                this->moveBackward();
                lastCommand = &MotorCommands::moveBackward;
                break;
            case TURN_LEFT:
                this->moveBackward();
                lastCommand = &MotorCommands::turnLeft;
                break;
            case TURN_RIGHT:
                this->moveBackward();
                lastCommand = &MotorCommands::turnRight;
                break;
            case STOP:
                this->moveBackward();
                lastCommand = &MotorCommands::stopMotors;
                break;
            default:
                //maintain last command();
                (this->*lastCommand)();
                break;
        }
    }
    // If a valid lastCommand exists, call it, even without serialization
    if (lastCommand) {
        (this->*lastCommand)();
    }
};

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