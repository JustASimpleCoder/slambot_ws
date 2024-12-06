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

void MotorCommands::getMotorData() {
    int pwm_left = right.pwm_front_pin;
    int pwm_right = left.pwm_front_pin;

    Serial.print("PWM Left: ");
    Serial.print(pwm_left);
    Serial.print(", PWM Right: ");
    Serial.println(pwm_right);
}

void MotorCommands::getOdom(){  
    double robot_x_front = 0.0; 
    double robot_x_back = 0.0; 

    double robot_y_front = 0.0; 
    double robot_y_back = 0.0; 

    double robot_theta_front = 0.0; 
    double robot_theta_back = 0.0; 

    // Estimate the time of travel for this cycle (in seconds)
    unsigned long timeInterval = 100; // Time interval in milliseconds
    double timeInSeconds = timeInterval / 1000.0;

    // Estimate wheel speeds based on PWM
    double right_front_speed = map(right.pwm_front_pin , 0, 255, 0, 1); 
    double right_back_speed = map(right.pwm_back_pin , 0, 255, 0, 1);
    double right_front_speed = map(left.pwm_front_pin , 0, 255, 0, 1); 
    double right_back_speed = map(left.pwm_back_pin , 0, 255, 0, 1);

  
    // Calculate the linear distance traveled by each wheel
    double left_front_dist = right_front_speed * wheel_circum * timeInSeconds;
    double left_back_dist = right_front_speed * wheel_circum * timeInSeconds;
    double right_front_dist = right_front_speed * wheel_circum * timeInSeconds;
    double right_back_dist = right_front_speed * wheel_circum * timeInSeconds;

    // Calculate robot movement
    double delta_front_dist = (left_front_dist + right_front_dist) / 2.0;
    double delta_back_dist = (left_back_dist + right_back_dist) / 2.0;

    double delta_front_theta = (right_front_dist - left_front_dist) / wheel_base;
    double delta_back_theta = (right_back_dist - left_back_dist) / wheel_base;
    // Update robot position and orientation
    robot_x_front += delta_front_dist * cos(delta_front_theta);
    robot_x_back += delta_back_dist * cos(delta_back_theta);

    robot_y_front += delta_front_dist * sin(delta_front_theta);
    robot_y_back += delta_back_dist * sin(delta_back_theta);


    robot_theta_front += delta_front_theta; // Update orientation
    robot_theta_back += delta_back_theta;
    // Normalize angle (keep between -PI and PI)
    if (robot_theta_front > PI) robot_theta_front -= 2 * PI;
    if (robot_theta_front < -PI) robot_theta_front += 2 * PI;

    if (robot_theta_back > PI) robot_theta_back -= 2 * PI;
    if (robot_theta_back < -PI) robot_theta_back += 2 * PI;

  // Print the robot's current position and orientation
    Serial.print("FrontX: "); Serial.print(robot_x_front); 
    Serial.print(", FrontY: "); Serial.print(robot_y_front);
    Serial.print(", FrontTheta: "); Serial.println(robot_theta_front);

    Serial.print("BackX: "); Serial.print(robot_x_back); 
    Serial.print(", BackY: "); Serial.print(robot_y_back);
    Serial.print(", BackTheta: "); Serial.println(robot_theta_back);
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

void MotorCommands::turn(bool rightTurn) {
    setStartingSpeed();
    if(rightTurn){
        //for a right run, right side should move back and left side forward
        right.setDirection(backwards);
        right.setSpeed(wheel_speed);

        left.setDirection(forwards);
        left.setSpeed(wheel_speed);
    }else{
        //for a left run, right side should move back and left side forward
        right.setDirection(forwards);
        right.setSpeed(wheel_speed);

        left.setDirection(backwards);
        left.setSpeed(wheel_speed);
    }
}

void MotorCommands::turnLeft() {
    turn(false);
}

void MotorCommands::turnRight() {
    turn(true);
}

void MotorCommands::loopMotorControl() {

    if (Serial.available() > 0) {
        char input = Serial.read();
        RobotCommand command = RobotCommand::INVALID;
        switch (input) {
            case 'x': 
                command = RobotCommand::STOP; 
                break;
            case 'w': 
                command = RobotCommand::MOVE_FORWARD; 
                break;
            case 's': 
                command = RobotCommand::MOVE_BACKWARD; 
                break;
            case 'a': 
                command = RobotCommand::TURN_LEFT_OPP; 
                break;
            case 'd': 
                command = RobotCommand::TURN_RIGHT_OPP; 
                break;
            case '-': 
                command = RobotCommand::SLOWER; 
                break;
            case '+': 
                command = RobotCommand::FASTER; 
                break;
            default: 
                command = RobotCommand::INVALID; 
                break;
        }

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
    this->getMotorData();
}


void MotorCommands::setupArduino(){

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