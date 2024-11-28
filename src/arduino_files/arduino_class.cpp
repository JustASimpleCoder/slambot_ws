
//OOP abstraction for arduino sketch to control the robot

#define ANALOG_WRITE_WHEEL_MAX 255
#define ANALOG_WRITE_WHEEL_MIN 125

const int STOP = 0;
const int MOVE_FORWARD = 1;
const int MOVE_BACKWARD = 2;
const int TURN_LEFT = 3;
const int TURN_RIGHT = 4;
const int TURN_LEFT_OPP = 5; // both side of robot move in opposite dir  
const int TURN_RIGHT_OPP = 6; //both side of robot move in opposite dir  
const int INVALID = -1;

class Motor {
    public:
        int pwm_pin;
        int dir1_pin;
        int dir2_pin;
        Motor(int pwm, int dir1, int dir2)
            : pwm_pin(pwm), dir1_pin(dir1), dir2_pin(dir2) {}

        void setSpeed(int speed) {
            analogWrite(pwm_pin, speed);
        }

        void setDirection(bool forward) {
            digitalWrite(dir1_pin, forward ? HIGH : LOW);
            digitalWrite(dir2_pin, forward ? LOW : HIGH);
        }
        void setOppDirection(bool forward) {
            digitalWrite(dir1_pin, forward ? LOW : HIGH);
            digitalWrite(dir2_pin, forward ? HIGH : LOW);
        }
};

class MotorCommands {
    private:
        Motor right_front;
        Motor right_back;
        Motor left_front;
        Motor left_back;
        

        unsigned int wheel_speed;
        bool forward = true;
        void (MotorCommands::*lastCommand)();

    public:
        MotorCommands()
            : right_front(6, 4, 7),
            right_back(5, 2, 3),
            left_front(9, 11, 8),
            left_back(10, 13, 12),
            wheel_speed(0),
            lastCommand(nullptr) {}

        void arduino_setup() {
            pinMode(right_front.pwm_pin,OUTPUT);
            pinMode(right_front.dir1_pin,OUTPUT);
            pinMode(right_front.dir2_pin,OUTPUT);
            
            pinMode(right_back.pwm_pin,OUTPUT);
            pinMode(right_back.dir1_pin,OUTPUT);
            pinMode(right_back.dir2_pin,OUTPUT);

            pinMode(left_front.pwm_pin,OUTPUT);
            pinMode(left_front.dir1_pin,OUTPUT);
            pinMode(left_front.dir2_pin,OUTPUT);
            
            pinMode(left_back.pwm_pin, OUTPUT);
            pinMode(left_back.dir1_pin, OUTPUT);
            pinMode(left_back.dir2_pin, OUTPUT);
            Serial.begin(9600);

        }  
        void moveForward() {
            wheel_speed = 255;
            right_front.setDirection(true);
            right_front.setSpeed(wheel_speed);

            right_back.setOppDirection(true);
            right_back.setSpeed(wheel_speed);

            left_front.setDirection(true);
            left_front.setSpeed(wheel_speed);

            left_back.setOppDirection(true);
            left_back.setSpeed(wheel_speed);
        }

        void moveBackward() {
                wheel_speed = 255;

                right_front.setDirection(false);
                right_front.setSpeed(wheel_speed);

                right_back.setOppDirection(false);
                right_back.setSpeed(wheel_speed);

                left_front.setDirection(false);
                left_front.setSpeed(wheel_speed);

                left_back.setOppDirection(false);
                left_back.setSpeed(wheel_speed);
 
        }
        void stopMotors() {
            right_front.setSpeed(0);
            right_back.setSpeed(0);
            left_front.setSpeed(0);
            left_back.setSpeed(0);
            wheel_speed = 0;
        }

        void turnLeft() {
            wheel_speed = 255;
            right_front.setDirection(true);
            right_front.setSpeed(wheel_speed);

            right_back.setOppDirection(true);
            right_back.setSpeed(wheel_speed);

            left_front.setDirection(false);
            left_front.setSpeed(wheel_speed);

            left_back.setOppDirection(false);
            left_back.setSpeed(wheel_speed);
        }

        void turnRight() {
            wheel_speed = 255;
            right_front.setDirection(false);
            right_front.setSpeed(wheel_speed);

            right_back.setOppDirection(false);
            right_back.setSpeed(wheel_speed);

            left_front.setDirection(true);
            left_front.setSpeed(wheel_speed);

            left_back.setOppDirection(true);
            left_back.setSpeed(wheel_speed);
        }

        void motor_control_loop() {
            if (Serial.available() > 0) {
                int command = Serial.parseInt();
                switch (command) {
                    case MOVE_FORWARD:
                        lastCommand = &MotorCommands::moveForward;  // Store function pointer
                        break;
                    case MOVE_BACKWARD:
                        lastCommand = &MotorCommands::moveBackward;
                        break;
                    case TURN_LEFT:
                        lastCommand = &MotorCommands::turnLeft;
                        break;
                    case TURN_RIGHT:
                        lastCommand = &MotorCommands::turnRight;
                        break;
                    case STOP:
                        lastCommand = &MotorCommands::stopMotors;
                        break;
                    default:
                        // Invalid command, do nothing
                        break;
                }
            }

            // If a valid lastCommand exists, call it
            if (lastCommand) {
                (this->*lastCommand)();
            }
        }
};

MotorCommands commands;
void  setup() {
    commands.arduino_setup();
}  

void loop() {
    commands.motor_control_loop();
}