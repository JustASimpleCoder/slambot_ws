
//OOP abstraction for arduino sketch to control the robot
#ifndef ARDUINO_CLASS_HPP
#define ARDUINO_CLASS_HPP

#define ANALOG_WRITE_WHEEL_MAX 255
#define ANALOG_WRITE_WHEEL_MIN 125

# define PI 3.14159265358979323846

// const int STOP = 0;
// const int MOVE_FORWARD = 1;
// const int MOVE_BACKWARD = 2;
// const int TURN_LEFT = 3;
// const int TURN_RIGHT = 4;
// const int TURN_LEFT_OPP = 5; // both side of robot move in opposite dir  
// const int TURN_RIGHT_OPP = 6; //both side of robot move in opposite dir  
// const int INVALID = -1;
enum class SpeedLimit {
    MIN = 125,
    MAX = 255
};

enum class RobotCommand {
    STOP = 0,
    MOVE_FORWARD = 1,
    MOVE_BACKWARD = 2,
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
    TURN_LEFT_OPP = 5,
    TURN_RIGHT_OPP = 6,
    FASTER = 7,
    SLOWER = 8,
    INVALID = -1
};

class Motor {
    public:
        int pwm_front_pin;
        int dir1_front_pin;
        int dir2_front_pin;

        int pwm_back_pin;
        int dir1_back_pin;
        int dir2_back_pin;


        Motor(int pwm_front, int dir1_front, int dir2_ront, int pwm_back, int dir1_back, int dir2_back)
            : pwm_front_pin(pwm_front), dir1_front_pin(dir1_front), dir2_front_pin(dir2_ront),
             pwm_back_pin(pwm_back), dir1_back_pin(dir1_back), dir2_back_pin(dir2_back) {}

        void setSpeed(int speed);
        void setDirection(bool forward);
};

class MotorCommands {
    private:
        Motor right;
        Motor left;

        void (MotorCommands::*lastCommand)();
        
        bool forwards = true;
        bool backwards = false;

        //3 inches is diamter -> 0.0762 m D -> 0.0281 for radius
        double wheel_radius = 0.0281;
        double wheel_circum  = 2*PI*wheel_radius;
        double wheel_base = 0.16002;

    public:
        unsigned int wheel_speed;
        MotorCommands()
            : right(6, 4, 7, 5, 2, 3),
            left(9, 11, 8, 10, 13, 12),
            wheel_speed(0),
            lastCommand(MotorCommands::lastCommand) {}
        
        ~MotorCommands(){
            //stopMotors();
        }

        void getMotorData();
        
        void getOdom();

        void setupArduino();

        void moveForward();

        void moveBackward();

        void stopMotors();

        void turnLeft();

        void turnRight();

        void loopMotorControl();

        void changeSpeed(bool increase);

        void increaseSpeed();

        void decreaseSpeed();

        void setStartingSpeed();

        void turn(bool right);
};
#endif