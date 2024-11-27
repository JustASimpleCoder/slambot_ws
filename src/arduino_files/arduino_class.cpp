
//OOP abstraction for arduino sketch to control the robot
enum class RobotCommand {
    STOP = 'x',
    MOVE_FORWARD = 'w',
    MOVE_BACKWARD = 's',
    TURN_LEFT = 'a',
    TURN_RIGHT = 'd',
    INVALID = 'z'
};

class Motor {
    private:
        int pwm_pin;
        int dir1_pin;
        int dir2_pin;
    public:
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

    public:
        MotorCommands()
            : right_front(6, 4, 7),
            right_back(5, 2, 3),
            left_front(9, 11, 8),
            left_back(10, 13, 12) {}

        void moveForward(int speed) {
            right_front.setDirection(true);
            right_front.setSpeed(speed);

            right_back.setOppDirection(true);
            right_back.setSpeed(speed);

            left_front.setDirection(true);
            left_front.setSpeed(speed);

            left_back.setDirection(true);
            left_back.setSpeed(speed);
        }

        void moveBackward(int speed) {
            right_front.setDirection(false);
            right_front.setSpeed(speed);

            right_back.setDirection(false);
            right_back.setSpeed(speed);

            left_front.setDirection(false);
            left_front.setSpeed(speed);

            left_back.setDirection(false);
            left_back.setSpeed(speed);
        }
        void stopMotors() {
            right_front.setSpeed(0);
            right_back.setSpeed(0);
            left_front.setSpeed(0);
            left_back.setSpeed(0);
        }

        void turnLeft(int speed) {
            right_front.setDirection(true);
            right_front.setSpeed(speed);

            right_back.setDirection(true);
            right_back.setSpeed(speed);

            left_front.setDirection(false);
            left_front.setSpeed(speed);

            left_back.setDirection(false);
            left_back.setSpeed(speed);
        }

        void turnRight(int speed) {
            right_front.setDirection(false);
            right_front.setSpeed(speed);

            right_back.setDirection(false);
            right_back.setSpeed(speed);

            left_front.setDirection(true);
            left_front.setSpeed(speed);

            left_back.setDirection(true);
            left_back.setSpeed(speed);
        }
};

MotorCommands commands;


void setup() {
  // TO_DO: add this to an init()
  pinMode(right_front_pin,OUTPUT);
  pinMode(right_front_dir1_pin,OUTPUT);
  pinMode(right_front_dir2_pin,OUTPUT);
  
  pinMode(right_back_pin,OUTPUT);
  pinMode(right_back_dir1_pin,OUTPUT);
  pinMode(right_back_dir2_pin,OUTPUT);

  pinMode(left_front_pin,OUTPUT);
  pinMode(left_front_dir1_pin,OUTPUT);
  pinMode(left_front_dir2_pin,OUTPUT);
  
  pinMode(left_back_pin,OUTPUT);
  pinMode(left_back_dir1_pin,OUTPUT);
  pinMode(left_back_dir2_pin,OUTPUT);
  Serial.begin(9600);

}  

void loop() {
    if (Serial.available() > 0) {
        char input = Serial.read();
        RobotCommand command = static_cast<RobotCommand>(input);

        switch (command) {
            case RobotCommand::MOVE_FORWARD:
                commands.moveForward(255);
                break;
            case RobotCommand::MOVE_BACKWARD:
                commands.moveBackward(255);
                break;
            case RobotCommand::TURN_LEFT:
                commands.turnLeft(200);
                break;
            case RobotCommand::TURN_RIGHT:
                commands.turnRight(200);
                break;
            case RobotCommand::STOP:
            default:
                commands.stopMotors();
                break;
        }
    }
}