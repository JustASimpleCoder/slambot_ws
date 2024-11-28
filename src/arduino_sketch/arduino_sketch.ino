#include "include/arduino_class.hpp"
#include "include/arduino_class.cpp"

MotorCommands commands;

void setup() {
    commands.arduino_setup();
}

void loop() {
    commands.motor_control_loop();
}
