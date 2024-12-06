#include "include/arduino_class.hpp"
#include "include/arduino_class.cpp"

MotorCommands commands;

void setup() {
    commands.setupArduino();
}

void loop() {
    commands.loopMotorControl();
}
