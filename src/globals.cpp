#include "main.h"

// motors
pros::Motor intake(1, pros::v5::MotorGears::blue);

// pneumatics
pros::adi::Pneumatics doinker('H', false);

// rotation sensor
pros::Rotation vertical_sensor(1);

// color sensor
pros::Optical colorsensor(17);