#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup left_motors({-14, -15, -16}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3 with blue motors
pros::MotorGroup right_motors({11, 12, 13}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6 with blue motors

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
							  &right_motors, // right motor group
							  11.5,               // 10 inch track width
							  lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
							  450,              // drivetrain rpm is 450
							  2					// horizontal drit is 2 (for now)
);             

// create an imu on port 10
pros::Imu imu(10); 

pros::Rotation vertical_sensor(1);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, -2.5);
// replace -2.5 with the inch offset of tracking wheel from tracking center

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, //vertical tracking wheel 1, set to null
	nullptr,
	&imu //inertial
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
	0, //integral gain (kI)
	3, // derivative gain (kD)
	3, // anti windup
	1, // small eror range, in inches
	100, // small error range timeout, in milliseconds
	3, // large error range, in inches
	500, // large error range timeout, in milliseconds
	20, // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
	0, // integral gain (kI)
	10, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors, // odometry sensors
	&throttle_curve, // input curves for throttle input
	&steer_curve, // input curves for steer input
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
	3, // joystick deadband out of 127
	10, // minmum output where drivetrain will move out of 127
	1.019 // expo durve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
	3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);



void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate(); // calibrate sensors
	// print position to brain screen
	pros::Task screen_task([&]() {
		while (true) {
			// print robot location to brain screen
			pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
			pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
			// delay to save resources
			pros::delay(20);
		}
	});
}


void disabled() {


}


void competition_initialize() {


}


void autonomous() {
	// set position to x:0, y:0, heading:0
	chassis.setPose(0, 0, 0);

	// turn to face heading 90 with a very long timeout
	chassis.turnToHeading(90, 10000);

}

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {

	// brake mode
	chassis.drive_brake_set(MOTOR_BRAKE_COAST);

	// loop forever
	while (true) {
		// get left y and right y positions
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// move the robot
		// tank drive
		chassis.tank(leftY, rightY);

		// delay to save resources
		pros::delay(25);
	}

}