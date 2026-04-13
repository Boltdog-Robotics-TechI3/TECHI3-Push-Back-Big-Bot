#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	initializeScreen();
	chassis.reset();
	intakeInitialize();
	chassis.setInputScale(Chassis::InputScale::SINSQUARED);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// chassis.setPose(0, 0, -M_PI/4);
	// imu.set_yaw(45);
	chassis.startTracking();


	
	chassis.moveToPose(Pose(26, 26, 0), 5000, 75);
	// chassis.moveToPose(Pose(0, 37, 0), 5000, 75);
	// chassis.turnToAngle(45, 3000);




	pros::delay(5000);

// 	imu.set_heading(180);
// 	chassis.startTracking();

// 	chassis.moveToPose(Pose(30, 0, 0), 3000, 75);
// 	pros::delay(200);

// 	// Grab from goal
// 	intakePiston.extend();
// 	setIntakeSpeed(100);
// 	chassis.moveToPose(Pose(30, -20, 0), 1000, 75);

// 	// Score the goods
// 	pros::delay(3000);
// 	chassis.moveToPose(Pose(30, 20, 0), 3000, 75);
// 	pros::delay(200);
// 	scoreHigh(100);
// 	pros::delay(2000);

// 	// Eject bads
// 	chassis.moveToPose(Pose(30, 0, 0), 2000, 75);
// 	chassis.turnToAngle(45, 1000);
// 	pros::delay(200);
// 	scoreHigh(100);
// 	pros::delay(2000);
// 	setIntakeSpeed(0);
// 	chassis.turnToAngle(0, 1000);
// 	pros::delay(200);

// 	// grab goods from loader
// 	setIntakeSpeed(100);
// 	chassis.moveToPose(Pose(30, -20, 0), 2000, 75);
// 	pros::delay(3000);
	
// 	// score remaining blocks long goal
// 	chassis.moveToPose(Pose(30, 20, 0), 2000, 75);
// 	scoreHigh(100);


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	int leftX;
	int leftY;
	int rightX;
	chassis.startTracking();

	while (true) {
		leftX = controller.get_analog(ANALOG_LEFT_X);
		leftY = controller.get_analog(ANALOG_LEFT_Y);
		rightX = controller.get_analog(ANALOG_RIGHT_X);

		intakePeriodic();
		wingPeriodic();
		
		if(controller.get_digital_new_press(DIGITAL_X)){
			imu.tare();
		}

		// if (wingPiston.is_extended()) {
		// 	leftX = 0;
		// }

		// if (controller.get_digital(DIGITAL_B)) {
		// 	autonomous();
		// 	// controller.set_text(0, 0, " balls");+
		// 	// chassis.moveDistanceJANKY(500, 10000);
		// }
		
		chassis.fieldCentricDrive(leftX, leftY, rightX);
		
		controller.set_text(0,0, std::to_string(indexer.get_actual_velocity()));
		//controller.set_text(.0, 0, chassis.getPose().to_string());

		pros::delay(20);
	}
}