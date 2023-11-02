#include "main.h"

#define ARM_MOTOR_PORT 12
#define LEFT_MOTOR_ONE_PORT 1
#define LEFT_MOTOR_TWO_PORT 19
#define RIGHT_MOTOR_ONE_PORT 11
#define RIGHT_MOTOR_TWO_PORT 13
#define CLAW_MOTOR_PORT 6


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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

	double maxSlowVelocity = 30;
	double maxVelocity = 100;
	//controller object
	Controller controller;

	//arm motor control objects
	Motor armMotor(-ARM_MOTOR_PORT);
	ControllerButton armUpButton(ControllerDigital::A);
	ControllerButton armDownButton(ControllerDigital::B);

	//drivetrain
	Motor frontLeftMotor(LEFT_MOTOR_ONE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backLeftMotor(LEFT_MOTOR_TWO_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor frontRightMotor(RIGHT_MOTOR_ONE_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backRightMotor(RIGHT_MOTOR_TWO_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	ControllerButton slowButton(ControllerDigital::L2);
	
	// Chassis Controller - lets us drive the robot around with open- or closed-loop control
	std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
	        .withMotors(
				 frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor   // Bottom left
			)
	        // Green gearset, 4 in wheel diam, 11.5 in wheel track
	        .withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
			.withOdometry()
	        .buildOdometry();
	
	auto xModel = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());
	/*std::shared_ptr<ChassisController> drive =
	    ChassisControllerBuilder()
	        .withMotors(
				 frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor   // Bottom left
			)
	        // Green gearset, 4 in wheel diam, 11.5 in wheel track
	        .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
			.withOdometry()
	        .buildOdometry();
*/
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
    // Tank drive with left and right sticks.
	double leftX = slowButton.isPressed() ? controller.getAnalog(ControllerAnalog::leftX) / 4 : controller.getAnalog(ControllerAnalog::leftX);
	double leftY = slowButton.isPressed() ? controller.getAnalog(ControllerAnalog::leftY) / 4 : controller.getAnalog(ControllerAnalog::leftY);
	double rightX = slowButton.isPressed() ? controller.getAnalog(ControllerAnalog::rightX) / 4 : controller.getAnalog(ControllerAnalog::rightX);


    xModel->xArcade(leftX, leftY, rightX);
		pros::delay(10);
	
	if (armUpButton.isPressed()) {
                armMotor.moveVoltage(12000);
            } else if (armDownButton.isPressed()) {
                armMotor.moveVoltage(-12000);
            } else {
                armMotor.moveVoltage(0);
            }

	}

}