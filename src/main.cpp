#include "main.h"

#define ARM_MOTOR_PORT 15
#define LEFT_MOTOR_ONE_PORT 20
#define LEFT_MOTOR_TWO_PORT 19
#define RIGHT_MOTOR_ONE_PORT 16
#define RIGHT_MOTOR_TWO_PORT 14
#define CLAW_MOTOR_PORT 13

#define INERTIAL_SENSOR_PORT 11
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
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
void opcontrol()
{
	int newHeading = 0;
	int JoisticHeading = 0;
	int chassisHeading;
	int headingError;

	double forward, straff, turning;
	int temp;
	float theta;
	bool FieldCenteric = true;

	double maxSlowVelocity = 30;
	double maxVelocity = 100;
	// controller object
	Controller controller;
	ControllerButton armUpButton(ControllerDigital::A);
	ControllerButton armDownButton(ControllerDigital::B);
	ControllerButton slowButton(ControllerDigital::L2);
	ControllerButton resetHeading(ControllerDigital::down);
	ControllerButton enableFieldCentric(ControllerDigital::left);
	// arm motor control objects
	Motor armMotor(-ARM_MOTOR_PORT);

	// inertial meausrement unit
	pros::Imu inertialSensor(INERTIAL_SENSOR_PORT);

	// drivetrain
	Motor frontLeftMotor(LEFT_MOTOR_ONE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backLeftMotor(LEFT_MOTOR_TWO_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor frontRightMotor(RIGHT_MOTOR_ONE_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backRightMotor(RIGHT_MOTOR_TWO_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

	// reset IMU
	inertialSensor.reset();

	// Chassis Controller - lets us drive the robot around with open- or closed-loop control
	std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
													 .withMotors(
														 frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor // Bottom left
														 )
													 // Green gearset, 4 in wheel diam, 11.5 in wheel track
													 .withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
													 .withOdometry()
													 .buildOdometry();

	std::shared_ptr<okapi::XDriveModel> driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());
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
	while (inertialSensor.is_calibrating())
	{
		pros::delay(20);
	}
	while (true)
	{;
		chassisHeading = inertialSensor.get_heading() - headingError;
		printf("Heading: %f\n", chassisHeading);
		pros::lcd::print(3, "Heading: %f", chassisHeading);
		forward = controller.getAnalog(ControllerAnalog::leftY) * 100;
		/**if (abs(forward) < 0.3)
		{
			forward = 0;
		}*/
		straff = controller.getAnalog(ControllerAnalog::leftX) * 100;
		/**if (abs(straff) < 0.3)
		{
			straff = 0;
		}*/
		turning = controller.getAnalog(ControllerAnalog::rightX) * 100;
		/*if (abs(turning) < 0.3)
		{
			turning = 0;
		}*/

		theta = (-chassisHeading * 3.1415926535) / 180;
		temp = forward * cos(theta) - straff * sin(theta);
		straff = forward * sin(theta) + straff * cos(theta);
		forward = temp;
		std::cout << inertialSensor.get_pitch() << std::endl;
		if (inertialSensor.get_pitch() < -10)
		{
			driveTrain->forward(-30);
			pros::delay(200);
		}
		if (enableFieldCentric.changedToPressed())
		{
			if (FieldCenteric == true)
			{
				controller.setText(1, 8, "False");
				FieldCenteric = false;
			}
			else
			{
				controller.setText(1, 8, "True");
				FieldCenteric = true;
			}
		}
		if (resetHeading.isPressed())
		{
			headingError = inertialSensor.get_heading();
		}

		int TargetStraff = FieldCenteric ? straff / 100 : controller.getAnalog(ControllerAnalog::leftX);
		int TargetForward = FieldCenteric ? forward / 100 : controller.getAnalog(ControllerAnalog::leftY);
		int TargetTurning = FieldCenteric ? turning / 100 : controller.getAnalog(ControllerAnalog::rightX);

		TargetStraff = slowButton.isPressed() ? TargetStraff / 4 : TargetStraff;
		TargetForward = slowButton.isPressed() ? TargetForward / 4 : TargetForward;
		TargetTurning = slowButton.isPressed() ? TargetTurning / 4 : TargetTurning;

		driveTrain->xArcade(TargetStraff, TargetForward, TargetTurning);

		if (armUpButton.isPressed())
		{
			armMotor.moveVoltage(12000);
		}
		else if (armDownButton.isPressed())
		{
			armMotor.moveVoltage(-12000);
		}
		else
		{
			armMotor.moveVoltage(0);
		}
	}
	/**
	while (true)
	{
		// print values to display
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		pros::lcd::print(2, "%f", inertialSensor.get());

		// mecanum drive with left and right sticks.

		// raw values from controller
		double leftXRaw = controller.getAnalog(ControllerAnalog::leftX);
		double leftYRaw = controller.getAnalog(ControllerAnalog::leftY);
		double rightXRaw = controller.getAnalog(ControllerAnalog::rightX);

		// clip values close to 0
		const int deadband = 15;
		if (abs(leftXRaw) < deadband)
			leftXRaw = 0;
		if (abs(leftYRaw) < deadband)
			leftYRaw = 0;
		if (abs(rightXRaw) < deadband)
			rightXRaw = 0;

		double theta = degToRad(inertialSensor.get());

		int temp = leftXRaw * sin(theta) + leftYRaw * cos(theta);
		leftXRaw = leftXRaw * cos(theta) - leftYRaw * sin(theta);
		leftYRaw = temp;
		// scale values to max velocity
		double leftX = slowButton.isPressed() ? leftXRaw / 4 : leftXRaw;
		double leftY = slowButton.isPressed() ? leftYRaw / 4 : leftYRaw;
		double rightX = slowButton.isPressed() ? rightXRaw / 4 : rightXRaw;

		xModel->xArcade(leftX, leftY, rightX);

		if (armUpButton.isPressed())
		{
			armMotor.moveVoltage(12000);
		}
		else if (armDownButton.isPressed())
		{
			armMotor.moveVoltage(-12000);
		}
		else
		{
			armMotor.moveVoltage(0);
		}
		// sleep at end of loop
		pros::delay(10);
	}
	*/
}

double degToRad(double deg)
{
	return deg * 3.14159265358979323846 / 180;
}