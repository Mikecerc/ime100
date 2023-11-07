#include "main.h"

#define ARM_MOTOR_PORT 18
#define LEFT_MOTOR_ONE_PORT 20
#define LEFT_MOTOR_TWO_PORT 19
#define RIGHT_MOTOR_ONE_PORT 16
#define RIGHT_MOTOR_TWO_PORT 14
#define CLAW_DOOR_MOTOR_PORT 17
#define INERTIAL_SENSOR_PORT 11
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Broke Engineers!");
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
	//field centric drive control
	int newHeading = 0;
	int chassisHeading, headingError, temp;
	double forward, straff, turning;
	float theta;
	bool FieldCenteric = true;

	// velocity variables
	int slowMultiplier = 0.5;
	//double maxSlowVelocity = 30;
	//double maxVelocity = 100;

	// controller object
	Controller controller;
	ControllerButton armUpButton(ControllerDigital::A);
	ControllerButton armDownButton(ControllerDigital::B);
	ControllerButton slowButton(ControllerDigital::L2);
	ControllerButton resetHeading(ControllerDigital::down);
	ControllerButton enableFieldCentric(ControllerDigital::left);
	ControllerButton doorIn(ControllerDigital::R1);
	ControllerButton doorOut(ControllerDigital::R2);
	ControllerButton runAuton(ControllerDigital::up);
	ControllerButton tempDisableFieldCentric(ControllerDigital::L1);

	// inertial meausrement unit
	pros::Imu inertialSensor(INERTIAL_SENSOR_PORT);

	// arm
	Motor armMotor(-ARM_MOTOR_PORT);
	armMotor.setBrakeMode(AbstractMotor::brakeMode::hold);

	//claw
	Motor clawDoorMotor(CLAW_DOOR_MOTOR_PORT);

	// drivetrain
	Motor frontLeftMotor(LEFT_MOTOR_ONE_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backLeftMotor(LEFT_MOTOR_TWO_PORT, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor frontRightMotor(RIGHT_MOTOR_ONE_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	Motor backRightMotor(RIGHT_MOTOR_TWO_PORT, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

	// reset IMU
	inertialSensor.reset();

	// Chassis Controller - lets us drive the robot around with open- or closed-loop control
	std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
													 .withMotors(
														 frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor // Bottom left
														 )
													 // Green gearset, 4 in wheel diam, 11.5 in wheel track
													 .withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
													 .withMaxVelocity(90)
													 .withOdometry()
													 .buildOdometry();
	// X-Drive Model for mecanum drive
	std::shared_ptr<okapi::XDriveModel> driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());
	
	//wait for IMU to calibrate
	while (inertialSensor.is_calibrating())
	{
		pros::delay(20);
	}

	//control loop
	while (true)
	{
		//update heading based off IMU reading
		chassisHeading = inertialSensor.get_heading() - headingError;

		//get controller values
		forward = controller.getAnalog(ControllerAnalog::leftY) * 100;
		straff = controller.getAnalog(ControllerAnalog::leftX) * 100;
		turning = controller.getAnalog(ControllerAnalog::rightX) * 100;

		//coordinate transform.
		theta = (-chassisHeading * 3.1415926535) / 180;
		temp = forward * cos(theta) - straff * sin(theta);
		straff = forward * sin(theta) + straff * cos(theta);
		forward = temp;

		//field centric drive control toggle button
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

		//temporarly disable field centric drive control function
		if (tempDisableFieldCentric.isPressed()) {
			FieldCenteric == false;
		} else {
			FieldCenteric == true;
		}

		//reset heading function
		if (resetHeading.isPressed())
		{
			headingError = inertialSensor.get_heading();
		}

		// set target stick values based off whether or not field centric mode is enabled
		double TargetStraff = FieldCenteric ? straff / 100 : controller.getAnalog(ControllerAnalog::leftX);
		double TargetForward = FieldCenteric ? forward / 100 : controller.getAnalog(ControllerAnalog::leftY);
		double TargetTurning = FieldCenteric ? turning / 100 : controller.getAnalog(ControllerAnalog::rightX);

		// scale values to max velocity
		TargetStraff = slowButton.isPressed() ? TargetStraff * slowMultiplier : TargetStraff;
		TargetForward = slowButton.isPressed() ? TargetForward * slowMultiplier : TargetForward;
		TargetTurning = slowButton.isPressed() ? TargetTurning * slowMultiplier : TargetTurning;

		// HARD CAP VELOCITY (TEMP)
		TargetStraff = TargetStraff / 2;
		TargetForward = TargetForward / 2;
		TargetTurning = TargetTurning / 2;

		// feed control input values to okapi chassis controller
		driveTrain->xArcade(TargetStraff, TargetForward, TargetTurning,0);

		// arm control
		if (armUpButton.isPressed())
		{
			armMotor.moveVoltage(4000);
		}
		else if (armDownButton.isPressed())
		{
			armMotor.moveVoltage(-8000);
		}
		else
		{
			armMotor.moveVoltage(-1700);
		}

		// claw control
		if (doorIn.isPressed())
		{
			clawDoorMotor.moveVoltage(3000);
		}
		else if (doorOut.isPressed())
		{
			clawDoorMotor.moveVoltage(-3000);
		}
		else
		{
			clawDoorMotor.moveVoltage(0);
		}

		// run auton
		if (runAuton.isPressed())
		{
			std::cout << "Running Auton" << std::endl;
			/**chassis->moveDistance(0.5_m);
			chassis->moveDistance(-0.5_m);
			chassis->turnAngle(90_deg);
			chassis->moveDistance(4.5_ft);
			chassis->turnAngle(-90_deg);
			chassis->moveDistance(0.8_m); */
			chassis->moveDistance(1_m);

		}

		// sleep at end of loop
		pros::delay(20);
	}
}