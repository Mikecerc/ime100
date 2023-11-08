#include "main.h"

// field centric drive control
int newHeading = 0;
int chassisHeading, headingError, temp;
float theta;
bool Fieldcentric = true;
bool slowMode = false;

// inertial meausrement unit
pros::Imu inertialSensor(Constants::Port::InertialSensor);

// drivetrain
Motor frontLeftMotor(Constants::Port::LeftFrontDrive, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor backLeftMotor(Constants::Port::LeftBackDrive, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor frontRightMotor(Constants::Port::RightFrontDrive, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor backRightMotor(Constants::Port::RightBackDrive, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<ChassisController> chassis;
std::shared_ptr<okapi::XDriveModel> driveTrain;

void ChassisOpcontrol(void *param)
{
    // reset IMU
    // inertialSensor.reset();

    // Chassis Controller - lets us drive the robot around with open- or closed-loop control
    chassis = ChassisControllerBuilder()
                  .withMotors(
                      frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor // Bottom left
                      )
                  // Green gearset, 4 in wheel diam, 11.5 in wheel track
                  .withDimensions(Constants::Drivetrain::gearset, Constants::Drivetrain::chassisDimensions)
                  .withMaxVelocity(Constants::Drivetrain::maxVelocity)
                  .withOdometry()
                  .buildOdometry();
    // X-Drive Model for mecanum drive
    driveTrain = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());

    // wait for IMU to calibrate
    while (inertialSensor.is_calibrating())
    {
        pros::delay(20);
    }

    // control loop
    while (true)
    {
        // update heading based off IMU reading
        chassisHeading = inertialSensor.get_heading() - headingError;

        // update field centric toggle button state
        controller.setText(1, 8, Fieldcentric ? "True" : "False");
    }
}
void resetHeading()
{
    headingError = inertialSensor.get_heading();
}
controllerValues computeFieldCentricValues(double forward, double straff, double turning)
{
    forward * 100;
    straff * 100;
    turning * 100;
    // coordinate transform.
    theta = (-chassisHeading * 3.1415926535) / 180;
    temp = forward * cos(theta) - straff * sin(theta);
    straff = forward * sin(theta) + straff * cos(theta);
    forward = temp;

    // set target stick values based off whether or not field centric mode is enabled
    double TargetStraff = Fieldcentric ? straff / 100 : controller.getAnalog(ControllerAnalog::leftX);
    double TargetForward = Fieldcentric ? forward / 100 : controller.getAnalog(ControllerAnalog::leftY);
    double TargetTurning = Fieldcentric ? turning / 100 : controller.getAnalog(ControllerAnalog::rightX);

    // scale values to max velocity
    TargetStraff = slowMode ? TargetStraff * Constants::Drivetrain::slowMultiplier : TargetStraff;
    TargetForward = slowMode ? TargetForward * Constants::Drivetrain::slowMultiplier : TargetForward;
    TargetTurning = slowMode ? TargetTurning * Constants::Drivetrain::slowMultiplier : TargetTurning;

    // HARD CAP VELOCITY (TEMP)
    TargetStraff = TargetStraff / 2;
    TargetForward = TargetForward / 2;
    TargetTurning = TargetTurning / 2;

    return {TargetForward, TargetStraff, TargetTurning};
}
