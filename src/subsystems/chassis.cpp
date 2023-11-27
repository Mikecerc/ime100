#include "main.h"

// field centric drive control
int newHeading = 0;
int chassisHeading, headingError, temp;
float theta;
bool Fieldcentric = true;
bool tempNotFieldCentric = false;
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
void chassisInitialize()
{
    inertialSensor.reset();
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
}
void ChassisOpcontrol(void *param)
{
    // reset IMU
    // inertialSensor.reset();

    // Chassis Controller - lets us drive the robot around with open- or closed-loop control

    // wait for IMU to calibrate
    while (inertialSensor.is_calibrating())
    {
        pros::delay(20);
    }

    // control loop
    while (true)
    {
        
        pros::delay(30);
    }
}
void resetHeading()
{
    headingError = inertialSensor.get_heading();
}
controllerValues computeFieldCentricValues(double forward, double straff, double turning)
{
    // update heading based off IMU reading
    chassisHeading = inertialSensor.get_heading() - headingError;

    // L floating point math moment
    forward = forward * 100;
    straff = straff * 100;
    turning = turning * 100;

    // coordinate transform.
    theta = (-chassisHeading * 3.1415926535) / 180;
    temp = forward * cos(theta) - straff * sin(theta);
    straff = forward * sin(theta) + straff * cos(theta);
    forward = temp;

    // set target stick values based off whether or not field centric mode is enabled
    double TargetStraff = (Fieldcentric && !tempNotFieldCentric) ? straff / 100 : controller.getAnalog(ControllerAnalog::leftX);
    double TargetForward = (Fieldcentric && !tempNotFieldCentric) ? forward / 100 : controller.getAnalog(ControllerAnalog::leftY);
    double TargetTurning = (Fieldcentric && !tempNotFieldCentric) ? turning / 100 : controller.getAnalog(ControllerAnalog::rightX);

    // scale values to slow mode multiplier
    TargetStraff = slowMode ? TargetStraff * Constants::Drivetrain::slowMultiplier : TargetStraff;
    TargetForward = slowMode ? TargetForward * Constants::Drivetrain::slowMultiplier : TargetForward;
    TargetTurning = slowMode ? TargetTurning * Constants::Drivetrain::slowMultiplier : TargetTurning;

    return {TargetForward, TargetStraff, TargetTurning};
}
