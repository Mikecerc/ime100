#pragma once

extern Motor leftFrontDrive;
extern Motor leftBackDrive;
extern Motor rightFrontDrive;
extern Motor rightBackDrive;
extern pros::Imu inertialSensor;
extern std::shared_ptr<ChassisController> chassis;
extern std::shared_ptr<okapi::XDriveModel> driveTrain;

extern bool Fieldcentric;
extern bool slowMode;

struct controllerValues {
    double forward;
    double straff;
    double turning; 
};

extern void ChassisOpcontrol(void* param);
extern void resetHeading();
extern controllerValues computeFieldCentricValues(double forward, double straff, double turning);

