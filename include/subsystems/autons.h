using namespace okapi;
#pragma once
extern void turnAngle(okapi::ChassisController *Chassis, double targetAngle_deg, pros::Imu inertialSensor);
class autons
{
public:
    static void simpleForward(okapi::ChassisController *Chassis)
    {
        Chassis->moveDistance(1_m);
    }
    static void sampleForward(okapi::ChassisController *Chassis)
    {
        clawMotor.moveVoltage(-12000);
        pros::delay(1500);
        clawMotor.moveVoltage(Constants::claw::holdPower::voltage);
        driveTrain->left(-0.2);
        driveTrain->right(-0.2);
        pros::delay(2000);
        driveTrain->left(0);
        driveTrain->right(0);
        clawDoorMotor.moveVoltage(-4000);
        pros::delay(2000);
        clawDoorMotor.moveVoltage(-2500);
        armTarget = Constants::Arm::SetPoints::mid;
        pros::delay(1000);
        driveTrain->left(-0.7);
        driveTrain->right(-0.7);
        pros::delay(500);
        //turnAngle(Chassis, 180, inertialSensor);
        driveTrain->left(1);
        driveTrain->right(-1);
        pros::delay(2500);
        driveTrain->left(0);
        driveTrain->right(0);
        pros::delay(1000);
        driveTrain->strafe(-0.8);
        pros::delay(2000);
        armTarget = Constants::Arm::SetPoints::high;
        pros::delay(2000);
        driveTrain->strafe(0);
        //chassis->moveDistance(1_m);
        driveTrain->left(0.9);
        driveTrain->right(0.9);
        pros::delay(2000);
        clawMotor.moveVoltage(12000);
        clawDoorMotor.moveVoltage(12000);
        pros::delay(1000);
        clawMotor.moveVoltage(0);
        clawDoorMotor.moveVoltage(0);
    }
    static void sample2(okapi::ChassisController *Chassis)
    {
clawMotor.moveVoltage(-12000);
        pros::delay(1500);
        clawMotor.moveVoltage(Constants::claw::holdPower::voltage);
        driveTrain->left(-0.2);
        driveTrain->right(-0.2);
        pros::delay(2000);
        driveTrain->left(0);
        driveTrain->right(0);
        clawDoorMotor.moveVoltage(-4000);
        pros::delay(2000);
        clawDoorMotor.moveVoltage(-2500);
        armTarget = Constants::Arm::SetPoints::mid;
        pros::delay(1000);
        driveTrain->left(-0.7);
        driveTrain->right(-0.7);
        pros::delay(500);
        //turnAngle(Chassis, 180, inertialSensor);
        driveTrain->left(1);
        driveTrain->right(-1);
        pros::delay(2500);
        driveTrain->left(0);
        driveTrain->right(0);
        pros::delay(1000);
        driveTrain->strafe(0.8);
        pros::delay(2000);
        armTarget = Constants::Arm::SetPoints::high;
        pros::delay(2000);
        driveTrain->strafe(0);
        //chassis->moveDistance(1_m);
        driveTrain->left(0.9);
        driveTrain->right(0.9);
        pros::delay(2000);
        clawMotor.moveVoltage(12000);
        clawDoorMotor.moveVoltage(12000);
        pros::delay(1000);
        clawMotor.moveVoltage(0);
        clawDoorMotor.moveVoltage(0);
           
    }

};