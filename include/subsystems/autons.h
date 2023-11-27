using namespace okapi;
#pragma once
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
        clawDoorMotor.moveVoltage(-4000);
        pros::delay(1500);
        clawMotor.moveVoltage(Constants::claw::holdPower::voltage);

        pros::delay(1000);
         armTarget = Constants::Arm::SetPoints::mid;
        pros::delay(1000);
        chassis->moveDistance(-0.5_m);
        driveTrain->strafe(1);
        chassis->waitUntilSettled();
        pros::delay(2000);
        driveTrain->strafe(0);
        driveTrain->left(1);
        driveTrain->right(-1);
        pros::delay(2300);
        driveTrain->left(0);
        driveTrain->right(0);
        armTarget = Constants::Arm::SetPoints::high;
        pros::delay(2000);
        chassis->moveDistance(1_m);
        clawMotor.moveVoltage(-12000);
        clawDoorMotor.moveVoltage(12000);
        pros::delay(1000);
        clawMotor.moveVoltage(0);
        clawDoorMotor.moveVoltage(0);
    }
    static void sample2(okapi::ChassisController *Chassis)
    {
           
    }

};