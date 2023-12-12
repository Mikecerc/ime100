#include "main.h"
#include "subsystems/autons.h"
void turnAngle(okapi::ChassisController *Chassis, double targetAngle_deg, pros::Imu inertialSensor)
{
    double leftPower, rightPower;
    double currentAngle, error;
    IterativePosPIDController turnPID = IterativeControllerFactory::posPID(0.005, 0, 0);
    targetAngle_deg = targetAngle_deg + inertialSensor.get_heading();
    if (targetAngle_deg > 360)
    {
        targetAngle_deg = targetAngle_deg - 360;
    }
    else if (targetAngle_deg < 0)
    {
        targetAngle_deg = targetAngle_deg + 360;
    }
    while (inertialSensor.get_heading() < targetAngle_deg - 5 || inertialSensor.get_heading() > targetAngle_deg + 5)
    {
        currentAngle = inertialSensor.get_heading();
        error = targetAngle_deg - currentAngle;
        leftPower = turnPID.step(error);
        driveTrain->tank(leftPower, -leftPower);
        pros::delay(20);
        std::cout << "Current Angle: " << currentAngle << " Target Angle: " << targetAngle_deg << " Error: " << error << " Left Power: " << leftPower << " Right Power: " << rightPower << std::endl;
    }
}
