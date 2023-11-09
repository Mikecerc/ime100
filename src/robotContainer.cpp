#include "main.h"
// controller object
Controller controller;

controllerValues target;

void RobotContainer(void *param)
{

    ControllerButton armUpButton(ControllerDigital::A);
    ControllerButton armDownButton(ControllerDigital::B);

    ControllerButton doorIn(ControllerDigital::X);
    ControllerButton doorOut(ControllerDigital::Y);
    ControllerButton clawIn(ControllerDigital::R1);
    ControllerButton clawOut(ControllerDigital::R2);

    ControllerButton slowButton(ControllerDigital::L2);
    ControllerButton zeroHeading(ControllerDigital::down);
    ControllerButton enableFieldCentric(ControllerDigital::left);
    ControllerButton runAuton(ControllerDigital::up);
    ControllerButton tempDisableFieldCentric(ControllerDigital::L1);

    chassisInitialize();

    // pros::Task ChassisOpcontrol_TR(ChassisOpcontrol, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "chassis subsystem");
    //  control loop
    while (true)
    {
        // drive control
        slowMode ? driveTrain->setMaxVelocity(Constants::Drivetrain::maxVelocity * Constants::Drivetrain::slowMultiplier) : driveTrain->setMaxVelocity(Constants::Drivetrain::maxVelocity);
        target = computeFieldCentricValues(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::leftX), controller.getAnalog(ControllerAnalog::rightX));
        // feed control input values to okapi chassis controller
        // std::cout << "Straff: " << target.straff << " Forward: " << target.forward << " Turning: " << target.turning << std::endl;
        while (inertialSensor.is_calibrating())
        {
            pros::delay(20);
        }
        std::cout << driveTrain << std::endl;
        driveTrain->xArcade(target.straff, target.forward, target.turning, 0);

        // field centric drive control toggle button
        if (enableFieldCentric.changedToPressed())
        {
            Fieldcentric = !Fieldcentric;
        }

        // temporarly disable field centric drive control function
        tempNotFieldCentric = tempDisableFieldCentric.isPressed();

        // slow mode toggle button
        slowMode = slowButton.isPressed();

        // reset heading function
        if (zeroHeading.isPressed())
        {
            resetHeading();
        }

        // arm control
        if (armDownButton.isPressed())
        {
            armMotor.moveVoltage(4000);
        }
        else if (armUpButton.isPressed())
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
        if (clawIn.isPressed())
        {
            clawMotor.moveVoltage(12000);
        }
        else if (clawOut.isPressed())
        {
            clawMotor.moveVoltage(-12000);
        }
        else
        {
            clawMotor.moveVoltage(0);
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