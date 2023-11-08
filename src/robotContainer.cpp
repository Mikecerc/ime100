#include "main.h"
// controller object
Controller controller;

// arm
Motor armMotor(Constants::Port::ArmMotor, true, Constants::Arm::gearset, AbstractMotor::encoderUnits::degrees);

// drivetrain
double forward, straff, turning;
controllerValues target;

void RobotContainer(void *param)
{
    armMotor.setBrakeMode(Constants::Arm::brakeMode);

    ControllerButton armUpButton(ControllerDigital::A);
    ControllerButton armDownButton(ControllerDigital::B);

    ControllerButton doorIn(ControllerDigital::R1);
    ControllerButton doorOut(ControllerDigital::R2);

    ControllerButton slowButton(ControllerDigital::L2);
    ControllerButton zeroHeading(ControllerDigital::down);
    ControllerButton enableFieldCentric(ControllerDigital::left);
    ControllerButton runAuton(ControllerDigital::up);
    ControllerButton tempDisableFieldCentric(ControllerDigital::L1);

    // control loop
    while (true)
    {
        // drive control

        target = computeFieldCentricValues(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::leftX), controller.getAnalog(ControllerAnalog::rightX));
        // feed control input values to okapi chassis controller
        driveTrain->xArcade(target.straff, target.forward, target.turning, 0);

        // field centric drive control toggle button
        if (enableFieldCentric.changedToPressed())
        {
            Fieldcentric = !Fieldcentric;    
        }

        // temporarly disable field centric drive control function
        if (tempDisableFieldCentric.isPressed() && Fieldcentric == true)
        {
            Fieldcentric == false;
        }
        else if (Fieldcentric == false)
        {
            Fieldcentric == true;
        }

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