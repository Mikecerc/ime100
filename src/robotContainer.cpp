#include "main.h"
// controller object
Controller controller;

controllerValues target;

void RobotContainer(void *param)
{
    int auton = 0;
    controller.setText(1, 0, "A: 0");
    controller.setText(1, 8, "FC: T");
    ControllerButton armUpButton(ControllerDigital::A);
    ControllerButton armDownButton(ControllerDigital::B);

    // ControllerButton doorIn(ControllerDigital::X);
    // ControllerButton doorOut(ControllerDigital::Y);
    ControllerButton cycleAutonUp(ControllerDigital::right);
    ControllerButton cycleAutonDown(ControllerDigital::left);
    ControllerButton runAuton(ControllerDigital::up);

    ControllerButton clawIn(ControllerDigital::R1);
    ControllerButton clawOut(ControllerDigital::R2);

    ControllerButton slowButton(ControllerDigital::L2);
    ControllerButton zeroHeading(ControllerDigital::down);
    ControllerButton enableFieldCentric(ControllerDigital::L1);

    // ControllerButton tempDisableFieldCentric(ControllerDigital::L1);

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
        driveTrain->xArcade(target.straff, target.forward, target.turning, 0);

        // field centric drive control toggle button
        if (enableFieldCentric.changedToPressed())
        {
            Fieldcentric = !Fieldcentric;
            // update field centric toggle button state
            controller.setText(1, 8, Fieldcentric ? ", FC: T" : ", FC: F");
        }

        // temporarly disable field centric drive control function
        // tempNotFieldCentric = tempDisableFieldCentric.isPressed();

        // slow mode toggle button
        slowMode = slowButton.isPressed();

        // reset heading function
        if (zeroHeading.isPressed())
            resetHeading();

        if (armDownButton.isPressed())
        {
            armTarget = -20;
        }
        else if (armUpButton.isPressed())
        {
            armTarget = 30;
        }

        // claw control

        /*if (doorIn.isPressed())
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
        }*/
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

        // auton cycle
        if (cycleAutonUp.changedToPressed())
        {
            auton = auton + 1;
            if (auton > 2)
            {
                auton = 0;
            }
            std::string string = "A:" + std::to_string(auton);
            controller.setText(1, 0, string);
        }
        else if (cycleAutonDown.changedToPressed())
        {
            auton = auton - 1;
            if (auton < 0)
            {
                auton = 2;
            }
            std::string string = "A:" + std::to_string(auton);
            controller.setText(1, 0, string);
        }

        // run auton
        if (runAuton.isPressed())
        {
            std::cout << "Running Auton" << std::endl;
            // chassis->moveDistance(1_m);
            switch (auton)
            {
            case 0:
                autons::simpleForward(chassis.get());
                break;
            case 1:
                autons::sampleForward(chassis.get());
                break;
            };
        };

        // sleep at end of loop
        pros::delay(20);
    }
}