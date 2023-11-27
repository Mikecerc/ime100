#include "main.h"

// claw
Motor clawMotor(Constants::Port::ClawMotor);
Motor clawDoorMotor(Constants::Port::ClawDoorMotor);
bool holdPowerOn = false;
bool doorHoldPowerOn = false;

void ManipulatorOpcontrol(void* param) {
    clawDoorMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
}