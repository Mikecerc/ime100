#include "main.h"

Motor armMotor(Constants::Port::ArmMotor, true, Constants::Arm::gearset, AbstractMotor::encoderUnits::degrees);

void ArmOpcontrol(void* param) {
     armMotor.setBrakeMode(Constants::Arm::brakeMode);
}
