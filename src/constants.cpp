#include "main.h"

// motor port assisngments
const int Constants::Port::LeftFrontDrive = 20;
const int Constants::Port::LeftBackDrive = 19;
const int Constants::Port::RightFrontDrive = 16;
const int Constants::Port::RightBackDrive = 14;
const int Constants::Port::ArmMotor = 18;
const int Constants::Port::ClawMotor = 15;
const int Constants::Port::ClawDoorMotor = 17;

// sensor port assignments
const int Constants::Port::InertialSensor = 11;
const int Constants::Port::IntertialSensor2 = 12;
const int Constants::Port::armRotationalSensor = 13;

// Drivetrain constants
const ChassisScales Constants::Drivetrain::chassisDimensions = {{4_in, 12_in}, imev5GreenTPR};
 
const AbstractMotor::gearset Constants::Drivetrain::gearset = AbstractMotor::gearset::green;
const int Constants::Drivetrain::maxVelocity = 90;
const float Constants::Drivetrain::slowMultiplier = 0.5;

// Arm constants
const AbstractMotor::gearset Constants::Arm::gearset = AbstractMotor::gearset::red;
const AbstractMotor::brakeMode Constants::Arm::brakeMode = AbstractMotor::brakeMode::hold;
const double Constants::Arm::encoderOffset = 0;
const double Constants::Arm::maxVelocity = 100;
// arm gains
const double Constants::Arm::Gains::kP = 0.5;
const double Constants::Arm::Gains::kI = 0.0;
const double Constants::Arm::Gains::kD = 0.0;
const double Constants::Arm::Gains::kBias = 0.0;
