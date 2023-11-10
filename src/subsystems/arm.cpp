#include "main.h"

Motor armMotor(Constants::Port::ArmMotor, true, Constants::Arm::gearset, AbstractMotor::encoderUnits::degrees);
// std::shared_ptr<IterativePositionController<double, double>> armController;
// std::shared_ptr<AsyncPositionController<double,double>> armController;
RotationSensor armRotationalSensor(Constants::Port::armRotationalSensor);
// std::shared_ptr<RotationSensor> armRotationalSensorPtr = std::make_shared<RotationSensor>(armRotationalSensor);
double error, targetPower, armTarget, angle;
void initializeArm()
{
     armMotor.setBrakeMode(Constants::Arm::brakeMode);
     /*
     armController = AsyncPosControllerBuilder()
                          .withMotor(armMotor)
                          .withGearset(Constants::Arm::gearset)
                          .withGains({Constants::Arm::Gains::kP, Constants::Arm::Gains::kI, Constants::Arm::Gains::kD})
                          .withSensor(armRotationalSensorPtr)
                          .withMaxVelocity(5)
                          .build();
     std::cout << armController->getTarget() << armRotationalSensor.controllerGet() << std::endl;
     armController->setTarget(0);
     */
     // armController = IterativeControllerFactory::posPID(Constants::Arm::Gains::kP, Constants::Arm::Gains::kI, Constants::Arm::Gains::kD, Constants::Arm::Gains::kBias);
}
void ArmOpcontrol(void *param)
{
     auto armPID = IterativeControllerFactory::posPID(Constants::Arm::Gains::kP, Constants::Arm::Gains::kI, Constants::Arm::Gains::kD);
     armTarget = 0;
     while (true)
     {
          angle = armRotationalSensor.get() <= 180 ? armRotationalSensor.get() : armRotationalSensor.get() - 360;
          
          error = armTarget - angle;
          targetPower = armPID.step(error) * 12000;
          armMotor.moveVoltage(targetPower);
          std::cout << armMotor.getPower() << std::endl;
          std::cout << "Angle: " << angle << " Error: " << error << " Target Power: " << targetPower << std::endl;
          pros::delay(20);
     }
}
