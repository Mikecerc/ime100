#include "main.h"

Motor armMotor(Constants::Port::ArmMotor, true, Constants::Arm::gearset, AbstractMotor::encoderUnits::degrees);
// std::shared_ptr<IterativePositionController<double, double>> armController;
// std::shared_ptr<AsyncPositionController<double,double>> armController;
RotationSensor armRotationalSensor(Constants::Port::armRotationalSensor);
// std::shared_ptr<RotationSensor> armRotationalSensorPtr = std::make_shared<RotationSensor>(armRotationalSensor);
double error, targetPower, armTarget, angle;
void initializeArm()
{
     //set arm motor to break mode
     armMotor.setBrakeMode(Constants::Arm::brakeMode);
}
void ArmOpcontrol(void *param)
{
     //define arm pid controller
     IterativePosPIDController armPID = IterativeControllerFactory::posPID(Constants::Arm::Gains::kP, Constants::Arm::Gains::kI, Constants::Arm::Gains::kD);
     armPID.setOutputLimits(-Constants::Arm::maxVoltage, Constants::Arm::maxVoltage);

     // [TEMPORARY] temp set arm target to 0 for testing 
     armTarget = 0;
     while (true)
     {
          //cap arm target angle to maxminum and minimum angles 
          armTarget = armTarget >= Constants::Arm::maxAngle ? Constants::Arm::maxAngle : armTarget;
          armTarget = armTarget <= Constants::Arm::minAngle ? Constants::Arm::minAngle : armTarget;

          //convert 0-360 scale to -180-180 scale
          angle = armRotationalSensor.get() <= 180 ? armRotationalSensor.get() : armRotationalSensor.get() - 360;
          
          //find the error between target angle and actual angle given by encoder
          error = armTarget - angle;

          //set the target power to the output of the pid controller
          targetPower = armPID.step(error);
          armMotor.moveVoltage(targetPower);
          
          //long to console
          std::cout << "Angle: " << angle << " Error: " << error << " Target Power: " << targetPower << std::endl;
          pros::delay(20);
     }
}
