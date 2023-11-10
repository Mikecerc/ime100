using namespace okapi;
#pragma once

extern Motor armMotor; 
extern std::shared_ptr<AsyncPositionController<double,double>> armController;
extern RotationSensor armRotationalSensor;
extern double armTarget;
extern void ArmOpcontrol(void* param);
extern void initializeArm();
