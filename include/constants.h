using namespace okapi;
class Constants
{
public:
    class Drivetrain
    {
    public:
        static const ChassisScales chassisDimensions;
        static const AbstractMotor::gearset gearset;
        static const int maxVelocity;
        static const float slowMultiplier;
    };
    class Arm
    {
    public:
        static const AbstractMotor::gearset gearset;
        static const AbstractMotor::brakeMode brakeMode;
        class Gains
        {
        public:
            static const double kP;
            static const double kI;
            static const double kD;
            static const double kBias;
        };
        static const double encoderOffset;
        static const double maxVelocity;
    };
    class Port
    {
    public:
        static const int LeftFrontDrive;
        static const int LeftBackDrive;
        static const int RightFrontDrive;
        static const int RightBackDrive;
        static const int ArmMotor;
        static const int ClawMotor;
        static const int ClawDoorMotor;
        static const int InertialSensor;
        static const int IntertialSensor2;
        static const int armRotationalSensor;
        static const int leftTrackingEncoder;
        static const int leftTrackingEncoder2;
        static const int rightTrackingEncoder;
        static const int rightTrackingEncoder2;
        static const int backTrackingEncoder;
        static const int backTrackingEncoder2;
    };
};
