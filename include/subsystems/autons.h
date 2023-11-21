using namespace okapi;
#pragma once
class autons
{
public:
    static void simpleForward(okapi::ChassisController *Chassis)
    {
        Chassis->moveDistance(1_m);
    }
    static void sampleForward(okapi::ChassisController *Chassis)
    {
        Chassis->moveDistance(2_m);
    }
    static void sample2(okapi::ChassisController *Chassis)
    {
            Chassis->moveDistance(0.5_m);
            Chassis->moveDistance(-0.5_m);
            Chassis->turnAngle(90_deg);
            Chassis->moveDistance(4.5_ft);
            Chassis->turnAngle(-90_deg);
            Chassis->moveDistance(0.8_m); 
    }

};