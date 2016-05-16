#pragma once 

#define wheelCircumference 0.225
#define gearRatio (1/48.0)
#define ticksPerRotation 12
#define wheelDistance 0.245

double encoderToMeters(int16_t encTicks)
{
    return encTicks * wheelCircumference / ticksPerRotation * gearRatio;
}


double metersToEncoder(double meters)
{
    return meters / wheelCircumference / gearRatio * ticksPerRotation;
}