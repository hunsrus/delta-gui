#ifndef OCTO_KINEMATICS_H
#define OCTO_KINEMATICS_H

#include <math.h>
#include <stdio.h>

#define MOTOR_OFFSET_Z 0
#define MOTOR_ANGLE_MIN -M_PI/2
#define MOTOR_ANGLE_MAX M_PI/2

class OctoKinematics
{
    public:
        OctoKinematics(double armLength,double rodLength,double effRadius,double basRadius);
        bool inverse_kinematics(float xt, float yt, float zt);
        void set_axis_direction(bool dir);

        double _x = 0;
        double _y = 0;
        double _z = 0;
        double a = 0;
        double b = 0;
        double c = 0;

    private:
        double armLength;
        double rodLength;
        double effRadius;
        double basRadius;

        bool axis_direction = 1;

        double motorOffsetX;

        bool inverse_kinematics_1(float xt, float yt, float zt);
        bool inverse_kinematics_2(float xt, float yt, float zt);
        bool inverse_kinematics_3(float xt, float yt, float zt);
};

#endif //OCTO_KINEMATICS_H