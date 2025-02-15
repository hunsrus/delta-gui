#ifndef OCTO_KINEMATICS_H
#define OCTO_KINEMATICS_H

// detección de la arquitectura
#if defined(__aarch64__) || defined(_M_ARM64) || !defined(__x86_64__) || !defined(__x86_64__)
    #define ARCH_ARM true
#else
    #define ARCH_ARM false
#endif

#include <math.h>
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <iostream>

#define MOTOR_OFFSET_Z 0
#define MOTOR_ANGLE_MIN -M_PI/2
#define MOTOR_ANGLE_MAX M_PI/2

static bool IGNORE_LIMIT_SWITCHES = false;

class OctoKinematics
{
    public:
        OctoKinematics(double armLength,double rodLength,double effRadius,double basRadius);
        int step(int step_pin, int dir_pin, bool dir);
        int home(float x, float y, float z);
        bool inverse_kinematics(float x, float y, float z);
        int updateKinematics(void);
        int linear_move(float x, float y, float z, float stepDist, int stepDelay);
        void set_axis_direction(bool dir);
        void set_step_precision(int stepsNum);
        void set_transmission_ratio(double transRatio);
        void set_pulse_width(useconds_t us);

        void set_pin_step_ctrl(unsigned int ms1, unsigned int ms2, unsigned int ms3);
        void set_pin_motor_1(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_motor_2(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_motor_3(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_limit_sw(unsigned int ls1, unsigned int ls2, unsigned int ls3);

        double x = 0;
        double y = 0;
        double z = 0;
        double a = 0;
        double b = 0;
        double c = 0;

        bool ls_hit = false;

    private:
        double armLength;
        double rodLength;
        double effRadius;
        double basRadius;

        unsigned int pin_ms1, pin_ms2, pin_ms3;
        unsigned int pin_step1, pin_step2, pin_step3;
        unsigned int pin_dir1, pin_dir2, pin_dir3;
        unsigned int pin_ls1, pin_ls2, pin_ls3;

        bool axis_direction = 1;

        double motorOffsetX;

        double lastA = 0;
        double lastB = 0;
        double lastC = 0;

        double trans_ratio;
        double steps_num;
        double step_angle = 0;

        useconds_t pulse_width;

        bool inverse_kinematics_1(float xt, float yt, float zt);
        bool inverse_kinematics_2(float xt, float yt, float zt);
        bool inverse_kinematics_3(float xt, float yt, float zt);
};

#endif //OCTO_KINEMATICS_H