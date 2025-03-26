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
        OctoKinematics(float armLength, float rodLength, float effRadius, float basRadius);
        int step(int step_pin, int dir_pin, bool dir);
        void step_eff(bool dir);
        int home(float x, float y, float z);
        bool inverse_kinematics(float x, float y, float z);
        int updateKinematics(void);
        int linear_move(float x, float y, float z, float stepDist, int stepDelay);
        int linear_move_eased(float x, float y, float z, float stepDist, int stepDelay);
        void rotate_effector(float angle);
        void correct_effector_init(int steps);
        void set_axis_direction(bool dir);
        void set_step_precision(int stepsNum);
        void set_effector_precision(int stepsNum);
        void set_transmission_ratio(float transRatio);
        void set_starting_z(float z);
        void set_pulse_width(useconds_t us);
        void set_pulse_width_effector(useconds_t us);

        void set_pin_step_ctrl(unsigned int ms1, unsigned int ms2, unsigned int ms3);
        void set_pin_motor_1(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_motor_2(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_motor_3(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_motor_effector(unsigned int step_pin, unsigned int dir_pin);
        void set_pin_limit_sw(unsigned int ls1, unsigned int ls2, unsigned int ls3);

        float x = 0;
        float y = 0;
        float z = 0;
        float a = 0;
        float b = 0;
        float c = 0;

        bool ls_hit = false;

    private:
        float armLength;
        float rodLength;
        float effRadius;
        float basRadius;

        unsigned int pin_ms1, pin_ms2, pin_ms3;
        unsigned int pin_step1, pin_step2, pin_step3, pin_step_eff;
        unsigned int pin_dir1, pin_dir2, pin_dir3, pin_dir_eff;
        unsigned int pin_ls1, pin_ls2, pin_ls3;

        bool axis_direction = 1;

        float motorOffsetX;

        float starting_z = 0;

        float lastA = 0;
        float lastB = 0;
        float lastC = 0;

        float trans_ratio;
        float steps_num;
        float step_angle = 0;
        float steps_num_eff = 2;
        float step_angle_eff = 0;

        useconds_t pulse_width;
        useconds_t pulse_width_eff;

        bool inverse_kinematics_1(float xt, float yt, float zt);
        bool inverse_kinematics_2(float xt, float yt, float zt);
        bool inverse_kinematics_3(float xt, float yt, float zt);
};

#endif //OCTO_KINEMATICS_H