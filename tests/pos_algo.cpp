// detección de la arquitectura
#if defined(__aarch64__) || defined(_M_ARM64) || !defined(__x86_64__) || !defined(__x86_64__)
    #define ARCH_ARM true
#else
    #define ARCH_ARM false
#endif

#include "../DeltaKinematics.h"
#include <iostream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <list>

#include <pigpio.h>

#define PIN_DIR1 6
#define PIN_STEP1 5
#define PIN_DIR2 22
#define PIN_STEP2 23
#define PIN_DIR3 18
#define PIN_STEP3 17

#define PIN_MS1 2
#define PIN_MS2 3
#define PIN_MS3 4

#define PIN_JOY_X0 19
#define PIN_JOY_X1 26
#define PIN_JOY_Y0 20
#define PIN_JOY_Y1 16
#define PIN_JOY_PB 21

#define PIN_FC_M1 12
#define PIN_FC_M2 13
#define PIN_FC_M3 27

#define PIN_BOMBA 0

#define ARM_LENGTH 130.0f
#define ROD_LENGTH 166.0f
#define BASS_TRI 35.0f
#define PLATFORM_TRI 115.45f
#define PLATFORM_POS (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}

#define TRANS_MULTIPLIER 3
#define STEPS_NUM 4
double STEP_ANGLE = 1.8/(STEPS_NUM*TRANS_MULTIPLIER*1.0);

static bool EXIT = false;
static int ERROR = 0;

std::chrono::milliseconds elapsedTime = std::chrono::milliseconds(0);
std::chrono::milliseconds calcTime = std::chrono::milliseconds(0);
std::chrono::milliseconds loadTime = std::chrono::milliseconds(0);
std::chrono::milliseconds stepTime = std::chrono::milliseconds(0);
std::chrono::high_resolution_clock::time_point time0;
std::chrono::high_resolution_clock::time_point time1;

void step(int step_pin, int dir_pin, bool dir)
{
    gpioWrite(dir_pin, dir);
    gpioWrite(step_pin, 1);
    usleep(1500);
    gpioWrite(step_pin, 0);
}

int home(void)
{
    bool m1_ready = false;
    bool m2_ready = false;
    bool m3_ready = false;

    fprintf(stdout, "Homing...");
    // paso completo
    gpioWrite(PIN_MS1,0);
    gpioWrite(PIN_MS2,0);
    gpioWrite(PIN_MS3,0);

    while(!m1_ready || !m2_ready || !m3_ready)
    {
        if(!gpioRead(PIN_FC_M1))
        {
            step(PIN_STEP1, PIN_DIR1, 1);
        }else{
            m1_ready = true;
            fprintf(stdout, " M1 ready...");
        }

        if(!gpioRead(PIN_FC_M2))
        {
            step(PIN_STEP2, PIN_DIR2, 1);
        }else{
            m2_ready = true;
            fprintf(stdout, " M2 ready...\n");
        }

        if(!gpioRead(PIN_FC_M3))
        {
            step(PIN_STEP3, PIN_DIR3, 1);
        }else{
            m3_ready = true;
            fprintf(stdout, " M3 ready...\n");
        }
    }

    fprintf(stdout, "Homing complete\n");

    return EXIT_SUCCESS;
}

void updateKinematics(DeltaKinematics dk, double *lastA, double *lastB, double *lastC)
{
    double diffA, diffB, diffC;

    time0 = std::chrono::high_resolution_clock::now();
    bool reached = false;

    while(!reached)
    {
        diffA = dk.a - *lastA;
        diffB = dk.b - *lastB;
        diffC = dk.c - *lastC;

        reached = true;
        if(diffA > STEP_ANGLE)
        {
            *lastA += STEP_ANGLE;
            step(PIN_STEP1, PIN_DIR1, 1);
            reached = false;
        }else if(diffA < -STEP_ANGLE)
        {
            *lastA -= STEP_ANGLE;
            step(PIN_STEP1, PIN_DIR1, 0);
            reached = false;
        }
        if(diffB > STEP_ANGLE)
        {
            *lastB += STEP_ANGLE;
            step(PIN_STEP2, PIN_DIR2, 1);
            reached = false;
        }else if(diffB < -STEP_ANGLE)
        {
            *lastB -= STEP_ANGLE;
            step(PIN_STEP2, PIN_DIR2, 0);
            reached = false;
        }
        if(diffC > STEP_ANGLE)
        {
            *lastC += STEP_ANGLE;
            step(PIN_STEP3, PIN_DIR3, 1);
            reached = false;
        }else if(diffC < -STEP_ANGLE)
        {
            *lastC -= STEP_ANGLE;
            step(PIN_STEP3, PIN_DIR3, 0);
            reached = false;
        }
    }
    time1 = std::chrono::high_resolution_clock::now();
    loadTime = std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0);
}

int main(int argc, char** argv)
{
    DeltaKinematics dk = DeltaKinematics(ARM_LENGTH, ROD_LENGTH, BASS_TRI, PLATFORM_TRI);
    double x = 0, y = 0, z = -ROD_LENGTH;
    double lastX = -1, lastY = -1, lastZ = -1;
    double lastA, lastB, lastC;
    double thetaA, thetaB, thetaC;

    bool mode = 0;

    // inicio control de i/o
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }
    fprintf(stdout, "pigpio initialisation complete\n");

    gpioSetMode(PIN_DIR1,PI_OUTPUT);
    gpioSetMode(PIN_STEP1,PI_OUTPUT);
    gpioSetMode(PIN_DIR2,PI_OUTPUT);
    gpioSetMode(PIN_STEP2,PI_OUTPUT);
    gpioSetMode(PIN_DIR3,PI_OUTPUT);
    gpioSetMode(PIN_STEP3,PI_OUTPUT);
    
    gpioSetMode(PIN_MS1,PI_OUTPUT);
    gpioSetMode(PIN_MS2,PI_OUTPUT);
    gpioSetMode(PIN_MS3,PI_OUTPUT);

    gpioSetMode(PIN_JOY_X0,PI_INPUT);
    gpioSetMode(PIN_JOY_X1,PI_INPUT);
    gpioSetMode(PIN_JOY_Y0,PI_INPUT);
    gpioSetMode(PIN_JOY_Y1,PI_INPUT);
    gpioSetMode(PIN_JOY_PB,PI_INPUT);

    gpioSetMode(PIN_FC_M1,PI_INPUT);
    gpioSetMode(PIN_FC_M2,PI_INPUT);
    gpioSetMode(PIN_FC_M3,PI_INPUT);

    gpioSetMode(PIN_BOMBA,PI_OUTPUT);

    home();

    dk.inverse(x,y,z);
    lastA = dk.a;
    lastB = dk.b;
    lastC = dk.c;

    if(STEPS_NUM == 1)
    {
        gpioWrite(PIN_MS1,0);
        gpioWrite(PIN_MS2,0);
        gpioWrite(PIN_MS3,0);
    }else if(STEPS_NUM == 2){
        gpioWrite(PIN_MS1,1);
        gpioWrite(PIN_MS2,0);
        gpioWrite(PIN_MS3,0);
    }else if(STEPS_NUM == 4){
        gpioWrite(PIN_MS1,0);
        gpioWrite(PIN_MS2,1);
        gpioWrite(PIN_MS3,0);
    }else if(STEPS_NUM == 8){
        gpioWrite(PIN_MS1,1);
        gpioWrite(PIN_MS2,1);
        gpioWrite(PIN_MS3,0);
    }else if(STEPS_NUM == 16){
        gpioWrite(PIN_MS1,1);
        gpioWrite(PIN_MS2,1);
        gpioWrite(PIN_MS3,1);
    }

    z += 50;

    dk.inverse(x,y,z);
    updateKinematics(dk, &lastA, &lastB, &lastC);
    lastX = x;
    lastY = y;
    lastZ = z;

    unsigned int timestep = 0;

    const std::chrono::milliseconds targetPeriod = std::chrono::milliseconds(10);
    std::chrono::high_resolution_clock::time_point initTime = std::chrono::high_resolution_clock::now();

    while(!EXIT)
    {
        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        if(gpioRead(PIN_FC_M1) || gpioRead(PIN_FC_M2) || gpioRead(PIN_FC_M3))
        {
            ERROR = 1;
            std::cout << "[ERROR] Contacto con final de carrera" << std::endl;
        }

        // centro x0=1 x1=0 / y0=1 y1=0
        if(gpioRead(PIN_JOY_X0) == 0 && gpioRead(PIN_JOY_X1) == 0) //izq
        {
            x -= 1.0f;
        }
        if(gpioRead(PIN_JOY_X0) == 1 && gpioRead(PIN_JOY_X1) == 1) //der
        {
            x += 1.0f;
        }
        if(gpioRead(PIN_JOY_Y0) == 1 && gpioRead(PIN_JOY_Y1) == 1) //abajo
        {
            y -= 1.0f;
        }
        if(gpioRead(PIN_JOY_Y0) == 0 && gpioRead(PIN_JOY_Y1) == 0) //arriba
        {
            y += 1.0f;
        }

        if(gpioRead(PIN_JOY_PB)) //reset
        {
            ERROR = -1; //haciendo homing
            
            home();

            dk.inverse(x,y,z);
            lastA = dk.a;
            lastB = dk.b;
            lastC = dk.c;

            if(STEPS_NUM == 1)
            {
                gpioWrite(PIN_MS1,0);
                gpioWrite(PIN_MS2,0);
                gpioWrite(PIN_MS3,0);
            }else if(STEPS_NUM == 2){
                gpioWrite(PIN_MS1,1);
                gpioWrite(PIN_MS2,0);
                gpioWrite(PIN_MS3,0);
            }else if(STEPS_NUM == 4){
                gpioWrite(PIN_MS1,0);
                gpioWrite(PIN_MS2,1);
                gpioWrite(PIN_MS3,0);
            }else if(STEPS_NUM == 8){
                gpioWrite(PIN_MS1,1);
                gpioWrite(PIN_MS2,1);
                gpioWrite(PIN_MS3,0);
            }else if(STEPS_NUM == 16){
                gpioWrite(PIN_MS1,1);
                gpioWrite(PIN_MS2,1);
                gpioWrite(PIN_MS3,1);
            }

            z += 50;

            dk.inverse(x,y,z);
            updateKinematics(dk, &lastA, &lastB, &lastC);
            lastX = x;
            lastY = y;
            lastZ = z;

            ERROR = 0;
        }

        // mode = gpioRead(PIN_JOY_PB);

        // if(mode == 1)
        // {
        //     x = sin(timestep*0.05f)*30.0f;
        //     y = cos(timestep*0.05f)*30.0f;
        // }

        if(!ERROR)
        {
            if(lastX != x || lastY != y || lastZ != z) // calcula solo si hubo variaciones
            {
                
                // Cálculos de cinemática
                time0 = std::chrono::high_resolution_clock::now();
                dk.inverse(x,y,z);
                time1 = std::chrono::high_resolution_clock::now();
                calcTime = std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0);

                updateKinematics(dk, &lastA, &lastB, &lastC);

                std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();
                elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

                std::cout << "ELAP_T " << elapsedTime.count() << "[ms]" << std::endl;
                std::cout << "CALC_T " << calcTime.count() << "[ms]" << std::endl;
                std::cout << "LOAD_T " << loadTime.count() << "[ms]" << std::endl;
                std::cout << "STEP_T " << stepTime.count() << "[ms]" << std::endl;

                if (elapsedTime < targetPeriod) {
                    // Espera el tiempo restante para completar el periodo deseado
                    //std::this_thread::sleep_for(targetPeriod - elapsedTime);
                } else {
                    // Si la iteración tardó más tiempo del permitido
                    //std::cerr << "Advertencia: Iteración excedió el tiempo deseado!" << std::endl;
                }
            }
        }

        lastX = x;
        lastY = y;
        lastZ = z;
        
        timestep++;
    }

    return EXIT_SUCCESS;
}