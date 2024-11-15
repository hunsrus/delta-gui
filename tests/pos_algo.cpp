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

#include <pigpio.h>

#define PIN_DIR1 6
#define PIN_STEP1 5
#define PIN_DIR2 18
#define PIN_STEP2 17
#define PIN_DIR3 22
#define PIN_STEP3 23

#define PIN_MS1 2
#define PIN_MS2 3
#define PIN_MS3 4

#define PIN_JOY_X0 19
#define PIN_JOY_X1 26
#define PIN_JOY_Y0 20
#define PIN_JOY_Y1 16
#define PIN_JOY_PB 21

#define ARM_LENGTH 130.0f
#define ROD_LENGTH 166.0f
#define BASS_TRI 35.0f
#define PLATFORM_TRI 115.45f
#define PLATFORM_POS (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}

#define TRANS_MULTIPLIER 3
#define STEPS_NUM 1
double STEP_ANGLE = 1.8/(STEPS_NUM*TRANS_MULTIPLIER*1.0);

static bool EXIT = false;

// Función para mover el motor de un ángulo actual a un ángulo objetivo
void moveToAngle(int motorID, double currentAngle, double targetAngle) {
    // Calcular la diferencia de ángulo
    double angleDiff = targetAngle - currentAngle;

    // Calcular el número de pasos necesarios
    int steps = (int)round(angleDiff/STEP_ANGLE);

    // Determinar la dirección del movimiento
    bool direction = (steps > 0) ? 1 : 0;

    // Hacer los pasos necesarios
    if(motorID = 1)
    {
        gpioWrite(PIN_DIR1, direction);
        for (int i = 0; i < abs(steps); i++)
        {
            gpioWrite(PIN_STEP1, 1);
            usleep(1500);
            gpioWrite(PIN_STEP1, 0);
        }
    }
    if(motorID = 2)
    {
        gpioWrite(PIN_DIR2, direction);
        for (int i = 0; i < abs(steps); i++)
        {
            gpioWrite(PIN_STEP2, 1);
            usleep(1500);
            gpioWrite(PIN_STEP2, 0);
        }
    }
    if(motorID = 3)
    {
        gpioWrite(PIN_DIR3, direction);
        for (int i = 0; i < abs(steps); i++)
        {
            gpioWrite(PIN_STEP3, 1);
            usleep(1500);
            gpioWrite(PIN_STEP3, 0);
        }
    }
}

int main(int argc, char** argv)
{
    DeltaKinematics dk = DeltaKinematics(ARM_LENGTH, ROD_LENGTH, BASS_TRI, PLATFORM_TRI);
    double x = 0, y = 0, z = -ROD_LENGTH/2.0f;
    double lastX = -1, lastY = -1, lastZ = -1;
    double lastA, lastB, lastC;
    double thetaA, thetaB, thetaC;

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

    dk.inverse(x,y,z);
    lastA = dk.a;
    lastB = dk.b;
    lastC = dk.c;

    unsigned int timestep = 0;

    const std::chrono::milliseconds targetPeriod = std::chrono::milliseconds(10);
    std::chrono::milliseconds elapsedTime = std::chrono::milliseconds(0);
    std::chrono::milliseconds calcTime = std::chrono::milliseconds(0);
    std::chrono::milliseconds moveTime = std::chrono::milliseconds(0);
    std::chrono::high_resolution_clock::time_point time0;
    std::chrono::high_resolution_clock::time_point time1;
    std::chrono::high_resolution_clock::time_point initTime = std::chrono::high_resolution_clock::now();

    while(!EXIT)
    {
        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        x = sin(timestep*0.5f)*30.0f;
        y = cos(timestep*0.5f)*30.0f;

        if(lastX != x || lastY != y || lastZ != z) // calcula solo si hubo variaciones
        {
            // Cálculos de cinemática
            time0 = std::chrono::high_resolution_clock::now();
            dk.inverse(x,y,z);
            time1 = std::chrono::high_resolution_clock::now();
            calcTime = std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0);

            time0 = std::chrono::high_resolution_clock::now();
            if(fabs(dk.a - lastA) > STEP_ANGLE)
            {
                moveToAngle(1,lastA, dk.a);
                lastA = dk.a;
            }
            if(fabs(dk.b - lastB) > STEP_ANGLE)
            {
                moveToAngle(2,lastB, dk.b);
                lastB = dk.b;
            }
            if(fabs(dk.c - lastC) > STEP_ANGLE)
            {
                moveToAngle(3,lastC, dk.c);
                lastC = dk.c;
            }
            time1 = std::chrono::high_resolution_clock::now();
            moveTime = std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0);

        }

        lastX = x;
        lastY = y;
        lastZ = z;

        std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();
        elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        std::cout << "ELA_T " << elapsedTime.count() << "[ms]" << std::endl;
        std::cout << "CAL_T " << calcTime.count() << "[ms]" << std::endl;
        std::cout << "MOV_T " << moveTime.count() << "[ms]" << std::endl;

        if (elapsedTime < targetPeriod) {
            // Espera el tiempo restante para completar el periodo deseado
            std::this_thread::sleep_for(targetPeriod - elapsedTime);
        } else {
            // Si la iteración tardó más tiempo del permitido
            std::cerr << "Advertencia: Iteración excedió el tiempo deseado!" << std::endl;
        }
        
        timestep++;
    }

    return EXIT_SUCCESS;
}