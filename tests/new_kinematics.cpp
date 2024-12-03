// detección de la arquitectura
#if defined(__aarch64__) || defined(_M_ARM64) || !defined(__x86_64__) || !defined(__x86_64__)
    #define ARCH_ARM true
#else
    #define ARCH_ARM false
#endif

#include "OctoKinematics.h"
#include <iostream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <list>
#include <fstream>
#include <string>

#define PIN_DIR1 12
#define PIN_STEP1 16
#define PIN_DIR2 21
#define PIN_STEP2 20
#define PIN_DIR3 18
#define PIN_STEP3 23
#define PIN_DIR4 22
#define PIN_STEP4 27

#define PIN_MS1 17
#define PIN_MS2 4
#define PIN_MS3 3

#define PIN_JOY_X0 0
#define PIN_JOY_X1 0
#define PIN_JOY_Y0 0
#define PIN_JOY_Y1 0
#define PIN_JOY_PB 2

#define PIN_FC_M1 5
#define PIN_FC_M2 6
#define PIN_FC_M3 13

#define PIN_BOMBA 26

#define ARM_LENGTH 130.0f
#define ROD_LENGTH 310.0f
#define EFF_RADIUS 35.0f
#define BAS_RADIUS 120.0f
#define BAS_POSITION (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}
#define HOME_Z -166.0f
#define LIM_Z -305

#define TRANS_MULTIPLIER 3
#define STEPS_NUM 8

static bool EXIT = false;
static int ERROR = 0;

std::chrono::milliseconds elapsedTime = std::chrono::milliseconds(0);
std::chrono::milliseconds calcTime = std::chrono::milliseconds(0);
std::chrono::milliseconds loadTime = std::chrono::milliseconds(0);
std::chrono::milliseconds stepTime = std::chrono::milliseconds(0);
std::chrono::high_resolution_clock::time_point time0;
std::chrono::high_resolution_clock::time_point time1;

int main(int argc, char** argv)
{
    OctoKinematics octoKin = OctoKinematics(ARM_LENGTH, ROD_LENGTH, EFF_RADIUS, BAS_RADIUS);
    double x = 0, y = 0, z = HOME_Z;
    double lastX = -1, lastY = -1, lastZ = -1;

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

    // config robot pins
    octoKin.set_pin_step_ctrl(PIN_MS1,PIN_MS2,PIN_MS3);
    octoKin.set_pin_motor_1(PIN_STEP1, PIN_DIR1);
    octoKin.set_pin_motor_2(PIN_STEP2, PIN_DIR2);
    octoKin.set_pin_motor_3(PIN_STEP3, PIN_DIR3);
    octoKin.set_pin_limit_sw(PIN_FC_M1, PIN_FC_M2, PIN_FC_M3);
    octoKin.set_axis_direction(1);
    octoKin.set_step_precision(STEPS_NUM);
    octoKin.set_transmission_ratio(TRANS_MULTIPLIER);
    octoKin.set_pulse_width(1500);

    gpioWrite(PIN_BOMBA,0);

    int effector_steps = 0;
    std::string nombreArchivo = "initdata";

    std::ifstream archivoEntrada(nombreArchivo); // Abrir archivo en modo lectura
    if (archivoEntrada.is_open()) {
        archivoEntrada >> effector_steps; // Leer el valor del archivo
        archivoEntrada.close();            // Cerrar el archivo
        std::cout << "Correción de efector: " << effector_steps << '\n';
    } else {
        std::cerr << "No se pudo abrir el archivo para leer.\n";
        //return 1; // Error
    }

    while(effector_steps)
    {
        if(effector_steps > 0)
        {
            octoKin.step(PIN_STEP4, PIN_DIR4, 0);
            effector_steps--;
        }   
        if(effector_steps < 0)
        {
            octoKin.step(PIN_STEP4, PIN_DIR4, 1);
            effector_steps++;
        }
    }

    std::ofstream archivoSalida(nombreArchivo); // Crear y abrir archivo en modo escritura
    
    octoKin.home(0,0,HOME_Z);

    z = -280;

    std::cout << "a: " << octoKin.a << std::endl;
    std::cout << "b: " << octoKin.b << std::endl;
    std::cout << "c: " << octoKin.c << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;

    octoKin.inverse_kinematics(x, y, z);
    octoKin.updateKinematics();
    lastX = x;
    lastY = y;
    lastZ = z;

    float low_z = -294;

    while(true)
    {
        x = 30;
        y = 30;
        octoKin.linear_move(x, y, z, 0.4, 0);
        z = low_z;
        octoKin.linear_move(x, y, z, 0.4, 0);
        gpioWrite(PIN_BOMBA,1);
        usleep(500000);
        z = -280;
        octoKin.linear_move(x, y, z, 0.4, 0);
        for(int i = 0; i < 4076; i++)
        {
            octoKin.step(PIN_STEP4, PIN_DIR4, 0);
            effector_steps--;
            if (archivoSalida.is_open()) {
                archivoSalida.seekp(0);                // Mover el puntero de escritura al inicio del archivo
                archivoSalida << effector_steps; // Escribir el valor en el archivo
                archivoSalida.flush(); 
            } else {
                std::cerr << "No se pudo abrir el archivo para escribir.\n";
                return 1; // Error
            }
        }
        x = 30;
        y = -30;
        octoKin.linear_move(x, y, z, 0.4, 0);
        z = low_z;
        octoKin.linear_move(x, y, z, 0.4, 0);
        usleep(500000);
        gpioWrite(PIN_BOMBA,0);
        usleep(500000);
        z = -280;
        octoKin.linear_move(x, y, z, 0.4, 0);
        for(int i = 0; i < 4076; i++)
        {
            octoKin.step(PIN_STEP4, PIN_DIR4, 1);
            effector_steps++;
            if (archivoSalida.is_open()) {
                archivoSalida.seekp(0);                // Mover el puntero de escritura al inicio del archivo
                archivoSalida << effector_steps; // Escribir el valor en el archivo
                archivoSalida.flush(); 
            } else {
                std::cerr << "No se pudo abrir el archivo para escribir.\n";
                return 1; // Error
            }
        }
        x = -30;
        y = -30;
        octoKin.linear_move(x, y, z, 0.4, 0);
        z = low_z;
        octoKin.linear_move(x, y, z, 0.4, 0);
        gpioWrite(PIN_BOMBA,1);
        usleep(500000);
        z = -280;
        octoKin.linear_move(x, y, z, 0.4, 0);
        for(int i = 0; i < 4076; i++)
        {
            octoKin.step(PIN_STEP4, PIN_DIR4, 0);
            effector_steps--;
            if (archivoSalida.is_open()) {
                archivoSalida.seekp(0);                // Mover el puntero de escritura al inicio del archivo
                archivoSalida << effector_steps; // Escribir el valor en el archivo
                archivoSalida.flush(); 
            } else {
                std::cerr << "No se pudo abrir el archivo para escribir.\n";
                return 1; // Error
            }
        }
        x = -30;
        y = 30;
        octoKin.linear_move(x, y, z, 0.4, 0);
        z = low_z;
        octoKin.linear_move(x, y, z, 0.4, 0);
        usleep(500000);
        gpioWrite(PIN_BOMBA,0);
        usleep(500000);
        z = -280;
        octoKin.linear_move(x, y, z, 0.4, 0);
        for(int i = 0; i < 4076; i++)
        {
            octoKin.step(PIN_STEP4, PIN_DIR4, 1);
            effector_steps++;
            if (archivoSalida.is_open()) {
                archivoSalida.seekp(0);                // Mover el puntero de escritura al inicio del archivo
                archivoSalida << effector_steps; // Escribir el valor en el archivo
                archivoSalida.flush(); 
            } else {
                std::cerr << "No se pudo abrir el archivo para escribir.\n";
                return 1; // Error
            }
        }
        x = 0;
        y = 0;
        octoKin.linear_move(x, y, z, 0.4, 1000);
        z = -250;
        octoKin.linear_move(x, y, z, 0.4, 0);
        for(float i = 0; i < 2*M_PI; i+=0.05f)
        {
            x = sin(i)*30.0f;
            y = cos(i)*30.0f;
            octoKin.inverse_kinematics(x,y,z);
            octoKin.updateKinematics();
        }
        x = 0;
        y = 0;
        octoKin.linear_move(x, y, z, 0.4, 1000);
        z = -280;
        octoKin.linear_move(x, y, z, 0.4, 0);
    }

    unsigned int timestep = 0;

    const std::chrono::milliseconds targetPeriod = std::chrono::milliseconds(10);
    std::chrono::high_resolution_clock::time_point initTime = std::chrono::high_resolution_clock::now();

    while(!EXIT)
    {
        std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        if(gpioRead(PIN_FC_M1) || gpioRead(PIN_FC_M2) || gpioRead(PIN_FC_M3))
        {
            ERROR = 1;
//            std::cout << "[ERROR] Contacto con final de carrera" << std::endl;
        }

        // centro x0=1 x1=0 / y0=1 y1=0
        if(gpioRead(PIN_JOY_X0) == 0 && gpioRead(PIN_JOY_X1) == 0) //izq
        {
            //x -= 1.0f;
        }
        if(gpioRead(PIN_JOY_X0) == 1 && gpioRead(PIN_JOY_X1) == 1) //der
        {
            //x += 1.0f;
        }
        if(gpioRead(PIN_JOY_Y0) == 1 && gpioRead(PIN_JOY_Y1) == 1) //abajo
        {
           // y -= 1.0f;
        }
        if(gpioRead(PIN_JOY_Y0) == 0 && gpioRead(PIN_JOY_Y1) == 0) //arriba
        {
            //y += 1.0f;
        }
	
	/*
	if(gpioRead(PIN_JOY_PB)) //reset
        {
            ERROR = -1; //haciendo homing
            
            home();

            x = 0, y = 0, z = HOME_Z;

            inverse_kinematics(x, y, z);
            lastA = servo_1_angle;
            lastB = servo_2_angle;
            lastC = servo_3_angle;

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

            z += 30;

            inverse_kinematics(x, y, z);
            updateKinematics(&lastA, &lastB, &lastC);
            lastX = x;
            lastY = y;
            lastZ = z;

            ERROR = 0;
        }
	*/

        
	mode = gpioRead(PIN_JOY_PB);

        if(mode == 1)
        {
            //x = sin(timestep*0.05f)*30.0f;
            //y = cos(timestep*0.05f)*30.0f;
	    //z = HOME_Z+cos(timestep*0.05f)*10.0f;
	    z -= 1;
        }
	

        if(!ERROR)
        {
            if(lastX != x || lastY != y || lastZ != z) // calcula solo si hubo variaciones
            {
                
                // Cálculos de cinemática
                time0 = std::chrono::high_resolution_clock::now();
                octoKin.inverse_kinematics(x, y, z);
                time1 = std::chrono::high_resolution_clock::now();
                calcTime = std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0);

                octoKin.updateKinematics();

                std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();
                elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

                std::cout << "a: " << octoKin.a << std::endl;
                std::cout << "b: " << octoKin.b << std::endl;
                std::cout << "c: " << octoKin.c << std::endl;
                std::cout << "x: " << x << std::endl;
                std::cout << "y: " << y << std::endl;
                std::cout << "z: " << z << std::endl;
                
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

    archivoSalida.close();             // Cerrar el archivo

    return EXIT_SUCCESS;
}
