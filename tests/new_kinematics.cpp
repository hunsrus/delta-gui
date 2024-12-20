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
#include <vector>//Agregado

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
#define BAS_RADIUS 117.0f
#define BAS_POSITION (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}
#define HOME_Z -170.0f
#define LIM_Z -278.0f//-239.0f

#define TRANS_MULTIPLIER 3
#define STEPS_NUM 16//number steps: 1(full), 2(half), 4(quarter), 8(eighth), 16(sixteenth)

static bool EXIT = false;
static int ERROR = 0;

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

    x = 0;
    y = 0;
    z = LIM_Z;

    octoKin.inverse_kinematics(x, y, z);
    octoKin.updateKinematics();
    lastX = x;
    lastY = y;
    lastZ = z;
 
    std::cout << "a: " << octoKin.a << std::endl;
    std::cout << "b: " << octoKin.b << std::endl;
    std::cout << "c: " << octoKin.c << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;   
 
    //while(true){};

    float low_z = LIM_Z;//-294
    float high_z = LIM_Z+10;
    std::vector<bool> bomb_on = {1, 0, 1, 0};//Agregado

    while(true)
    {
        /*
	x = 30;
        y = 30;
	z = high_z;
        octoKin.linear_move(x, y, z, 0.1, 0);
	z = low_z;
        octoKin.linear_move(x, y, z, 0.1, 0);
        //gpioWrite(PIN_BOMBA,1);
	//gpioWrite(PIN_BOMBA,bomb_on[0]);//Agregado
        //usleep(500000);
	usleep(25000);
        //z = high_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
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
	z = low_z;
        octoKin.linear_move(x, y, z, 0.1, 0);
        //z = low_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
        //usleep(500000);
        //gpioWrite(PIN_BOMBA,0);
	//gpioWrite(PIN_BOMBA,bomb_on[1]);//Agregado
        //usleep(500000);
	usleep(25000);
        //z = high_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
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
	z = low_z;
        octoKin.linear_move(x, y, z, 0.1, 0);
        //z = low_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
        //gpioWrite(PIN_BOMBA,1);
	//gpioWrite(PIN_BOMBA,bomb_on[2]);//Agregado
        //usleep(500000);
	usleep(25000);
        //z = high_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
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
	z = low_z;
        octoKin.linear_move(x, y, z, 0.1, 0);
        //z = low_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
        //usleep(500000);
        //gpioWrite(PIN_BOMBA,0);
	//gpioWrite(PIN_BOMBA,bomb_on[3]);//Agregado
        //usleep(500000);
	usleep(25000);
        //z = high_z;
        //octoKin.linear_move(x, y, z, 0.1, 0);
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
        octoKin.linear_move(x, y, z, 0.1, 1000);
	gpioWrite(PIN_BOMBA,0);//Agregado
        z = LIM_Z;//-250
        octoKin.linear_move(x, y, z, 0.1, 0);
        for(float i = 0; i < 2*M_PI; i+=0.05f)
        {
            x = sin(i)*30.0f;
            y = cos(i)*30.0f;
            octoKin.inverse_kinematics(x,y,z);
            octoKin.updateKinematics();
        }
        x = 0;
        y = 0;
        octoKin.linear_move(x, y, z, 0.1, 1000);
        z = high_z;
        octoKin.linear_move(x, y, z, 0.1, 0);
	for(int i=0; i<bomb_on.size(); i++){//Agregado
		bomb_on[i] = !bomb_on[i];
	}
	*/
	
	// dibujar grilla
	double stepDist = 0.002;
	for(int i=-30; i<=30; i=i+3){
	  x = i;
	  y = -30;
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	  x = i;
	  y = 30;
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	}
	for(int i=-30; i<=30; i=i+3){
	  x = -30;
	  y = i;
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	  x = 30;
	  y = i;
	  z = low_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	  z = high_z;
	  octoKin.linear_move(x, y, z, stepDist, 0);
	  usleep(250000);
	}

	// Ubicar componentes
	std::vector<double> vect1_x = {-30.0, -30.0, -30.0, -30.0, -30.0, -30.0};
	std::vector<double> vect1_y = {0.0, -3.0, -6.0, -9.0, -12.0, -15.0};
	std::vector<double> vect2_x = {-15.0, -24.0, -7.0, -1.0, -10.0, -14.0};
	std::vector<double> vect2_y = {-15.0, -8.0, -23.0, -30.0, -7.0, -1.0};
	double stepDist = 0.0008;
	for(int i=0; i<=5; i++){
	 x = vect1_x[i];
	 y = vect1_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 gpioWrite(PIN_BOMBA,1);
	 usleep(25000);
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 x = vect2_x[i];
	 y = vect2_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(25000);
	 gpioWrite(PIN_BOMBA,0);
	 usleep(250000);
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	}
	for(int i=0; i<=5; i++){
	 x = vect2_x[i];
	 y = vect2_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 gpioWrite(PIN_BOMBA,1);
	 usleep(25000);
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 x = vect1_x[i];
	 y = vect1_y[i];
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	 z = low_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(25000);
	 gpioWrite(PIN_BOMBA,0);
	 usleep(250000);
	 z = high_z;
	 octoKin.linear_move(x, y, z, stepDist, 0);
	 usleep(250000);
	}	
    }

    unsigned int timestep = 0;

    while(!EXIT)
    {

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
                octoKin.inverse_kinematics(x, y, z);
                octoKin.updateKinematics();


                std::cout << "a: " << octoKin.a << std::endl;
                std::cout << "b: " << octoKin.b << std::endl;
                std::cout << "c: " << octoKin.c << std::endl;
                std::cout << "x: " << x << std::endl;
                std::cout << "y: " << y << std::endl;
                std::cout << "z: " << z << std::endl;
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
