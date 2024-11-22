// detección de la arquitectura
#if defined(__aarch64__) || defined(_M_ARM64) || !defined(__x86_64__) || !defined(__x86_64__)
    #define ARCH_ARM true
#else
    #define ARCH_ARM false
#endif

#include <iostream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <list>

#include <pigpio.h>

// Vector3, 3 components
typedef struct Vector3 {
    float x;                // Vector x component
    float y;                // Vector y component
    float z;                // Vector z component
} Vector3;

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
#define ROD_LENGTH 310.0f
#define BASS_TRI 35.0f
#define PLATFORM_TRI 120.0f
#define PLATFORM_POS (Vector3){0,ARM_LENGTH+ROD_LENGTH,0}
#define HOME_Z 166.0f

#define L1 ARM_LENGTH
#define L2 ROD_LENGTH
#define L3 BASS_TRI
#define SERVO_OFFSET_X PLATFORM_TRI
#define SERVO_OFFSET_Z 0
#define SERVO_ANGLE_MIN -M_PI/4
#define SERVO_ANGLE_MAX M_PI*1.7
double servo_1_angle;
double servo_2_angle;
double servo_3_angle;
bool axis_direction = 0;
Vector3 end_effector;

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

bool inverse_kinematics_1(float xt, float yt, float zt){
    zt -= SERVO_OFFSET_Z; //Remove the differance in height from ground level to the centre of rotation of the servos
    
    float arm_end_x = xt + L3; //Adding the distance between the end effector centre and ball joints to the target x coordinate
    float l2p = sqrt(pow(L2, 2) - pow(yt, 2)); //The length of link 2 when projected onto the XZ plane
    
    float l2pAngle = asin(yt / L2); //Gives the angle between link2 and the ball joints. (Not actually necessary to calculate the inverse kinematics. Just used to prevent the arms ripping themselves apart.)
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range. (Angle was determined by emprical testing.)
//        printi("ERROR: Ball joint 1 out of range: l2pAngle = ", radsToDeg(l2pAngle));
        return false;
    }

    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2)); //Extension of the arm from the centre of the servo rotation to the end ball joint of link2

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //Checks the extension in the reachable range (This limit assumes that L2 is greater than L1)
//       printi("ERROR: Extension 1 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext)); //Cosine rule that calculates the angle between the ext line and L1
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x); //Calculates the angle between horizontal (X) the ext line with respect to its quadrant
    float theta = phi + omega; //Theta is the angle between horizontal (X) and L1

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){ //Checks the angle is in the reachable range
//        printi("ERROR: Servo angle 1 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_1_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics_2(float xt, float yt, float zt){
    zt -= SERVO_OFFSET_Z;
    float x = xt;
    float y = yt;
    xt = x * cos(2.0943951023931954923084289221863f) - y * sin(2.0943951023931954923084289221863f); //Rotate coordinate frame 120 degrees
    yt = x * sin(2.0943951023931954923084289221863f) + y * cos(2.0943951023931954923084289221863f);
    
    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(L2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / L2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//        printi("ERROR: Ball joint 2 out of range: l2pAngle = ", radsToDeg(l2pAngle));        
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
//        printi("ERROR: Extension 2 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//        printi("ERROR: Servo angle 2 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_2_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics_3(float xt, float yt, float zt){
    zt -= SERVO_OFFSET_Z;

    float x = xt;
    float y = yt;
    xt = x * cos(4.1887902047863909846168578443727f) - y * sin(4.1887902047863909846168578443727f); //Rotate coordinate frame 240 degrees
    yt = x * sin(4.1887902047863909846168578443727f) + y * cos(4.1887902047863909846168578443727f);

    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(L2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / L2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//        printi("ERROR: Ball joint 1 out of range: l2pAngle = ", radsToDeg(l2pAngle));
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
//        printi("ERROR: Extension 3 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//        printi("ERROR: Servo angle 3 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_3_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics(float xt, float yt, float zt){    
    if(axis_direction == 1){//if axis are inverted
        xt = -xt;
        zt = -zt;
    }
    
    if(inverse_kinematics_1(xt, yt, zt) && inverse_kinematics_2(xt, yt, zt) && inverse_kinematics_3(xt, yt, zt)){ //Calculates and checks the positions are valid.
        if(axis_direction == 1){//if axis are inverted
            end_effector.x = -xt;
            end_effector.z = -zt;
        }
        else{
            end_effector.x = xt;
            end_effector.z = zt;
        }
        end_effector.y = yt;

        return true;
    }
    return false;
}

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

void updateKinematics(double *lastA, double *lastB, double *lastC)
{
    double diffA, diffB, diffC;

    time0 = std::chrono::high_resolution_clock::now();
    bool reached = false;

    while(!reached)
    {
        diffA = servo_1_angle - *lastA;
        diffB = servo_2_angle - *lastB;
        diffC = servo_3_angle - *lastC;

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
    double x = 0, y = 0, z = -HOME_Z;
    double lastX = -1, lastY = -1, lastZ = -1;
    double lastA, lastB, lastC;

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

    x = 0, y = 0, z = -HOME_Z;

    inverse_kinematics(x, y, z);
    lastA = servo_1_angle;
    lastB = servo_2_angle;
    lastC = servo_3_angle;
    lastX = x;
    lastY = y;
    lastZ = z;

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

    z -= 30;

    inverse_kinematics(x, y, z);
    updateKinematics(&lastA, &lastB, &lastC);
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

            x = 0, y = 0, z = -HOME_Z;

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

            z -= 30;

            inverse_kinematics(x, y, z);
            updateKinematics(&lastA, &lastB, &lastC);
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
                inverse_kinematics(x, y, z);
                time1 = std::chrono::high_resolution_clock::now();
                calcTime = std::chrono::duration_cast<std::chrono::milliseconds>(time1 - time0);

                updateKinematics(&lastA, &lastB, &lastC);

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