#include <iostream>
#include <unistd.h> // para usleep
#include <pigpio.h>

#define DIR1 22
#define STEP1 23
#define DIR2 5
#define STEP2 25
#define DIR3 26
#define STEP3 27
/*
#define DIR4 33
#define STEP4 32
*/

#define MS1 16
#define MS2 20
#define MS3 21

/*
#define MS1B 35
#define MS2B 36
#define MS3B 37
*/
#define BOMBA 8


int main(int argc, char** argv)
{
    int i = 0;

    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }
    fprintf(stdout, "pigpio initialisation complete\n");
    
    gpioSetMode(DIR1,PI_OUTPUT);
    gpioSetMode(STEP1,PI_OUTPUT);
    gpioSetMode(DIR2,PI_OUTPUT);
    gpioSetMode(STEP2,PI_OUTPUT);
    gpioSetMode(DIR3,PI_OUTPUT);
    gpioSetMode(STEP3,PI_OUTPUT);
    /*
    gpioSetMode(DIR4,PI_OUTPUT);
    gpioSetMode(STEP4,PI_OUTPUT);
    */
    gpioSetMode(MS1,PI_OUTPUT);
    gpioSetMode(MS2,PI_OUTPUT);
    gpioSetMode(MS3,PI_OUTPUT);
    /*
    gpioSetMode(MS1B,PI_OUTPUT);
    gpioSetMode(MS2B,PI_OUTPUT);
    gpioSetMode(MS3B,PI_OUTPUT);
    */
    gpioSetMode(BOMBA,PI_OUTPUT);

    gpioWrite(MS1,1);
    gpioWrite(MS2,1);
    gpioWrite(MS3,1);
    /*
    gpioWrite(MS1B,0);
    gpioWrite(MS2B,0);
    gpioWrite(MS3B,0);
    */
    gpioWrite(BOMBA,0);

    while(true)
    {
        gpioWrite(BOMBA, 0);
        gpioWrite(DIR1, 0);
        gpioWrite(DIR2, 0);
        gpioWrite(DIR3, 0);
        //gpioWrite(DIR4, 0);
        for(i=0; i<1000; i++){
            gpioWrite(STEP1, 1);
            gpioWrite(STEP2, 1);
            gpioWrite(STEP3, 1);
            //gpioWrite(STEP4, 1);
            usleep(1500);
            gpioWrite(STEP1, 0);
            gpioWrite(STEP2, 0);
            gpioWrite(STEP3, 0);
            //gpioWrite(STEP4, 0);
            usleep(1500);
        }
        i = 0;
        gpioWrite(BOMBA, 1);
        usleep(2000);
        //  gpioWrite(GATE, 1);
        gpioWrite(DIR1, 1);
        gpioWrite(DIR2, 1);
        gpioWrite(DIR3, 1);
        //gpioWrite(DIR4, 1);
        for(i=0; i<1000; i++){
            gpioWrite(STEP1, 1);
            gpioWrite(STEP2, 1);
            gpioWrite(STEP3, 1);
            //gpioWrite(STEP4, 1);
            usleep(1500);
            gpioWrite(STEP1, 0);
            gpioWrite(STEP2, 0);
            gpioWrite(STEP3, 0);
            //gpioWrite(STEP4, 0);
            usleep(1500); 
        }
    }

    return EXIT_SUCCESS;
}