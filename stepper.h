#ifndef STEPPER_H
#define STEPPER_H

#include "raylib.h"
#include <unistd.h> // para usleep
#include <pigpio.h>


#define DS 26     //serial data
#define OE 19     //master reset
#define STCP 0   //storage
#define SHCP 5    //shift
#define MR 6     //
#define columnas 12
#define VACUUM 13

#define MOTORS_N 3
#define COILS_N 4

bool vector[columnas] = {};

long duty = 50;
int waitMicroSeconds = 500;
int pulseCount = 5;

int motorID = 0;

bool coils[MOTORS_N][COILS_N] = {0};

void secuencia(){
  int i,j;
  for(i = 0; i < MOTORS_N; i++)
  {
    for(j = 0; j < 4; j++)
    {
      gpioWrite(DS, coils[i][j]);
      if((i < MOTORS_N-1) && (j < COILS_N-1)){
        gpioWrite(SHCP, 0);
        gpioWrite(SHCP, 1);
        gpioWrite(SHCP, 0);
      }
    }
  }
  gpioWrite(STCP, 0);
  gpioWrite(STCP, 1);
  gpioWrite(STCP, 0);
}

void one(){
  coils[motorID][0] = 0;
  coils[motorID][1] = 1;
  coils[motorID][2] = 0;
  coils[motorID][3] = 1;
  //bool sec1[columnas] = {1,0,1,0,1,0,1,0,1,0,1,0}; // B'-B-A'-A Se encienden bobinas A y B
  //memcpy(vector,sec1,sizeof(sec1));
  secuencia();
}

void two(){
  /*digitalWrite(A, HIGH);   
  digitalWrite(B, LOW);   
  digitalWrite(C, LOW);   
  digitalWrite(D, HIGH);*/
  coils[motorID][0] = 0;
  coils[motorID][1] = 1;
  coils[motorID][2] = 1;
  coils[motorID][3] = 0;
  secuencia();
}

void three(){
  coils[motorID][0] = 1;
  coils[motorID][1] = 0;
  coils[motorID][2] = 1;
  coils[motorID][3] = 0;
  secuencia(); 
}

void four(){
  coils[motorID][0] = 1;
  coils[motorID][1] = 0;
  coils[motorID][2] = 0;
  coils[motorID][3] = 1;
  secuencia(); 
}


void oneB(){
  coils[motorID][0] = 0;
  coils[motorID][1] = 1;
  coils[motorID][2] = 1;
  coils[motorID][3] = 1;
  secuencia();
}

void twoB(){
  coils[motorID][0] = 1;
  coils[motorID][1] = 1;
  coils[motorID][2] = 1;
  coils[motorID][3] = 0;
  secuencia();  
}

void threeB(){
  coils[motorID][0] = 1;
  coils[motorID][1] = 0;
  coils[motorID][2] = 1;
  coils[motorID][3] = 1;
  secuencia();
}

void fourB(){
  coils[motorID][0] = 1;
  coils[motorID][1] = 1;
  coils[motorID][2] = 0;
  coils[motorID][3] = 1;
  secuencia();
}

// main routine to microstep
void doStep(int st){
  
  long dt1 = waitMicroSeconds * duty / 100;
  long dt2 = waitMicroSeconds * (100-duty) / 100;

  for (int j = 0; j < pulseCount; j++){
    switch (st){
    case 1: one();break;
    case 2: two();break;
    case 3: three();break;
    case 4: four();break;
    case 11: oneB();break;
    case 12: twoB();break;
    case 13: threeB();break;
    case 14: fourB();break;
    
    case 21: one();break;
    case 22: two();break;
    case 23: three();break;
    case 24: four();break;
    case 31: oneB();break;
    case 32: twoB();break;
    case 33: threeB();break;
    case 34: fourB();break;

    }

    usleep(dt1);

    switch (st){
    case 1: one();break;
    case 2: two();break;
    case 3: three();break;
    case 4: four();break;
    case 11: oneB();break;
    case 12: twoB();break;
    case 13: threeB();break;
    case 14: fourB();break;
    
    case 21: oneB();break;
    case 22: twoB();break;
    case 23: threeB();break;
    case 24: fourB();break;
    case 31: two();break;
    case 32: three();break;
    case 33: four();break;
    case 34: one();break;
    }
    usleep(dt2);
    
  }
}

// disable motor
void motorOff(){
  /* Important note:
       Turning off the motor will make it go into a 'rest' state. 
       When using microsteps (or even full steps), this may not be the last active step. 
       So using this routine may change the position of the motor a bit.
  */
  
  /*gpioWrite(A, 0);   
  gpioWrite(B, 0);   
  gpioWrite(C, 0);   
  gpioWrite(D, 0);*/
  //bool secMoff[columnas] = {1,1,1,1,1,1,1,1,1,1,1,1};
  coils[motorID][0] = 1;
  coils[motorID][1] = 1;
  coils[motorID][2] = 1;
  coils[motorID][3] = 1;
  secuencia();
}

// full stepping 4 steps :
void do4Steps(int cnt, bool forwards){
  for (int i = 0; i < cnt; i++){
    duty = 50;
    if (forwards)
      {for (int j = 1; j <= 4; j++){doStep(j);}}
    else
      {for (int j = 4; j >= 1; j--){doStep(j);}}

  }
}

// half stepping 8 steps :
void do8Steps(int cnt, bool forwards){
  const int list[] = {1,11,2,12,3,13,4,14};
  //const int list[] = {11,1,12,2,13,3,14,4};
  for (int i = 0; i < cnt; i++){
    duty = 50;
    if (forwards)
      {for (int j = 0; j <= 7; j++){doStep(list[j]);}}
    else
      {for (int j = 7; j >= 0; j--){doStep(list[j]);}}
  }
}


// microstepping 16 steps :
void do16Steps(int cnt, bool forwards){

  const int list[] = {1,21,11,31,2,22,12,32,3,23,13,33,4,24,14,34};
  //const int list[] = {1,21,11,31,2,22,12,32,3,23,13,33,4,24,14,34};
  for (int i = 0; i < cnt; i++){
    duty = 50;
    if (forwards)
      {for (int j = 0; j <= 15; j++){doStep(list[j]);}}
    else
      {for (int j = 15; j >= 0; j--){doStep(list[j]);}}
  }  
}
  
// microstepping >16 steps :
void doMoreSteps(int cnt, bool forwards){
  const int list1[] = {1,11,2,12,3,13,4,14};
  const int list2[] = {21,31,22,32,23,33,24,34};
  
  for (int i = 0; i < cnt; i++){

    duty = 50;
    if (forwards)
      {for (int j = 0; j <= 7; j++){doStep(list1[j]); doSteps(list2[j], forwards);}}
    else
      {for (int j = 7; j >= 0; j--){doSteps(list2[j], forwards); doStep(list1[j]);}}
     
  }
}

// this routine handles >16 microsteps 
// uncomment sections to choose # steps
void doSteps(int st, bool forwards){
  
// *********************** 24 steps 

  if (forwards){
    duty = 66;    doStep(st);
    duty = 33;    doStep(st);
  }
  else{
    duty = 33;    doStep(st);
    duty = 66;    doStep(st);
  }
 


// *********************** 32 steps 

  /*if (forwards){
    duty = 75;    doStep(st);
    duty = 50;    doStep(st);
    duty = 25;    doStep(st);
  }
  else{
    duty = 25;    doStep(st);
    duty = 50;    doStep(st);
    duty = 75;    doStep(st);
  }*/


// *********************** 48 steps 
/*
  if (forwards){
    for (int i = 5; i >= 1; i--){duty = 17 * i; doStep(st);}
  }
  else{
    for (int i = 1; i <= 5; i++){duty = 17 * i; doStep(st);}
  }
*/

/*
// *********************** 64 steps 

  if (forwards){
    for (int i = 7; i >= 1; i--){duty = 12 * i; doStep(st);}
  }
  else{
    for (int i = 1; i <= 7; i++){duty = 12 * i; doStep(st);}
  }
*/
     
// *********************** 96 steps 

/*  if (forwards){
    for (int i = 9; i >= 1; i--){duty = 10 * i; doStep(st);}
  }
  else{
    for (int i = 1; i <= 9; i++){duty = 10 * i; doStep(st);}
  }*/

}

#endif //STEPPER_H