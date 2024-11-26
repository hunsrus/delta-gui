#include "OctoKinematics.h"

OctoKinematics::OctoKinematics(double armLength,double rodLength,double effRadius,double basRadius)
{
  this->armLength = armLength;
  this->rodLength = rodLength;
  this->effRadius = effRadius;
  this->basRadius = basRadius;

  this->motorOffsetX = basRadius;
}

bool OctoKinematics::inverse_kinematics_1(float xt, float yt, float zt)
{
    zt -= MOTOR_OFFSET_Z; //Remove the differance in height from ground level to the centre of rotation of the servos
    
    float arm_end_x = xt + this->effRadius; //Adding the distance between the end effector centre and ball joints to the target x coordinate
    float rod_length_p = sqrt(pow(this->rodLength, 2) - pow(yt, 2)); //The length of link 2 when projected onto the XZ plane
    
    float rod_length_p_angle = asin(yt / this->rodLength); //Gives the angle between link2 and the ball joints. (Not actually necessary to calculate the inverse kinematics. Just used to prevent the arms ripping themselves apart.)
    if(!(abs(rod_length_p_angle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (this->rodLength) going out of range. (Angle was determined by emprical testing.)
        printf("ERROR: Ball joint 1 out of range: rod_length_p_angle = %.2f", rod_length_p_angle*(180.0/M_PI));
        return false;
    }

    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2)); //Extension of the arm from the centre of the servo rotation to the end ball joint of link2

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //Checks the extension in the reachable range (This limit assumes that this->rodLength is greater than this->armLength)
        printf("ERROR: Extension 1 out of range: ext = %.2f", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext)); //Cosine rule that calculates the angle between the ext line and this->armLength
    float omega = atan2(zt, this->motorOffsetX - arm_end_x); //Calculates the angle between horizontal (X) the ext line with respect to its quadrant
    float theta = phi + omega; //Theta is the angle between horizontal (X) and this->armLength
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){ //Checks the angle is in the reachable range
        printf("ERROR: Servo angle 1 out of range: Angle = ", theta*(180.0/M_PI));
        return false;
    }
    
    this->a = beta * (180.0/M_PI);
    return true;
}

bool OctoKinematics::inverse_kinematics_2(float xt, float yt, float zt)
{
    zt -= MOTOR_OFFSET_Z;
    float x = xt;
    float y = yt;
    xt = x * cos(2.0943951023931954923084289221863f) - y * sin(2.0943951023931954923084289221863f); //Rotate coordinate frame 120 degrees
    yt = x * sin(2.0943951023931954923084289221863f) + y * cos(2.0943951023931954923084289221863f);
    
    float arm_end_x = xt + this->effRadius;
    float rod_length_p = sqrt(pow(this->rodLength, 2) - pow(yt, 2));
    
    float rod_length_p_angle = asin(yt / this->rodLength);
    if(!(abs(rod_length_p_angle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (this->rodLength) going out of range.
        printf("ERROR: Ball joint 2 out of range: rod_length_p_angle = %.2f", rod_length_p_angle*(180.0/M_PI));        
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2));

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //This limit assumes that this->rodLength is greater than this->armLength
        printf("ERROR: Extension 2 out of range: ext = %.2f", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext));
    float omega = atan2(zt, this->motorOffsetX - arm_end_x);
    float theta = phi + omega;
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){
        printf("ERROR: Servo angle 2 out of range: Angle = %.2f", theta*(180.0/M_PI));
        return false;
    }
    
    this->b = beta * (180.0/3.141592653589793238463);
    return true;
}

bool OctoKinematics::inverse_kinematics_3(float xt, float yt, float zt)
{
    zt -= MOTOR_OFFSET_Z;

    float x = xt;
    float y = yt;
    xt = x * cos(4.1887902047863909846168578443727f) - y * sin(4.1887902047863909846168578443727f); //Rotate coordinate frame 240 degrees
    yt = x * sin(4.1887902047863909846168578443727f) + y * cos(4.1887902047863909846168578443727f);

    float arm_end_x = xt + this->effRadius;
    float rod_length_p = sqrt(pow(this->rodLength, 2) - pow(yt, 2));
    
    float rod_length_p_angle = asin(yt / this->rodLength);
    if(!(abs(rod_length_p_angle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (this->rodLength) going out of range.
        printf("ERROR: Ball joint 1 out of range: rod_length_p_angle = %.2f", rod_length_p_angle*(180.0/M_PI));
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2));

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //This limit assumes that this->rodLength is greater than this->armLength
        printf("ERROR: Extension 3 out of range: ext = %.2f", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext));
    float omega = atan2(zt, this->motorOffsetX - arm_end_x);
    float theta = phi + omega;
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){
        printf("ERROR: Servo angle 3 out of range: Angle = %.2f", theta*(180.0/M_PI));
        return false;
    }
    
    this->c = beta * (180.0/3.141592653589793238463);
    return true;
}

bool OctoKinematics::inverse_kinematics(float xt, float yt, float zt)
{    
    if(axis_direction == 1){//if axis are inverted
        xt = -xt;
        zt = -zt;
    }
    
    if(inverse_kinematics_1(xt, yt, zt) && inverse_kinematics_2(xt, yt, zt) && inverse_kinematics_3(xt, yt, zt)){ //Calculates and checks the positions are valid.
        if(axis_direction == 1){//if axis are inverted
            this->x = -xt;
            this->z = -zt;
        }
        else{
            this->x = xt;
            this->z = zt;
        }
        this->y = yt;

        return true;
    }
    return false;
}

void OctoKinematics::set_axis_direction(bool dir)
{
    this->axis_direction = dir;
}

void OctoKinematics::set_step_precision(int stepsNum)
{
    if(stepsNum == 1)
    {
        gpioWrite(this->pin_ms1,0);
        gpioWrite(this->pin_ms2,0);
        gpioWrite(this->pin_ms3,0);
    }else if(stepsNum == 2){
        gpioWrite(this->pin_ms1,1);
        gpioWrite(this->pin_ms2,0);
        gpioWrite(this->pin_ms3,0);
    }else if(stepsNum == 4){
        gpioWrite(this->pin_ms1,0);
        gpioWrite(this->pin_ms2,1);
        gpioWrite(this->pin_ms3,0);
    }else if(stepsNum == 8){
        gpioWrite(this->pin_ms1,1);
        gpioWrite(this->pin_ms2,1);
        gpioWrite(this->pin_ms3,0);
    }else if(stepsNum == 16){
        gpioWrite(this->pin_ms1,1);
        gpioWrite(this->pin_ms2,1);
        gpioWrite(this->pin_ms3,1);
    }

    this->steps_num = stepsNum;
    this->step_angle = 1.8/(this->steps_num*this->trans_ratio);
}

void OctoKinematics::set_transmission_ratio(double transRatio)
{
    this->trans_ratio = transRatio;
    this->step_angle = 1.8/(this->steps_num*this->trans_ratio);
}

void OctoKinematics::set_pin_step_ctrl(unsigned int ms1, unsigned int ms2, unsigned int ms3)
{
    this->pin_ms1 = ms1;
    this->pin_ms2 = ms2;
    this->pin_ms3 = ms3;
}

void OctoKinematics::set_pin_motor_1(unsigned int step_pin, unsigned int dir_pin)
{
    this->pin_step1 = step_pin;
    this->pin_dir1 = dir_pin;
}

void OctoKinematics::set_pin_motor_2(unsigned int step_pin, unsigned int dir_pin)
{
    this->pin_step2 = step_pin;
    this->pin_dir2 = dir_pin;
}

void OctoKinematics::set_pin_motor_3(unsigned int step_pin, unsigned int dir_pin)
{
    this->pin_step3 = step_pin;
    this->pin_dir3 = dir_pin;
}

void OctoKinematics::set_pin_limit_sw(unsigned int ls1, unsigned int ls2, unsigned int ls3)
{
    this->pin_ls1 = ls1;
    this->pin_ls2 = ls2;
    this->pin_ls3 = ls3;
}

void OctoKinematics::set_pulse_width(useconds_t us)
{
    this->pulse_width = us;
}

void OctoKinematics::step(int step_pin, int dir_pin, bool dir)
{
    gpioWrite(dir_pin, dir);
    gpioWrite(step_pin, 1);
    usleep(this->pulse_width);
    gpioWrite(step_pin, 0);
}

void OctoKinematics::updateKinematics(void)
{
    double diffA, diffB, diffC;

    bool reached = false;

    while(!reached)
    {
        diffA = this->a - this->lastA;
        diffB = this->b - this->lastB;
        diffC = this->c - this->lastC;

        reached = true;
        if(diffA > this->step_angle)
        {
            this->lastA += this->step_angle;
            step(this->pin_step1, this->pin_dir1, 1);
            reached = false;
        }else if(diffA < -this->step_angle)
        {
            this->lastA -= this->step_angle;
            step(this->pin_step1, this->pin_dir1, 0);
            reached = false;
        }
        if(diffB > this->step_angle)
        {
            this->lastB += this->step_angle;
            step(this->pin_step2, this->pin_dir2, 1);
            reached = false;
        }else if(diffB < -this->step_angle)
        {
            this->lastB -= this->step_angle;
            step(this->pin_step2, this->pin_dir2, 0);
            reached = false;
        }
        if(diffC > this->step_angle)
        {
            this->lastC += this->step_angle;
            step(this->pin_step3, this->pin_dir3, 1);
            reached = false;
        }else if(diffC < -this->step_angle)
        {
            this->lastC -= this->step_angle;
            step(this->pin_step3, this->pin_dir3, 0);
            reached = false;
        }
    }
}

//interpolates between two points to move in a stright line (beware of physical and kinematic limits)
void OctoKinematics::linear_move(float x1, float y1, float z1, float stepDist, int stepDelay)
{
    //Sets the initial position variables
    float x0 = this->x;
    float y0 = this->y;
    float z0 = this->z;
    
    //Distance change in each axis
    float xDist = x1 - x0;
    float yDist = y1 - y0;
    float zDist = z1 - z0;
    
    double totalDist = sqrt(pow(xDist,2) + pow(yDist,2) + pow(zDist,2));//Absolute magnitute of the distance
    int numberOfSteps = round(totalDist / stepDist);//Number of steps required for the desired step distance

    //Step size of each axis
    if(numberOfSteps == 0){
        printf("ERROR: No change in position: numberOfSteps = %i", numberOfSteps);
        return;
    }
    
    float xStep = xDist / (float)numberOfSteps;
    float yStep = yDist / (float)numberOfSteps;
    float zStep = zDist / (float)numberOfSteps;

    //Interpolation variables
    float xInterpolation;
    float yInterpolation;
    float zInterpolation;

    for(int i = 1; i <= numberOfSteps; i++){//Interpolate the points
        xInterpolation = x0 + i * xStep;
        yInterpolation = y0 + i * yStep;
        zInterpolation = z0 + i * zStep;

        this->inverse_kinematics(xInterpolation, yInterpolation, zInterpolation);//calculates the inverse kinematics for the interpolated values
        this->updateKinematics();
        usleep(stepDelay);
    }
}

int OctoKinematics::home(float x, float y, float z)
{
    bool m1_ready = false;
    bool m2_ready = false;
    bool m3_ready = false;

    fprintf(stdout, "Homing...");
    // paso completo
    gpioWrite(this->pin_ms1,1);
    gpioWrite(this->pin_ms2,0);
    gpioWrite(this->pin_ms3,0);

    while(!m1_ready || !m2_ready || !m3_ready)
    {
        if(!gpioRead(this->pin_ls1))
        {
            step(this->pin_step1, this->pin_dir1, 1);
        }else{
            m1_ready = true;
            fprintf(stdout, " M1 ready...");
        }

        if(!gpioRead(this->pin_ls2))
        {
            step(this->pin_step2, this->pin_dir2, 1);
        }else{
            m2_ready = true;
            fprintf(stdout, " M2 ready...\n");
        }

        if(!gpioRead(this->pin_ls3))
        {
            step(this->pin_step3, this->pin_dir3, 1);
        }else{
            m3_ready = true;
            fprintf(stdout, " M3 ready...\n");
        }
    }

    this->inverse_kinematics(x, y, z);
    this->lastA = this->a;
    this->lastB = this->b;
    this->lastC = this->c;

    fprintf(stdout, "Homing complete\n");

    return EXIT_SUCCESS;
}