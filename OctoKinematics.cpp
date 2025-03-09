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
        printf("ERROR: Ball joint 1 out of range: rod_length_p_angle = %.2f\n", rod_length_p_angle*(180.0/M_PI));
        return false;
    }

    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2)); //Extension of the arm from the centre of the servo rotation to the end ball joint of link2

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //Checks the extension in the reachable range (This limit assumes that this->rodLength is greater than this->armLength)
        printf("ERROR: Extension 1 out of range: ext = %.2f\n", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext)); //Cosine rule that calculates the angle between the ext line and this->armLength
    float omega = atan2(zt, this->motorOffsetX - arm_end_x); //Calculates the angle between horizontal (X) the ext line with respect to its quadrant
    float theta = phi + omega; //Theta is the angle between horizontal (X) and this->armLength
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){ //Checks the angle is in the reachable range
        printf("ERROR: Servo angle 1 out of range: Angle = %.2f\n", theta*(180.0/M_PI));
        return false;
    }
    
    this->a = beta * 57.29577951f;
    return true;
}

bool OctoKinematics::inverse_kinematics_2(float xt, float yt, float zt)
{
    zt -= MOTOR_OFFSET_Z;
    float x = xt;
    float y = yt;
    xt = x * (-0.5f) - y * (0.8660254038f); //Rotate coordinate frame 120 degrees
    yt = x * (0.8660254038f) + y * (-0.5f);
    
    float arm_end_x = xt + this->effRadius;
    float rod_length_p = sqrt(pow(this->rodLength, 2) - pow(yt, 2));
    
    float rod_length_p_angle = asin(yt / this->rodLength);
    if(!(abs(rod_length_p_angle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (this->rodLength) going out of range.
        printf("ERROR: Ball joint 2 out of range: rod_length_p_angle = %.2f\n", rod_length_p_angle*(180.0/M_PI));        
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2));

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //This limit assumes that this->rodLength is greater than this->armLength
        printf("ERROR: Extension 2 out of range: ext = %.2f\n", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext));
    float omega = atan2(zt, this->motorOffsetX - arm_end_x);
    float theta = phi + omega;
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){
        printf("ERROR: Servo angle 2 out of range: Angle = %.2f\n", theta*(180.0/M_PI));
        return false;
    }
    
    this->b = beta * 57.29577951f;
    return true;
}

bool OctoKinematics::inverse_kinematics_3(float xt, float yt, float zt)
{
    zt -= MOTOR_OFFSET_Z;

    float x = xt;
    float y = yt;
    xt = x * (-0.5f) - y * (-0.8660254038f); //Rotate coordinate frame 240 degrees
    yt = x * (-0.8660254038f) + y * (-0.5f);

    float arm_end_x = xt + this->effRadius;
    float rod_length_p = sqrt(pow(this->rodLength, 2) - pow(yt, 2));
    
    float rod_length_p_angle = asin(yt / this->rodLength);
    if(!(abs(rod_length_p_angle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (this->rodLength) going out of range.
        printf("ERROR: Ball joint 1 out of range: rod_length_p_angle = %.2f\n", rod_length_p_angle*(180.0/M_PI));
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2));

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //This limit assumes that this->rodLength is greater than this->armLength
        printf("ERROR: Extension 3 out of range: ext = %.2f\n", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext));
    float omega = atan2(zt, this->motorOffsetX - arm_end_x);
    float theta = phi + omega;
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){
        printf("ERROR: Servo angle 3 out of range: Angle = %.2f\n", theta*(180.0/M_PI));
        return false;
    }
    
    this->c = beta * 57.29577951f;
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
    #if ARCH_ARM
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
    #endif

    this->steps_num = stepsNum;
    this->step_angle = 1.8/(this->steps_num*this->trans_ratio);
}

void OctoKinematics::set_transmission_ratio(double transRatio)
{
    this->trans_ratio = transRatio;
    this->step_angle = 1.8/(this->steps_num*this->trans_ratio);
}

void OctoKinematics::set_starting_z(double z)
{
    this->starting_z = z;
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

void OctoKinematics::set_pin_motor_effector(unsigned int step_pin, unsigned int dir_pin)
{
    this->pin_step_eff = step_pin;
    this->pin_dir_eff = dir_pin;
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

int OctoKinematics::step(int step_pin, int dir_pin, bool dir)
{
    #if ARCH_ARM
    if(!ls_hit || IGNORE_LIMIT_SWITCHES)
    {
        gpioWrite(dir_pin, dir);
        gpioWrite(step_pin, 1);
        usleep(this->pulse_width);
        gpioWrite(step_pin, 0);
    }else return EXIT_FAILURE;

    if(gpioRead(this->pin_ls1) || gpioRead(this->pin_ls2) || gpioRead(this->pin_ls3))
    {
        ls_hit = true;
    }
    #else
        usleep(this->pulse_width);
    #endif

    return EXIT_SUCCESS;
}

int OctoKinematics::updateKinematics(void)
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
            if(step(this->pin_step1, this->pin_dir1, 1)) return EXIT_FAILURE;
            reached = false;
        }else if(diffA < -this->step_angle)
        {
            this->lastA -= this->step_angle;
            if(step(this->pin_step1, this->pin_dir1, 0)) return EXIT_FAILURE;
            reached = false;
        }
        if(diffB > this->step_angle)
        {
            this->lastB += this->step_angle;
            if(step(this->pin_step2, this->pin_dir2, 1)) return EXIT_FAILURE;
            reached = false;
        }else if(diffB < -this->step_angle)
        {
            this->lastB -= this->step_angle;
            if(step(this->pin_step2, this->pin_dir2, 0)) return EXIT_FAILURE;
            reached = false;
        }
        if(diffC > this->step_angle)
        {
            this->lastC += this->step_angle;
            if(step(this->pin_step3, this->pin_dir3, 1)) return EXIT_FAILURE;
            reached = false;
        }else if(diffC < -this->step_angle)
        {
            this->lastC -= this->step_angle;
            if(step(this->pin_step3, this->pin_dir3, 0)) return EXIT_FAILURE;
            reached = false;
        }
    }

    return EXIT_SUCCESS;
}


// Interpola entre dos puntos para moverse en línea recta con aceleración y desaceleración suaves
int OctoKinematics::linear_move_eased(float x1, float y1, float z1, float stepDist, int stepDelay)
{
    // Establece las variables de posición inicial
    float x0 = this->x;
    float y0 = this->y;
    float z0 = this->z;
    
    // Distancia a recorrer en cada eje
    float xDist = x1 - x0;
    float yDist = y1 - y0;
    float zDist = z1 - z0;
    
    // Distancia total del movimiento
    double totalDist = sqrt(pow(xDist,2) + pow(yDist,2) + pow(zDist,2));
    int numberOfSteps = round(totalDist / stepDist); // Número de pasos basado en stepDist

    if(numberOfSteps == 0){
        printf("ERROR: No change in position: numberOfSteps = %i\n", numberOfSteps);
        return EXIT_FAILURE;
    }
    
    // Variables de interpolación
    float xInterpolation, yInterpolation, zInterpolation;

    for(int i = 1; i <= numberOfSteps; i++){
        // Calcula t normalizado (0 a 1) y aplica easing cúbico (aceleración/desaceleración)
        float t = static_cast<float>(i) / numberOfSteps;

        // circ
        float eased_t = t < 0.5f
                        ? (1.0f - sqrt(1.0f - pow(2.0f * t, 2.0f))) / 2.0f
                        : (sqrt(1.0f - pow(-2.0f * t + 2.0f, 2.0f)) + 1.0f) / 2.0f;
        // quint
        // float eased_t = t < 0.5f ? 16.0f * t * t * t * t * t : 1.0f - pow(-2.0f * t + 2.0f, 5.0f) / 2.0f;
        // cubic
        // float eased_t = t < 0.5f ? 4.0f * t * t * t : 1.0f - pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;
        // float eased_t = t * t * (3.0f - 2.0f * t); // Función de easing cúbico
        
        // Interpolación con easing
        xInterpolation = x0 + eased_t * xDist;
        yInterpolation = y0 + eased_t * yDist;
        zInterpolation = z0 + eased_t * zDist;

        // Calcula cinemática inversa y actualiza
        this->inverse_kinematics(xInterpolation, yInterpolation, zInterpolation);
        if(this->updateKinematics()) return EXIT_FAILURE;
        // usleep(stepDelay);
    }
    return EXIT_SUCCESS;
}

//interpolates between two points to move in a stright line (beware of physical and kinematic limits)
int OctoKinematics::linear_move(float x1, float y1, float z1, float stepDist, int stepDelay)
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
        printf("ERROR: No change in position: numberOfSteps = %i\n", numberOfSteps);
        return EXIT_FAILURE;
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
        if(this->updateKinematics()) return EXIT_FAILURE;
        // usleep(stepDelay);
    }
    return EXIT_SUCCESS;
}

int OctoKinematics::home(float x, float y, float z)
{
    bool m1_ready = false;
    bool m2_ready = false;
    bool m3_ready = false;

    int aux_step_precision = this->steps_num;
    useconds_t aux_pulse_width = this->pulse_width;

    fprintf(stdout, "Homing...");
    fflush(stdout);
    
    this->set_step_precision(8);
    this->set_pulse_width(1500);

    IGNORE_LIMIT_SWITCHES = true;

    #if ARCH_ARM
    while(!m1_ready || !m2_ready || !m3_ready)
    {
        if(!gpioRead(this->pin_ls1))
        {
            step(this->pin_step1, this->pin_dir1, 1);
        }else{
            if(!m1_ready) fprintf(stdout, " M1 ready...");
            m1_ready = true;
        }

        if(!gpioRead(this->pin_ls2))
        {
            step(this->pin_step2, this->pin_dir2, 1);
        }else{
            if(!m2_ready) fprintf(stdout, " M2 ready...");
            m2_ready = true;
        }

        if(!gpioRead(this->pin_ls3))
        {
            step(this->pin_step3, this->pin_dir3, 1);
        }else{
            if(!m3_ready) fprintf(stdout, " M3 ready...");
            m3_ready = true;
        }

        fflush(stdout);
    }
    #endif

    this->inverse_kinematics(x,y,z);
    this->a = 45;
    this->b = 45;
    this->c = 45;
    std::cout << "a: " << this->a << std::endl;
    std::cout << "b: " << this->b << std::endl;
    std::cout << "c: " << this->c << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;

    this->lastA = this->a;
    this->lastB = this->b;
    this->lastC = this->c;

    // volver a poner el paso configurado por el usuario
    this->set_step_precision(aux_step_precision);
    this->set_pulse_width(aux_pulse_width);

    if(this->starting_z != 0)
        this->linear_move(x, y, this->starting_z, 0.1f, 1000);

    this->ls_hit = false;
    IGNORE_LIMIT_SWITCHES = false;

    fprintf(stdout, "Homing complete\n");

    return EXIT_SUCCESS;
}

void OctoKinematics::set_pulse_width_effector(useconds_t us)
{
    this->pulse_width_eff = us;
}

void OctoKinematics::set_effector_precision(int stepsNum)
{
    this->steps_num_eff = stepsNum;
    this->step_angle_eff = 5.625/(64.0*this->steps_num_eff);
}

void OctoKinematics::step_eff(bool dir)
{
    #if ARCH_ARM
    gpioWrite(this->pin_dir_eff, dir);
    gpioWrite(this->pin_step_eff, 1);
    usleep(this->pulse_width_eff);
    gpioWrite(this->pin_step_eff, 0);
    #endif
}


void OctoKinematics::correct_effector_init(int steps)
{
    while(steps)
    {
        if(steps > 0)
        {
            step_eff(0);
            steps--;
        }
        if(steps < 0)
        {
            step_eff(1);
            steps++;
        }
    }
}

void OctoKinematics::rotate_effector(float angle)
{
    bool direction = (angle > 0) ? 1 : 0;
    int steps = abs(round(angle/this->step_angle_eff));

    while(steps)
    {
        step_eff(direction);
        steps--;
    }
}