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
//        printi("ERROR: Ball joint 1 out of range: rod_length_p_angle = ", radsToDeg(rod_length_p_angle));
        return false;
    }

    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2)); //Extension of the arm from the centre of the servo rotation to the end ball joint of link2

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //Checks the extension in the reachable range (This limit assumes that this->rodLength is greater than this->armLength)
//       printi("ERROR: Extension 1 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext)); //Cosine rule that calculates the angle between the ext line and this->armLength
    float omega = atan2(zt, this->motorOffsetX - arm_end_x); //Calculates the angle between horizontal (X) the ext line with respect to its quadrant
    float theta = phi + omega; //Theta is the angle between horizontal (X) and this->armLength
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){ //Checks the angle is in the reachable range
//        printi("ERROR: Servo angle 1 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    this->a = beta * (180.0/3.141592653589793238463);
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
//        printi("ERROR: Ball joint 2 out of range: rod_length_p_angle = ", radsToDeg(rod_length_p_angle));        
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2));

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //This limit assumes that this->rodLength is greater than this->armLength
//        printi("ERROR: Extension 2 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext));
    float omega = atan2(zt, this->motorOffsetX - arm_end_x);
    float theta = phi + omega;
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){
//        printi("ERROR: Servo angle 2 out of range: Angle = ", radsToDeg(theta));
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
//        printi("ERROR: Ball joint 1 out of range: rod_length_p_angle = ", radsToDeg(rod_length_p_angle));
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(this->motorOffsetX - arm_end_x, 2));

    if(ext <= rod_length_p - this->armLength || ext >= this->armLength + rod_length_p){ //This limit assumes that this->rodLength is greater than this->armLength
//        printi("ERROR: Extension 3 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(this->armLength, 2) + pow(ext, 2) - pow(rod_length_p, 2)) / (2 * this->armLength * ext));
    float omega = atan2(zt, this->motorOffsetX - arm_end_x);
    float theta = phi + omega;
    float beta = theta-M_PI;

    if(!(beta >= MOTOR_ANGLE_MIN && beta <= MOTOR_ANGLE_MAX)){
//        printi("ERROR: Servo angle 3 out of range: Angle = ", radsToDeg(theta));
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
            this->_x = -xt;
            this->_z = -zt;
        }
        else{
            this->_x = xt;
            this->_z = zt;
        }
        this->_y = yt;

        return true;
    }
    return false;
}

void OctoKinematics::set_axis_direction(bool dir)
{
    this->axis_direction = dir;
}