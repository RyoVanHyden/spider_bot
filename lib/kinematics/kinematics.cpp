#include "kinematics.h"

void computeInverseKinematics(float x, float y, float z, float& theta1, float& theta2, float& theta3){
    float yr = sqrt(pow(y,2) + pow(x,2));
    float theta_1 = atan2(y,x);

    float L1 = sqrt(pow(yr,2) + pow(z,2));
    float J3 = acos((pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - pow (L1, 2))/(2*LEG2_LENGTH*LEG3_LENGTH));
    float J2 = acos ((pow(LEG2_LENGTH,2) + pow(L1,2) - pow (LEG3_LENGTH, 2))/(2*LEG2_LENGTH*L1)) - atan2(abs(z),yr);

    theta1 = theta_1;
    theta2 = 90.0 + J2;
    theta3 = 180.0 - J3;

}

void computeForwardKinematics(float theta1, float theta2, float theta3, float& x, float& y, float& z){
    float L1 = sqrt(pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - 2*LEG2_LENGTH*LEG3_LENGTH*cos(180.0-theta3));
    float B = theta2 -acos((pow(L1,2) + pow(LEG2_LENGTH,2) - pow(LEG3_LENGTH,2))/(2*L1*LEG2_LENGTH));

    float yr = L1*sin(B);

    float xc = cos(theta1)*yr;
    float yc = cos(90-theta1)*yr;
    float zc = -L1*cos(B);

    x = xc;
    y = yc;
    z = zc;
}