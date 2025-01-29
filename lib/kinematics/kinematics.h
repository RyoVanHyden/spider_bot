#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>

const float LEG3_LENGTH = 8.1;
const float LEG2_LENGTH = 4.7;
const float LEG1_LENGTH = 3.5;

void computeInverseKinematics (float x, float y, float z, float& theta1, float& theta2, float& theta3);
void computeForwardKinematics (float theta1, float theta2, float theta3, float& x, float& y, float& z);

#endif

