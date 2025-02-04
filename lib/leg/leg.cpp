#include "leg.h"
#include "position.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

LEG::LEG(int servo_id1, int servo_id2, int servo_id3, float Length1, float length2, float length3, float to1, float to2, float to3, char leg_type){
    
    current_pos.setX(0);
    current_pos.setY(0);
    current_pos.setZ(0);

    desired_pos.setX(0);
    desired_pos.setY(0);
    desired_pos.setZ(0);

    current_theta1 = 0;
    current_theta2 = 0;
    current_theta3 = 0;

    desired_theta1 = 0;
    desired_theta2 = 0;
    desired_theta3 = 0;

    delta_theta1 = 0;
    delta_theta2 = 0;
    delta_theta3 = 0;

    servo1_id = servo_id1;
    servo2_id = servo_id2;
    servo3_id = servo_id3;

    LEG1_LENGTH = Length1;
    LEG2_LENGTH = length2;
    LEG3_LENGTH = length3;

    theta1_offset = to1;
    theta2_offset = to2;
    theta3_offset = to3;

    type = leg_type;

    for (int i = 0; i<wtp; i++){
        W_first_trajectory_points[i] = Position(0,0,0);
        W_second_trajectory_points[i] = Position(0,0,0);
        W_third_trajectory_points[i] = Position(0,0,0);
    }

    W_trajectory1_computed = false;
    W_trajectory2_computed = false;
    W_trajectory3_computed = false;

    CR_trajectory1_computed = false;
    CR_trajectory2_computed = false;
    CR_trajectory3_computed = false;
    CR_trajectory4_computed = false;

    t_index = 0;

    isOnDesiredPosition = false;
    isOnTrajectory = false;

}

LEG::LEG(){

    current_pos.setX(0);
    current_pos.setY(0);
    current_pos.setZ(0);

    desired_pos.setX(0);
    desired_pos.setY(0);
    desired_pos.setZ(0);

    current_theta1 = 0;
    current_theta2 = 0;
    current_theta3 = 0;

    desired_theta1 = 0;
    desired_theta2 = 0;
    desired_theta3 = 0;

    delta_theta1 = 0;
    delta_theta2 = 0;
    delta_theta3 = 0;

    servo1_id = 0;
    servo2_id = 1;
    servo3_id = 2;

    LEG1_LENGTH = 0;
    LEG2_LENGTH = 0;
    LEG3_LENGTH = 0;

    theta1_offset = 0;
    theta2_offset = 0;
    theta3_offset = 0;

    type = 'C';

    for (int i = 0; i<wtp; i++){
        W_first_trajectory_points[i] = Position(0,0,0);
        W_second_trajectory_points[i] = Position(0,0,0);
        W_third_trajectory_points[i] = Position(0,0,0);
    }

    t_index = 0;

    W_trajectory1_computed = false;
    W_trajectory2_computed = false;
    W_trajectory3_computed = false;

    CR_trajectory1_computed = false;
    CR_trajectory2_computed = false;
    CR_trajectory3_computed = false;
    CR_trajectory4_computed = false;

    isOnDesiredPosition = true;
    isOnTrajectory = false;

}

void LEG::attachServoDriver(Adafruit_PWMServoDriver pwm){
    pwm_driver = pwm;
}

Position LEG::getCurrentFootPosition(){
    computeFootPosition();
    return current_pos;
}

Position LEG::getDesiredFootPosition(){
    return desired_pos;
}

void LEG::getCurrentJointAngles(float& t1, float& t2, float& t3){
    t1 = current_theta1;
    t2 = current_theta2;
    t3 = current_theta3;

    //Serial.println("Current angles are: [" + String(t1) + "º, " + String(t2) + "º, " + String(t3) + "º] \n");
    //Serial.println("Desired angles are: [" + String(desired_theta1) + "º, " + String(desired_theta2) + "º, " + String(desired_theta3) + "º]\n");

}

void LEG::getDesiredJointAngles(float& t1, float& t2, float& t3){
    t1 = desired_theta1;
    t2 = desired_theta2;
    t3 = desired_theta3;
}

void LEG::setDesiredFootPosition(Position pos){
    desired_pos = pos;
}

//If the difference between two values is less than df, return true, else return false
bool LEG::compareValues(float t1, float t2, float df){
    if (abs(t1-t2)>df){
        return false;
    } else {
        return true;
    }
}

bool LEG::checkDesiredPosition(){
    if (current_pos.isOnPosition(desired_pos)) {return true;} else {return false;}
}

void LEG::computeJointAngles(float x, float y, float z, float& t1, float& t2, float& t3){
    
    float xc = x;
    float yc = y;
    float zc = z;   

    computeInverseKinematics(xc, yc, zc, desired_theta1, desired_theta2, desired_theta3);

    t1 = desired_theta1;
    t2 = desired_theta2;
    t3 = desired_theta3;

    if (compareValues(current_theta1, desired_theta1, 0.2)){
        current_theta1 = desired_theta1;
    } 

    delta_theta1 = desired_theta1 - current_theta1;

    //Serial.println("[KINEMATICS]");
    //Serial.print("Desired theta1 is " + String(desired_theta1) + " and Current theta1 is " + String(current_theta1) + ", so delta theta1 is " + String(delta_theta1) + "\n");

    if (compareValues(current_theta2, desired_theta2, 1)){
        current_theta2 = desired_theta2; 
    } 

    delta_theta2 = desired_theta2 - current_theta2;

    //Serial.print("Desired theta2 is " + String(desired_theta2) + " and Current theta2 is " + String(current_theta2) + ", so delta theta2 is " + String(delta_theta2) + "\n");

    if (compareValues(current_theta3, desired_theta3, 1)){
        current_theta3 = desired_theta3;
    }

    delta_theta3 = desired_theta3 - current_theta3;

    //Serial.print("Desired theta3 is " + String(desired_theta3) + " and Current theta3 is " + String(current_theta3) + ", so delta theta3 is " + String(delta_theta3) + "\n");

    //Serial.println("Current pos = (" + String(current_pos.getX()) + ", " + String(current_pos.getY()) + ", " + String(current_pos.getZ()) + ") \n");
    
}

void LEG::computeFootPosition(){
    float x, y, z;
    computeForwardKinematics(current_theta1, current_theta2, current_theta3, x, y, z);
    current_pos.setX(x);
    current_pos.setY(y);
    current_pos.setZ(z);
}

//Descontinuada por enquanto
void LEG::moveFootBy(float df){
    checkDesiredPosition();

    if (isOnDesiredPosition) return;

    float step1 = df*delta_theta1;
    float step2 = df*delta_theta2;
    float step3 = df*delta_theta3;

    if (compareValues(current_theta1 + step1, desired_theta1, 1)){
        current_theta1 = desired_theta1;
    } else {
        current_theta1 = current_theta1 + step1;
    }

    if (compareValues(current_theta2 + step2, desired_theta2, 1)){
        current_theta2 = desired_theta2;
    } else {
        current_theta2 = current_theta2 + step2;
    }

    if (compareValues(current_theta3 + step3, desired_theta3, 1)){
        current_theta3 = desired_theta3;
    } else {
        current_theta3 = current_theta3 + step3;
    }

    float real_theta1 = current_theta1 + theta1_offset;
    float real_theta2 = current_theta2 + theta2_offset;
    float real_theta3 = current_theta3 + theta3_offset;

    float pulse_Length1 = map(real_theta1, 0.0, 180.0, 150.0, 580.0);
    float pulse_Length2 = map(real_theta2, 0.0, 180.0, 150.0, 580.0);
    float pulse_Length3 = map(real_theta3, 0.0, 180.0, 150.0, 580.0);

    Serial.println("---> REAL THETAS: " + String(real_theta1) + ", " + String(real_theta2) + ", " + String(real_theta3) + "\n");

    pwm_driver.setPWM(servo1_id, 0, pulse_Length1);
    pwm_driver.setPWM(servo2_id, 0, pulse_Length2);
    pwm_driver.setPWM(servo3_id, 0, pulse_Length3);

    Serial.println(" | Angles written on driver |");

    checkDesiredPosition();

}

//Moves leg to the desired position. Doesn't update the current position, but updates the current joint angles
void LEG::moveTo(Position pos){
    float x = pos.getX();
    float y = pos.getY();
    float z = pos.getZ();

    computeJointAngles(x, y, z, desired_theta1, desired_theta2, desired_theta3);

    quickMove(desired_theta1, desired_theta2, desired_theta3);

}

void LEG::moveOnTrajectory(int trajectory_index){
    
    Serial.println("[LEG " + String(type) + "] Moving on trajectory " + String(trajectory_index) + " to point " + String(t_index));

    if (checkDesiredPosition()){
        Serial.println("[Move On Traj] Current POS = (" + String(current_pos.getX()) + ", " + String(current_pos.getY()) + ", " + String(current_pos.getZ()) + "), Desired POS = (" + String(desired_pos.getX()) + ", " + String(desired_pos.getY()) + ", " + String(desired_pos.getZ()) + ")");
        Serial.println("[Move On Traj] Desired position already reached ");
        return;
    } else if (((trajectory_index == 1 || trajectory_index == 2 || trajectory_index == 3)&&(t_index == wtp)) || ((trajectory_index == 4 || trajectory_index == 5)&&(t_index == rtp))){
        Serial.println("[Move On Traj] Trajectory already finished");
        return;
    }

    isOnTrajectory = false;

    float d1, d2, d3;

    switch (trajectory_index)
    {
    case 1:
        moveTo(W_first_trajectory_points[t_index]);
        computeFootPosition();
        Serial.println("[Move On Traj] Current position is (" + String(current_pos.getX()) + ", " + String(current_pos.getY()) + ", " + String(current_pos.getZ()) + ")");
        if (current_pos.isOnPosition(W_first_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 2:
        computeJointAngles(W_second_trajectory_points[t_index].getX(), W_second_trajectory_points[t_index].getY(), W_second_trajectory_points[t_index].getZ(), d1, d2, d3);
        moveTo(W_second_trajectory_points[t_index]);
        computeFootPosition();
         if (current_pos.isOnPosition(W_second_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 3:
        moveTo(W_third_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(W_third_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 4:
        moveTo(R_first_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(R_first_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 5:
        moveTo(R_second_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(R_second_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 6:
        moveTo(CR_first_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(CR_first_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 7:
        moveTo(CR_second_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(CR_second_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 8:
        moveTo(CR_third_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(CR_third_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;
    case 9:
        moveTo(CR_fourth_trajectory_points[t_index]);
        computeFootPosition();
        if (current_pos.isOnPosition(CR_fourth_trajectory_points[t_index])){
            isOnTrajectory = true;
        } else {
            isOnTrajectory = false;
        }
        break;

    default:
        break;
    }   

    Serial.println("[Move On Traj] Is LEG " + String(type) + " on trajectory? " + String(isOnTrajectory));

    isOnDesiredPosition = checkDesiredPosition();

    t_index++;

}

void LEG::resetTrajectory(){
    t_index = 0;
    isOnTrajectory = false;
}

//Writes the desired angles to the servos, and updates the current joint angles of the leg with those values
void LEG::quickMove(float dt1, float dt2, float dt3){

    if (dt1 > 170){
        dt1=170;
    } else if (dt1 < 5){
        dt1 = 5;
    }

    if (dt2 > 170){
        dt2=170;
    } else if (dt2 < 5){
        dt2 = 5;
    }

    if (dt3 > 170){
        dt3=170;
    } else if (dt3 < 5){
        dt3 = 5;
    }

    desired_theta1 = dt1;
    desired_theta2 = dt2;
    desired_theta3 = dt3;

    float pulse_Length1 = map(desired_theta1 + theta1_offset, 0.0, 180.0, 150.0, 580.0);
    float pulse_Length2 = map(desired_theta2 + theta2_offset, 0.0, 180.0, 150.0, 580.0);
    float pulse_Length3 = map(desired_theta3 + theta3_offset, 0.0, 180.0, 150.0, 580.0);

    pwm_driver.setPWM(servo1_id, 0, pulse_Length1);
    pwm_driver.setPWM(servo2_id, 0, pulse_Length2);
    pwm_driver.setPWM(servo3_id, 0, pulse_Length3);

    current_theta1 = desired_theta1;
    current_theta2 = desired_theta2;
    current_theta3 = desired_theta3;
}

void LEG::stop(){
    quickMove(current_theta1, current_theta2, current_theta3);
}

bool LEG::checkTrajectoryComputation(int trajectory_index){
    switch (trajectory_index)
    {
    case 1:
        return W_trajectory1_computed;
        break;
    case 2:
        return W_trajectory2_computed;
        break;
    case 3:
        return W_trajectory3_computed;
        break;
    case 4:
        return R_trajectory1_computed;
        break;
    case 5:
        return R_trajectory2_computed;
        break;
    default:
        return false;
        break;
    }
}

void LEG::resetTrajectoryComputation(int trajectory_index){
    switch (trajectory_index)
    {
    case 1:
        W_trajectory1_computed = false;
        break;
    case 2:
        W_trajectory2_computed = false;
        break;
    case 3:
        W_trajectory3_computed = false;
        break;
    case 4:
        R_trajectory1_computed = false;
        break;
    case 5:
        R_trajectory2_computed = false;
        break;
    default:
        break;
    }
}

void LEG::resetAllTrajectoryComputations(){
    W_trajectory1_computed = false;
    W_trajectory2_computed = false;
    W_trajectory3_computed = false;
    R_trajectory1_computed = false;
    R_trajectory2_computed = false;
    CR_trajectory1_computed = false;
    CR_trajectory2_computed = false;
    CR_trajectory3_computed = false;
    CR_trajectory4_computed = false;
    resetTrajectory();
}

void LEG::computeTrajectory(Position d_pos, int mode, int trajectory_index){

    Position traject[20];

    float x = d_pos.getX();
    float y = d_pos.getY();
    float z = d_pos.getZ();

    desired_pos.setX(x);
    desired_pos.setY(y);
    desired_pos.setZ(z);

    float h_x, h_y, h_z;
    float t1, t2, t3;

    float half_x_distance = abs(x - current_pos.getX())/2;
    if (x > current_pos.getX()){
        h_x = current_pos.getX() + half_x_distance;
    } else {
        h_x = current_pos.getX() - half_x_distance;
    }

    if (trajectory_index == 4 && (type == 'C' || type == 'A')){
        h_y = y + 4.0;
    } else {
        h_y = y;
    }


    if (mode == 0){ //SETP
        if (trajectory_index == 4 || trajectory_index == 5){
            h_z = current_pos.getZ() + 6;
        } else if (trajectory_index == 1 || trajectory_index == 2 || trajectory_index == 3){
            h_z = current_pos.getZ() + 8;
        }
    } else if (mode == 1){ //SHIFT
        h_z = current_pos.getZ();
    } else {
        Serial.println("[LEG] Invalid mode");
        return;
    }

    Position half_pos(h_x, h_y, h_z);
    Position c_pos = getCurrentFootPosition();

    Serial.println("Bezier Points: C_Pos = " + String(c_pos.getX()) + ", " + String(c_pos.getY()) + ", " + String(c_pos.getZ()) + ", Half_Pos = " + String(half_pos.getX()) + ", " + String(half_pos.getY()) + ", " + String(half_pos.getZ()) + ", D_Pos = " + String(d_pos.getX()) + ", " + String(d_pos.getY()) + ", " + String(d_pos.getZ()) + "\n");

    switch (trajectory_index)
    {
    case 1:
        if(checkDesiredPosition()){
            for (int i = 0; i<wtp; i++){
                W_first_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, wtp);
        W_trajectory1_computed = true;
        for (int i = 0; i<wtp; i++){
          W_first_trajectory_points[i] = traject[i];
          Serial.println("Trajectory point " + String(i) + " is (" + String(W_first_trajectory_points[i].getX()) + ", " + String(W_first_trajectory_points[i].getY()) + ", " + String(W_first_trajectory_points[i].getZ()) + ")");
        }
        break;
    case 2:
        if(checkDesiredPosition()){
            for (int i = 0; i<wtp; i++){
                W_second_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, wtp);
        W_trajectory2_computed = true;
        for (int i = 0; i<wtp; i++){
          W_second_trajectory_points[i] = traject[i];
          Serial.println("Trajectory point " + String(i) + " is (" + String(W_second_trajectory_points[i].getX()) + ", " + String(W_second_trajectory_points[i].getY()) + ", " + String(W_second_trajectory_points[i].getZ()) + ")");
        }
        break;
    case 3:
        if(checkDesiredPosition()){
            for (int i = 0; i<wtp; i++){
                W_third_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, wtp);
        W_trajectory3_computed = true;
        for (int i = 0; i<wtp; i++){
          W_third_trajectory_points[i] = traject[i];  
          Serial.println("Trajectory point " + String(i) + " is (" + String(W_third_trajectory_points[i].getX()) + ", " + String(W_third_trajectory_points[i].getY()) + ", " + String(W_third_trajectory_points[i].getZ()) + ")");
        }
        break;
    case 4:
        if(checkDesiredPosition()){
            for (int i = 0; i<rtp; i++){
                R_first_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, rtp);
        R_trajectory1_computed = true;
        for (int i = 0; i<rtp; i++){
          R_first_trajectory_points[i] = traject[i];  
          Serial.print("Trajectory point " + String(i) + " is (" + String(R_first_trajectory_points[i].getX()) + ", " + String(R_first_trajectory_points[i].getY()) + ", " + String(R_first_trajectory_points[i].getZ()) + ")");
          computeInverseKinematics(R_first_trajectory_points[i].getX(), R_first_trajectory_points[i].getY(), R_first_trajectory_points[i].getZ(), t1, t2, t3);
          Serial.println(" | Joint angles are: " + String(t1) + ", " + String(t2) + ", " + String(t3));
        }
        break;
    case 5:
        if(checkDesiredPosition()){
            for (int i = 0; i<rtp; i++){
                R_second_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, rtp);
        R_trajectory2_computed = true;
        for (int i = 0; i<rtp; i++){
          R_second_trajectory_points[i] = traject[i];  
          Serial.println("Trajectory point " + String(i) + " is (" + String(R_second_trajectory_points[i].getX()) + ", " + String(R_second_trajectory_points[i].getY()) + ", " + String(R_second_trajectory_points[i].getZ()) + ")");
        }
        break;
    case 6: 	
        if(checkDesiredPosition()){
            for (int i = 0; i<rtp; i++){
                CR_first_trajectory_points[i] = current_pos;
            }
            return;
        } 
        computeBezierCurve(c_pos, d_pos, half_pos, traject, rtp);
        CR_trajectory1_computed = true;
        for (int i = 0; i<rtp; i++){
          CR_first_trajectory_points[i] = traject[i];  
          Serial.println("Trajectory point " + String(i) + " is (" + String(CR_first_trajectory_points[i].getX()) + ", " + String(CR_first_trajectory_points[i].getY()) + ", " + String(CR_first_trajectory_points[i].getZ()) + ")");
        }
        break;
    case 7:
        if(checkDesiredPosition()){
            for (int i = 0; i<rtp; i++){
                CR_second_trajectory_points[i] = current_pos;
            }
            return;
        } 
        computeBezierCurve(c_pos, d_pos, half_pos, traject, rtp);
        CR_trajectory2_computed = true;
        for (int i = 0; i<rtp; i++){
          CR_second_trajectory_points[i] = traject[i];  
          Serial.println("Trajectory point " + String(i) + " is (" + String(CR_second_trajectory_points[i].getX()) + ", " + String(CR_second_trajectory_points[i].getY()) + ", " + String(CR_second_trajectory_points[i].getZ()) + ")");
        }
        break;
    case 8:
        if(checkDesiredPosition()){
            for (int i = 0; i<rtp; i++){
                CR_third_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, rtp);
        CR_trajectory3_computed = true;
        for (int i = 0; i<rtp; i++){
          CR_third_trajectory_points[i] = traject[i];  
          Serial.println("Trajectory point " + String(i) + " is (" + String(CR_third_trajectory_points[i].getX()) + ", " + String(CR_third_trajectory_points[i].getY()) + ", " + String(CR_third_trajectory_points[i].getZ()) + ")");
        }

        break;
    case 9:
        if(checkDesiredPosition()){
            for (int i = 0; i<rtp; i++){
                CR_fourth_trajectory_points[i] = current_pos;
            }
            return;
        }
        computeBezierCurve(c_pos, d_pos, half_pos, traject, rtp);
        CR_trajectory4_computed = true;
        for (int i = 0; i<rtp; i++){
          CR_fourth_trajectory_points[i] = traject[i];  
          Serial.println("Trajectory point " + String(i) + " is (" + String(CR_fourth_trajectory_points[i].getX()) + ", " + String(CR_fourth_trajectory_points[i].getY()) + ", " + String(CR_fourth_trajectory_points[i].getZ()) + ")");
        }
        break;
    default:
        break;
    }

    resetTrajectory();

}
 
void LEG::computeBezierCurve(Position c_pos, Position d_pos, Position half_pos, Position (&trajectory)[20], int n_points){

    if (n_points>20){
        Serial.println("[Bezier] Too many points for trajectory");
        return;
    }

    float x0 = c_pos.getX();
    float y0 = c_pos.getY();
    float z0 = c_pos.getZ();

    float x1 = half_pos.getX();
    float y1 = half_pos.getY();
    float z1 = half_pos.getZ();

    float x2 = d_pos.getX();
    float y2 = d_pos.getY();
    float z2 = d_pos.getZ();

    float majorant = n_points-1;

    Serial.println("[Bezier] C_POS = " + String(x0) + ", " + String(y0) + ", " + String(z0) + " | HALF_POS = " + String(x1) + ", " + String(y1) + ", " + String(z1) + " | D_POS = " + String(x2) + ", " + String(y2) + ", " + String(z2));

    for (int i = 0; i<(n_points - 1); i++){
        
        float t = (i/(majorant));
        float x = pow((1-t),2)*x0 + 2*(1-t)*t*x1 + pow(t,2)*x2;
        float y = pow((1-t),2)*y0 + 2*(1-t)*t*y1 + pow(t,2)*y2;
        float z = pow((1-t),2)*z0 + 2*(1-t)*t*z1 + pow(t,2)*z2;

        trajectory[i] = Position(x, y, z);
    }
    trajectory[n_points-1] = d_pos;
    Serial.println("Overwriting last trajectory point with desired position: " + String(trajectory[n_points-1].getX()) + ", " + String(trajectory[n_points-1].getY()) + ", " + String(trajectory[n_points-1].getZ()) + "\n");
    
}

void LEG::initializePosition(Position init_pos){

   moveTo(init_pos);
   computeFootPosition();
   desired_pos.setX(current_pos.getX());
   desired_pos.setY(current_pos.getY());
   desired_pos.setZ(current_pos.getZ());

   Serial.println(" Initialized position @(" + String(init_pos.getX()) + ", " + String(init_pos.getY()) + ", " + String(init_pos.getZ()) + ") \n");
   Serial.println(" Current position @(" + String(current_pos.getX()) + ", " + String(current_pos.getY()) + ", " + String(current_pos.getZ()) + ") \n");
   Serial.println(" Desired position @(" + String(desired_pos.getX()) + ", " + String(desired_pos.getY()) + ", " + String(desired_pos.getZ()) + ") \n");
}

void LEG::hardReset(){
    current_pos.setX(0);
    current_pos.setY(0);
    current_pos.setZ(0);

    desired_pos.setX(0);
    desired_pos.setY(0);
    desired_pos.setZ(0);

}

void LEG::forceJointAngle(int theta, float angle){
    switch (theta)
    {
    case 1:
        current_theta1 = angle;
        break;
    case 2:
        current_theta2 = angle;
        break;
    case 3:
        current_theta3 = angle;
        break;
    default:
        break;
    }

    quickMove(current_theta1, current_theta2, current_theta3);

}

//Cinemática -------------------------------------------------------------------------

float LEG::rad2deg(float rad){
    return rad*180.0/PI;
}

float LEG::deg2rad(float deg){
    return deg*PI/180.0;
}

void LEG::computeInverseKinematics(float x, float y, float z, float& theta1, float& theta2, float& theta3){
    float yr, theta_1, L1, J3, J2;
   // Serial.println("Starting Computation of IK for leg " + String(type));
    switch (type)
    {
    case 'A':
        //Serial.println("Computing IK for leg C");
        yr = sqrt(pow(y,2) + pow(x,2));
        theta_1 = rad2deg(atan2(y,x));

        L1 = sqrt(pow(yr,2) + pow(z,2));
        J3 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - pow (L1, 2))/(2*LEG2_LENGTH*LEG3_LENGTH)));
        J2 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(L1,2) - pow (LEG3_LENGTH, 2))/(2*LEG2_LENGTH*L1)) - atan2(abs(z),yr));

        theta1 = theta_1;
        theta2 = 90.0 + J2;
        theta3 = 180.0 - J3;
        break;
    case 'B':
        //Serial.println("Computing IK for leg B");
        yr = sqrt(pow(y,2) + pow(x,2));
        theta_1 = rad2deg(atan2(x,y));

        L1 = sqrt(pow(yr,2) + pow(z,2));
        J3 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - pow (L1, 2))/(2*LEG2_LENGTH*LEG3_LENGTH)));
        J2 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(L1,2) - pow (LEG3_LENGTH, 2))/(2*LEG2_LENGTH*L1)) - atan2(abs(z),yr));
        
        theta1 = 90 + theta_1;
        theta2 = 90.0 - J2;
        theta3 = J3;
        break;
    case 'C':
        //Serial.println("Computing IK for leg C");
        yr = sqrt(pow(y,2) + pow(x,2));
        theta_1 = rad2deg(atan2(y,x));

        L1 = sqrt(pow(yr,2) + pow(z,2));
        J3 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - pow (L1, 2))/(2*LEG2_LENGTH*LEG3_LENGTH)));
        J2 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(L1,2) - pow (LEG3_LENGTH, 2))/(2*LEG2_LENGTH*L1)) - atan2(abs(z),yr));

        theta1 = theta_1;
        theta2 = 90.0 + J2;
        theta3 = 180.0 - J3;
        break;
    case 'D':
       // Serial.println("Computing IK for leg D");
        yr = sqrt(pow(y,2) + pow(x,2));
        theta_1 = rad2deg(atan2(x,y));

        L1 = sqrt(pow(yr,2) + pow(z,2));
        J3 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - pow (L1, 2))/(2*LEG2_LENGTH*LEG3_LENGTH)));
        J2 = rad2deg(acos((pow(LEG2_LENGTH,2) + pow(L1,2) - pow (LEG3_LENGTH, 2))/(2*LEG2_LENGTH*L1)) - atan2(abs(z),yr));
        
        theta1 = 90 + theta_1;
        theta2 = 90.0 - J2;
        theta3 = J3;
        break;
    default:
        break;
    }

   

}

void LEG::computeForwardKinematics(float theta1, float theta2, float theta3, float& x, float& y, float& z){
    float L1, B, yr, xc, yc, zc;

    switch (type)
    {
    case 'A':
        L1 = sqrt(pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - 2*LEG2_LENGTH*LEG3_LENGTH*cos(deg2rad(180-theta3)));
        B = theta2 - rad2deg(acos((pow(L1,2) + pow(LEG2_LENGTH,2) - pow(LEG3_LENGTH,2))/(2*L1*LEG2_LENGTH)));

        yr = L1*sin(deg2rad(B));

        xc = cos(deg2rad(theta1))*yr;
        yc = cos(deg2rad(90-theta1))*yr;
        zc = -L1*cos(deg2rad(B));
        break;
    case 'B':
        L1 = sqrt(pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - 2*LEG2_LENGTH*LEG3_LENGTH*cos(deg2rad(theta3)));
        B = 180 - theta2 - rad2deg(acos((pow(L1,2) + pow(LEG2_LENGTH,2) - pow(LEG3_LENGTH,2))/(2*L1*LEG2_LENGTH)));

        yr = L1*sin(deg2rad(B));

        xc = cos(deg2rad(180 - theta1))*yr;
        yc = sin(deg2rad(180 - theta1))*yr;
        zc = -L1*cos(deg2rad(B));
        break;
    case 'C':
        L1 = sqrt(pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - 2*LEG2_LENGTH*LEG3_LENGTH*cos(deg2rad(180-theta3)));
        B = theta2 - rad2deg(acos((pow(L1,2) + pow(LEG2_LENGTH,2) - pow(LEG3_LENGTH,2))/(2*L1*LEG2_LENGTH)));

        yr = L1*sin(deg2rad(B));

        xc = cos(deg2rad(theta1))*yr;
        yc = cos(deg2rad(90-theta1))*yr;
        zc = -L1*cos(deg2rad(B));
        break;
    case 'D':
        L1 = sqrt(pow(LEG2_LENGTH,2) + pow(LEG3_LENGTH,2) - 2*LEG2_LENGTH*LEG3_LENGTH*cos(deg2rad(theta3)));
        B = 180 - theta2 - rad2deg(acos((pow(L1,2) + pow(LEG2_LENGTH,2) - pow(LEG3_LENGTH,2))/(2*L1*LEG2_LENGTH)));

        yr = L1*sin(deg2rad(B));

        xc = cos(deg2rad(180 - theta1))*yr;
        yc = sin(deg2rad(180 - theta1))*yr;
        zc = -L1*cos(deg2rad(B));
        break;
    default:
        break;
    }

    x = xc;
    y = yc;
    z = zc;
}
