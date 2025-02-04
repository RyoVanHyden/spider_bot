#include "spider_robot.h"
#include "leg.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "position.h"
#include <cmath>

Spider_Robot::Spider_Robot(){
   
    setNewState(walk_fsm, sm_idle);
    setNewState(lateral_walk_fsm, sm_idle);
    setNewState(rotate_fsm, sm_idle);
    setNewState(incline_fsm, sm_idle);
    setNewState(continuos_rotation_fsm, sm_idle);

    // Different Hight positions ----------------------------------------

    // These positions are CARTESIAN oriented, and refer to the Y and Z coordinates of the foot. The X should be obtained trough Trigonometry
    // for the OUTTER position, the angle is 28.27º, for the INNER position its -9.93º


    higher_pos[0][0] = 3.75;
    higher_pos[0][1] = -5.7;
    higher_pos[1][0] = 4.10;
    higher_pos[1][1] = -6.54;
    higher_pos[2][0] = 4.33;
    higher_pos[2][1] = -7.44;
    higher_pos[3][0] = 4.38;
    higher_pos[3][1] = -8.36;
    higher_pos[4][0] = 4.27;
    higher_pos[4][1] = -9.28;
    higher_pos[5][0] = 3.98;
    higher_pos[5][1] = -10.16;
    higher_pos[6][0] = 3.55;
    higher_pos[6][1] = -10.96;
    higher_pos[7][0] = 2.97;
    higher_pos[7][1] = -11.66;
    higher_pos[8][0] = 2.28;
    higher_pos[8][1] = -12.21;
    higher_pos[9][0] = 1.50;
    higher_pos[9][1] = -12.62;

    hight_index = 0;

    // Robot's own Location ---------------------------------------------

    Current_location = Position(0,0,0);
    Desired_location = Position(0,0,0);
    robot_step_size = 7.5;

    // Walking positions ------------------------------------------------

    walk_z = -5.7;
    walk_y = 3.75;
    walk_x_IN = -0.65;
    walk_x_OUT = 1.85;
    BIG_f_step_size = 8.65;
    SMALL_f_step_size = 6.15;

    W_posA[0] = Position(walk_x_IN, walk_y, walk_z);   
    W_posA[1] = Position(walk_x_IN + BIG_f_step_size, walk_y, walk_z);       
    W_posA[2] = Position(walk_x_OUT, walk_y, walk_z);      

    W_posB[0] = Position(walk_x_OUT + SMALL_f_step_size, walk_y, walk_z + 0.5);        
    W_posB[1] = Position(walk_x_IN, walk_y, walk_z + 0.5);      
    W_posB[2] = Position(walk_x_OUT, walk_y, walk_z + 0.5);

    W_posC[0] = Position(walk_x_IN, walk_y, walk_z - 0.2);         
    W_posC[1] = Position(walk_x_OUT + SMALL_f_step_size, walk_y, walk_z - 0.2);        
    W_posC[2] = Position(walk_x_IN, walk_y, walk_z - 0.2);       

    W_posD[0] = Position(walk_x_IN + BIG_f_step_size - 0.5, walk_y, walk_z);           
    W_posD[1] = Position(walk_x_OUT, walk_y - 0.5, walk_z);        
    W_posD[2] = Position(walk_x_IN, walk_y - 0.5, walk_z);        


    // Lateral Walking positions ----------------------------------------

    lat_step_size = 3.25;

    LW_posA[0] = Position(-0.3650 , walk_y - lat_step_size/2, walk_z);
    LW_posA[1] = Position(-0.0865, walk_y - lat_step_size, walk_z);
    LW_posA[2] = Position(-0.650, walk_y, walk_z);

    LW_posB[0] = Position(-1.210, walk_y + lat_step_size, walk_z + 0.5);
    LW_posB[1] = Position(-0.9315, walk_y + lat_step_size/2, walk_z + 0.5);
    LW_posB[2] = Position(-0.65, walk_y, walk_z + 0.5);

    LW_posC[0] = Position(2.6517, walk_y + lat_step_size/2, walk_z - 0.2);
    LW_posC[1] = Position(1.0483, walk_y - lat_step_size/2, walk_z - 0.2);
    LW_posC[2] = Position(1.85, walk_y, walk_z - 0.2);

    LW_posD[0] = Position(2.6517, walk_y + lat_step_size/2, walk_z);
    LW_posD[1] = Position(1.0483, walk_y - lat_step_size/2, walk_z);
    LW_posD[2] = Position(1.85, walk_y, walk_z);

    // Rotation positions ----------------------------------------------

    //Pi/4 = 0.7854
    rot_angle = 35.0 *(PI/180.0);
    float x_rot = walk_y * sin(rot_angle);
    float y_rot = walk_y * cos(rot_angle);

    R_posA[0] = Position(0, walk_y, walk_z);
    R_posA[1] = Position(-x_rot, y_rot, walk_z);
    R_posA[2] = Position(0, walk_y, walk_z);
    
    R_posB[0] = Position(0, walk_y, walk_z + 0.5);
    R_posB[1] = Position(x_rot, y_rot, walk_z + 0.5);
    R_posB[2] = Position(0, walk_y, walk_z + 0.5);
    
    R_posC[0] = Position(0, walk_y, walk_z - 0.2);
    R_posC[1] = Position(-x_rot, y_rot, walk_z - 0.2);
    R_posC[2] = Position(0, walk_y, walk_z - 0.2);
    
    R_posD[0] = Position(0, walk_y, walk_z);
    R_posD[1] = Position(x_rot, y_rot, walk_z);
    R_posD[2] = Position(0, walk_y, walk_z);

    // Continuous Rotation Positions  ---------------------------------

    rot_angle = 60.0 *(PI/180.0);   

    CR_posA[0] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
    CR_posA[1] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
    CR_posA[2] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z);
    CR_posA[3] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);

    CR_posB[0] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);
    CR_posB[1] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);
    CR_posB[2] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);
    CR_posB[3] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);

    CR_posC[0] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
    CR_posC[1] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
    CR_posC[2] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
    CR_posC[3] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);

    CR_posD[0] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
    CR_posD[1] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
    CR_posD[2] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z);
    CR_posD[3] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);

    rotation_direction = false;
    current_angle = 0.0;

    // -----------------------------------------------------------------

    pos_index = 0;

    //
    delta_z = 0;

}
// LEG(int servo_id1, int servo_id2, int servo_id3, float Length1, float length2, float length3, float to1, float to2, float to3);

void Spider_Robot::attachLegs(LEG& legA, LEG& legB, LEG& legC, LEG& legD){
    this->legA = legA;
    this->legB = legB;
    this->legC = legC;
    this->legD = legD;
}

void Spider_Robot::initializePositions(Position init_posA, Position init_posB, Position init_posC, Position init_posD){
    legA.initializePosition(init_posA);
    legB.initializePosition(init_posB);
    legC.initializePosition(init_posC);
    legD.initializePosition(init_posD);
}

void Spider_Robot::reInitializePositions(){
    legA.initializePosition(Position(walk_x_OUT, walk_y, walk_z));
    legB.initializePosition(Position(walk_x_OUT, walk_y, walk_z + 0.5));
    legC.initializePosition(Position(walk_x_IN, walk_y, walk_z - 0.2));
    legD.initializePosition(Position(walk_x_IN, walk_y, walk_z));
}

void Spider_Robot::setNewState(fsm& sm, int new_state){
    if (sm.state!=new_state){
        sm.state = new_state;
        sm.tes = millis();
        sm.tis = 0;
    }
}

bool Spider_Robot::legsOnDesiredPositions(){
    Serial.println("[LEG A] Current POS = (" + String(legA.getCurrentFootPosition().getX()) + ", " + String(legA.getCurrentFootPosition().getY()) + ", " + String(legA.getCurrentFootPosition().getZ()) + ") --- Desired POS = ("+
                     String(legA.getDesiredFootPosition().getX()) + ", " + String(legA.getDesiredFootPosition().getY()) + ", " + String(legA.getDesiredFootPosition().getZ()) + ")");
    Serial.println("[LEG B] Current POS = (" + String(legB.getCurrentFootPosition().getX()) + ", " + String(legB.getCurrentFootPosition().getY()) + ", " + String(legB.getCurrentFootPosition().getZ()) + ") --- Desired POS = ("+
                     String(legB.getDesiredFootPosition().getX()) + ", " + String(legB.getDesiredFootPosition().getY()) + ", " + String(legB.getDesiredFootPosition().getZ()) + ")");
    Serial.println("[LEG C] Current POS = (" + String(legC.getCurrentFootPosition().getX()) + ", " + String(legC.getCurrentFootPosition().getY()) + ", " + String(legC.getCurrentFootPosition().getZ()) + ") --- Desired POS = ("+
                     String(legC.getDesiredFootPosition().getX()) + ", " + String(legC.getDesiredFootPosition().getY()) + ", " + String(legC.getDesiredFootPosition().getZ()) + ")");
    Serial.println("[LEG D] Current POS = (" + String(legD.getCurrentFootPosition().getX()) + ", " + String(legD.getCurrentFootPosition().getY()) + ", " + String(legD.getCurrentFootPosition().getZ()) + ") --- Desired POS = ("+
                        String(legD.getDesiredFootPosition().getX()) + ", " + String(legD.getDesiredFootPosition().getY()) + ", " + String(legD.getDesiredFootPosition().getZ()) + ")");
    bool on_pos = legA.checkDesiredPosition() && legB.checkDesiredPosition() && legC.checkDesiredPosition() && legD.checkDesiredPosition();
    Serial.println("Legs On Desired Position = " + String(on_pos));
    return (on_pos);

}

Position Spider_Robot::getCurrentLocation(){
    return Current_location;
}

void Spider_Robot::setCurrentLocation(Position pos){
    Current_location = pos;
}

Position Spider_Robot::getDesiredLocation(){
    return Desired_location;
}

void Spider_Robot::setDesiredLocation(Position pos){
    Desired_location = pos;
}

float Spider_Robot::getCurrentAngle(){
    return current_angle;
}

bool Spider_Robot::DesiredLocationReached(){
    return (Current_location.isOnPosition(Desired_location, robot_step_size, float(0.2), float(0.2)));
}

void Spider_Robot::stop(){

    START_WALKING = false;

    setNewState(walk_fsm, sm_idle);

    legA.stop();
    legB.stop();
    legC.stop();
    legD.stop();

}

void Spider_Robot::walkTo(bool enableA, bool enableB, bool enableC, bool enableD, bool next, Position Desired_Location){

    if (START_WALKING){
        setNewState(walk_fsm, sm_idle);
    }

    if (walk_fsm.state == sm_idle && START_WALKING && next){
        walk_fsm.new_state = sm_compute;
        pos_index = 0;
        START_WALKING = false;
        legA.resetAllTrajectoryComputations();
        legB.resetAllTrajectoryComputations();
        legC.resetAllTrajectoryComputations();
        legD.resetAllTrajectoryComputations();
        setDesiredLocation(Desired_Location);
    } else if (walk_fsm.state == sm_compute && next && !DesiredLocationReached()){
        walk_fsm.new_state = sm_moving;
    } else if (walk_fsm.state == sm_compute && next && DesiredLocationReached()){
        walk_fsm.new_state = sm_idle;
    } else if (walk_fsm.state == sm_moving && next && legsOnDesiredPositions()){
        walk_fsm.new_state = sm_compute;
        if (pos_index<5){
            pos_index++;
        } else {
            pos_index = 0;
            stepForward();
        }
    }

    setNewState(walk_fsm, walk_fsm.new_state);

    switch (walk_fsm.state)
    {
    case sm_idle:
        //Parado
        break;
    case sm_compute:
        if (true){
            switch (pos_index)
            {
            case 0:
                Serial.println("Computing trajectory for pos 0 ----------");
                    
                if (enableD && !legD.checkTrajectoryComputation(1)){
                    Serial.println("[LEG D]");
                    legD.computeTrajectory(W_posD[0], 0, 1);
                } else {
                    Serial.println("[LEG D] Trajectory 1 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(W_posD[0]);
                    legD.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 1:
                Serial.println("Computing trajectory for pos 1 ----------");   
                if (enableA && !legA.checkTrajectoryComputation(1)){
                    Serial.println("[LEG A]");
                    legA.computeTrajectory(W_posA[0], 1, 1);
                } else {
                    Serial.println("[LEG A] Trajectory 1 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(W_posA[0]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(1)){
                    Serial.println("[LEG B] Desired Position = " + String(W_posB[0].getX()) + ", " + String(W_posB[0].getY()) + ", " + String(W_posB[0].getZ()));
                    legB.computeTrajectory(W_posB[0], 1, 1);
                } else {
                    Serial.println("[LEG B] Trajectory 2 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(W_posB[0]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(1)){
                    Serial.println("[LEG C]");
                    legC.computeTrajectory(W_posC[0], 1, 1);
                } else {
                    Serial.println("[LEG C] Trajectory 1 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(W_posC[0]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(2)){
                    Serial.println("[LEG D]");
                    legD.computeTrajectory(W_posD[1], 1, 2);
                } else {
                    Serial.println("[LEG D] Trajectory 1 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(W_posD[1]);
                    legD.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 2:
                Serial.println("Computing trajectory for pos 2 ----------");

                if (enableB && !legB.checkTrajectoryComputation(2)){
                    Serial.println("[LEG B] Desired Position = " + String(W_posB[1].getX()) + ", " + String(W_posB[1].getY()) + ", " + String(W_posB[1].getZ()));
                    legB.computeTrajectory(W_posB[1], 0, 2);
                } else {
                    Serial.println("[LEG B] Trajectory 2 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(W_posB[1]);
                    legB.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 3:
                Serial.println("Computing trajectory for pos 3 ----------");

                if (enableA && !legA.checkTrajectoryComputation(2)){
                    Serial.println("[LEG A]");
                    legA.computeTrajectory(W_posA[1], 0, 2);
                } else {
                    Serial.println("[LEG A] Trajectory 1 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(W_posA[1]);
                    legA.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 4:
                if (enableA && !legA.checkTrajectoryComputation(3)){
                    Serial.println("[LEG A]");
                    legA.computeTrajectory(W_posA[2], 1, 3);
                } else {
                    Serial.println("[LEG A] Trajectory 2 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(W_posA[2]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(3)){
                    Serial.println("[LEG B] Desired Position = " + String(W_posB[1].getX()) + ", " + String(W_posB[1].getY()) + ", " + String(W_posB[1].getZ()));
                    legB.computeTrajectory(W_posB[2], 1, 3);
                } else {
                    Serial.println("[LEG B] Trajectory 3 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(W_posB[2]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(2)){
                    Serial.println("[LEG C]");
                    legC.computeTrajectory(W_posC[1], 1, 2);
                } else {
                    Serial.println("[LEG C] Trajectory 3 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(W_posC[1]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(3)){
                    Serial.println("[LEG D]");
                    legD.computeTrajectory(W_posD[2], 1, 3);
                } else {
                    Serial.println("[LEG D] Trajectory 3 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(W_posD[2]);
                    legD.resetTrajectory();
                }
                break;
            case 5: 
                if (enableC && !legC.checkTrajectoryComputation(3)){
                    Serial.println("[LEG C]");
                    legC.computeTrajectory(W_posC[2], 0, 3);
                } else {
                    Serial.println("[LEG C] Trajectory 3 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(W_posC[2]);
                    legC.resetTrajectory();
                }
                break;
            default:
                break;
            }
        }
        break;
    case sm_moving:
        Serial.println("Moving legs on Position " + String(pos_index) + " --------------------------------");
        switch(pos_index){
            case 0:
                if (enableD){legD.moveOnTrajectory(1);}
                break;
            case 1:
                if (enableA){legA.moveOnTrajectory(1);}
                if (enableB){legB.moveOnTrajectory(1);}
                if (enableC){legC.moveOnTrajectory(1);}
                if (enableD){legD.moveOnTrajectory(2);}
                break;
            case 2:
                if (enableB) {legB.moveOnTrajectory(2);}
                break;
            case 3:
                if (enableA ){legA.moveOnTrajectory(2);}
                break;
            case 4:
                if (enableA){legA.moveOnTrajectory(3);}
                if (enableB){legB.moveOnTrajectory(3);}
                if (enableC){legC.moveOnTrajectory(2);}
                if (enableD){legD.moveOnTrajectory(3);}
                break;
            case 5:
                if (enableC){legC.moveOnTrajectory(3);}
                break;
            default:
                break;
        }

        Serial.println("Legs moved ---------------------------------------");

        break;
    default:
        break;
    }

}

void Spider_Robot::stepForward(){
    float x = Current_location.getX();
    Current_location.setX(x + robot_step_size);
}

int Spider_Robot::getPosIndex(){
    return pos_index;
}

void Spider_Robot::lateral_walk(bool enableA, bool enableB, bool enableC, bool enableD, bool next){

    if (lateral_walk_fsm.state == sm_idle && START_LATERAL_WALKING && next){
        lateral_walk_fsm.new_state = sm_compute;
        pos_index = 0;
        START_LATERAL_WALKING = false;
        legA.resetAllTrajectoryComputations();
        legB.resetAllTrajectoryComputations();
        legC.resetAllTrajectoryComputations();
        legD.resetAllTrajectoryComputations();
    } else if (lateral_walk_fsm.state == sm_compute && next){
        lateral_walk_fsm.new_state = sm_moving;
    } else if (lateral_walk_fsm.state == sm_moving && legsOnDesiredPositions() && next){
        lateral_walk_fsm.new_state = sm_compute;
        if (pos_index<5){
            pos_index++;
        } else {
            pos_index = 0;
        }
    }

    setNewState(lateral_walk_fsm, lateral_walk_fsm.new_state);

    switch (lateral_walk_fsm.state)
    {
    case sm_idle:
        //Parado
        break;
    case sm_compute:
        if (next){
            switch (pos_index)
            {
            case 0:
                Serial.println("Computing trajectory for pos 0 ----------");
                
                if (enableB && !legB.checkTrajectoryComputation(1)){
                    Serial.println("[LEG B] Desired Position = " + String(LW_posB[0].getX()) + ", " + String(LW_posB[0].getY()) + ", " + String(LW_posB[0].getZ()));
                    legB.computeTrajectory(LW_posB[0], 0, 1);
                } else {
                    Serial.println("[LEG B] Trajectory 1 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(LW_posB[0]);
                    legB.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 1:
                Serial.println("Computing trajectory for pos 1 ----------");   
                if (enableA && !legA.checkTrajectoryComputation(1)){
                    Serial.println("[LEG A]");
                    legA.computeTrajectory(LW_posA[0], 1, 1);
                } else {
                    Serial.println("[LEG A] Trajectory 1 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(LW_posA[0]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(2)){
                    Serial.println("[LEG B] Desired Position = " + String(LW_posB[1].getX()) + ", " + String(LW_posB[1].getY()) + ", " + String(LW_posB[1].getZ()));
                    legB.computeTrajectory(LW_posB[1], 1, 2);
                } else {
                    Serial.println("[LEG B] Trajectory 2 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(LW_posB[1]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(1)){
                    Serial.println("[LEG C]");
                    legC.computeTrajectory(LW_posC[0], 1, 1);
                } else {
                    Serial.println("[LEG C] Trajectory 1 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(LW_posC[0]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(1)){
                    Serial.println("[LEG D]");
                    legD.computeTrajectory(LW_posD[0], 1, 1);
                } else {
                    Serial.println("[LEG D] Trajectory 1 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(LW_posD[0]);
                    legD.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 2:
                Serial.println("Computing trajectory for pos 2 ----------");

                if (enableD && !legD.checkTrajectoryComputation(2)){
                    Serial.println("[LEG D]");
                    legD.computeTrajectory(LW_posD[1], 0, 2);
                } else {
                    Serial.println("[LEG D] Trajectory 2 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(LW_posD[1]);
                    legD.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 3:
                Serial.println("Computing trajectory for pos 3 ----------");

                if (enableC && !legC.checkTrajectoryComputation(2)){
                    Serial.println("[LEG C]");
                    legC.computeTrajectory(LW_posC[1], 0, 2);
                } else {
                    Serial.println("[LEG C] Trajectory 2 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(LW_posC[1]);
                    legC.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 4:
                if (enableA && !legA.checkTrajectoryComputation(2)){
                    Serial.println("[LEG A]");
                    legA.computeTrajectory(LW_posA[1], 1, 2);
                } else {
                    Serial.println("[LEG A] Trajectory 2 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(LW_posA[1]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(3)){
                    Serial.println("[LEG B] Desired Position = " + String(LW_posB[1].getX()) + ", " + String(LW_posB[1].getY()) + ", " + String(LW_posB[1].getZ()));
                    legB.computeTrajectory(LW_posB[2], 1, 3);
                } else {
                    Serial.println("[LEG B] Trajectory 3 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(LW_posB[2]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(3)){
                    Serial.println("[LEG C]");
                    legC.computeTrajectory(LW_posC[2], 1, 3);
                } else {
                    Serial.println("[LEG C] Trajectory 3 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(LW_posC[2]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(3)){
                    Serial.println("[LEG D]");
                    legD.computeTrajectory(LW_posD[2], 1, 3);
                } else {
                    Serial.println("[LEG D] Trajectory 3 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(LW_posD[2]);
                    legD.resetTrajectory();
                }
                break;
            case 5: 
                if (enableA && !legA.checkTrajectoryComputation(3)){
                    Serial.println("[LEG A]");
                    legA.computeTrajectory(LW_posA[2], 0, 3);
                } else {
                    Serial.println("[LEG A] Trajectory 3 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(LW_posA[2]);
                    legA.resetTrajectory();
                }
                break;
            default:
                break;
            }
        }
        break;
    case sm_moving:
        Serial.println("Moving legs on Position " + String(pos_index) + " --------------------------------");
        switch(pos_index){
            case 0:
                if (enableB && next){legB.moveOnTrajectory(1);}
                break;
            case 1:
                if (enableA && next){legA.moveOnTrajectory(1);}
                if (enableB && next){legB.moveOnTrajectory(2);}
                if (enableC && next){legC.moveOnTrajectory(1);}
                if (enableD && next){legD.moveOnTrajectory(1);}
                break;
            case 2:
                if (enableD && next){legD.moveOnTrajectory(2);}
                break;
            case 3:
                if (enableC && next){legC.moveOnTrajectory(2);}
                break;
            case 4:
                if (enableA && next){legA.moveOnTrajectory(2);}
                if (enableB && next){legB.moveOnTrajectory(3);}
                if (enableC && next){legC.moveOnTrajectory(3);}
                if (enableD && next){legD.moveOnTrajectory(3);}
                break;
            case 5:
                if (enableA && next){legA.moveOnTrajectory(3);}
                break;
            default:
                break;
        }

        Serial.println("Legs moved ---------------------------------------");

        break;
    default:
        break;
    }

}

void Spider_Robot::rotate(bool enableA, bool enableB, bool enableC, bool enableD, bool next){
    if (rotate_fsm.state == sm_idle && START_ROTATING && next){
        rotate_fsm.new_state = sm_compute;
        pos_index = 0;
        START_ROTATING = false;
        legA.resetAllTrajectoryComputations();
        legB.resetAllTrajectoryComputations();
        legC.resetAllTrajectoryComputations();
        legD.resetAllTrajectoryComputations();
    } else if (rotate_fsm.state == sm_compute && next){
        rotate_fsm.new_state = sm_moving;
    } else if (rotate_fsm.state == sm_moving && legsOnDesiredPositions() && next){
        rotate_fsm.new_state = sm_compute;
        if (pos_index<5){
            pos_index++;
        } else {
            pos_index = 1;
        }
    }

    setNewState(rotate_fsm, rotate_fsm.new_state);

    switch (rotate_fsm.state)
    {
    case sm_idle:
        //Parado
        break;
    case sm_compute:
        if (next){
            switch (pos_index)
            {
            case 0:
                if (enableA && !legA.checkTrajectoryComputation(5)){
                    Serial.println("[LEG A] Desired Position = " + String(R_posA[0].getX()) + ", " + String(R_posA[0].getY()) + ", " + String(R_posA[0].getZ()));
                    legA.computeTrajectory(R_posA[0], 0, 5);
                } else {
                    Serial.println("[LEG A] Trajectory 1 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(R_posA[0]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(5)){
                    Serial.println("[LEG B] Desired Position = " + String(R_posB[0].getX()) + ", " + String(R_posB[0].getY()) + ", " + String(R_posB[0].getZ()));
                    legB.computeTrajectory(R_posB[0], 0, 5);
                } else {
                    Serial.println("[LEG B] Trajectory 2 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(R_posB[0]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(5)){
                    Serial.println("[LEG C] Desired Position = " + String(R_posC[0].getX()) + ", " + String(R_posC[0].getY()) + ", " + String(R_posC[0].getZ()));
                    legC.computeTrajectory(R_posC[0], 0, 5);
                } else {
                    Serial.println("[LEG C] Trajectory 1 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(R_posC[0]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(5)){
                    Serial.println("[LEG D] Desired Position = " + String(R_posD[0].getX()) + ", " + String(R_posD[0].getY()) + ", " + String(R_posD[0].getZ()));
                    legD.computeTrajectory(R_posD[0], 0, 5);
                } else {
                    Serial.println("[LEG D] Trajectory 1 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(R_posD[0]);
                    legD.resetTrajectory();
                }

                // Reset all other trajectories because trajectory 5 was just used for case 0 to compute the initial position
                legA.resetAllTrajectoryComputations();
                legB.resetAllTrajectoryComputations();
                legC.resetAllTrajectoryComputations();
                legD.resetAllTrajectoryComputations();

                break;
            case 1:
                Serial.println("Computing trajectory for pos 0 ----------");
                
                if (enableC && !legC.checkTrajectoryComputation(4)){
                    Serial.println("[LEG C] Desired Position = " + String(R_posC[1].getX()) + ", " + String(R_posC[1].getY()) + ", " + String(R_posC[1].getZ()) + ", Current Position = (" + String(legC.getCurrentFootPosition().getX()) + ", " + String(legC.getCurrentFootPosition().getY()) + ", " + String(legC.getCurrentFootPosition().getZ()) + ")");
                    legC.computeTrajectory(R_posC[1], 0, 4);
                } else {
                    Serial.println("[LEG C] Trajectory 1 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(R_posC[1]);
                    legC.resetTrajectory();
                }

                Serial.println("Trajectories computed -------------------");
                break;
            case 2:
                if (enableD && !legD.checkTrajectoryComputation(4)){
                    Serial.println("[LEG D] Desired Position = " + String(R_posD[1].getX()) + ", " + String(R_posD[1].getY()) + ", " + String(R_posD[1].getZ()));
                    legD.computeTrajectory(R_posD[1], 0, 4);
                } else {
                    Serial.println("[LEG D] Trajectory 1 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(R_posD[1]);
                    legD.resetTrajectory();
                }
                break;
            case 3:
                if (enableA && !legA.checkTrajectoryComputation(4)){
                    Serial.println("[LEG A] Desired Position = " + String(R_posA[1].getX()) + ", " + String(R_posA[1].getY()) + ", " + String(R_posA[1].getZ()));
                    legA.computeTrajectory(R_posA[1], 0, 4);
                } else {
                    Serial.println("[LEG A] Trajectory 1 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(R_posA[1]);
                    legA.resetTrajectory();
                }
                break;
            case 4:
                if (enableB && !legB.checkTrajectoryComputation(4)){
                    Serial.println("[LEG B] Desired Position = " + String(R_posB[1].getX()) + ", " + String(R_posB[1].getY()) + ", " + String(R_posB[1].getZ()));
                    legB.computeTrajectory(R_posB[1], 0, 4);
                } else {
                    Serial.println("[LEG B] Trajectory 1 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(R_posB[1]);
                    legB.resetTrajectory();
                }
                break;
            case 5:
                if (enableA && !legA.checkTrajectoryComputation(5)){
                    Serial.println("[LEG A] Desired Position = " + String(R_posA[2].getX()) + ", " + String(R_posA[2].getY()) + ", " + String(R_posA[2].getZ()));
                    legA.computeTrajectory(R_posA[2], 1, 5);
                } else {
                    Serial.println("[LEG A] Trajectory 1 already computed (or LEG disabled)");
                    legA.setDesiredFootPosition(R_posA[2]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(5)){
                    Serial.println("[LEG B] Desired Position = " + String(R_posB[2].getX()) + ", " + String(R_posB[2].getY()) + ", " + String(R_posB[2].getZ()));
                    legB.computeTrajectory(R_posB[2], 1, 5);
                } else {
                    Serial.println("[LEG B] Trajectory 2 already computed (or LEG disabled)");
                    legB.setDesiredFootPosition(R_posB[2]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(5)){
                    Serial.println("[LEG C] Desired Position = " + String(R_posC[2].getX()) + ", " + String(R_posC[2].getY()) + ", " + String(R_posC[2].getZ()));
                    legC.computeTrajectory(R_posC[2], 1, 5);
                } else {
                    Serial.println("[LEG C] Trajectory 1 already computed (or LEG disabled)");
                    legC.setDesiredFootPosition(R_posC[2]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(5)){
                    Serial.println("[LEG D] Desired Position = " + String(R_posD[2].getX()) + ", " + String(R_posD[2].getY()) + ", " + String(R_posD[2].getZ()));
                    legD.computeTrajectory(R_posD[2], 1, 5);
                } else {
                    Serial.println("[LEG D] Trajectory 1 already computed (or LEG disabled)");
                    legD.setDesiredFootPosition(R_posD[2]);
                    legD.resetTrajectory();
                }

                break;
            default:
                break;
            }
        break;
    case sm_moving:
        Serial.println("Moving legs on Position " + String(pos_index) + " --------------------------------");
        switch(pos_index){
            case 0:
                if (enableA && next){legA.moveOnTrajectory(5);}
                if (enableB && next){legB.moveOnTrajectory(5);}
                if (enableC && next){legC.moveOnTrajectory(5);}
                if (enableD && next){legD.moveOnTrajectory(5);}
                break;
            case 1:
                if (enableC && next){legC.moveOnTrajectory(4);}
                break;
            case 2:
                if (enableD && next){legD.moveOnTrajectory(4);}
                break;
            case 3:
                if (enableA && next){legA.moveOnTrajectory(4);}
                break;
            case 4:
                if (enableB && next){legB.moveOnTrajectory(4);}
                break;
            case 5:
                if (enableA && next){legA.moveOnTrajectory(5);}
                if (enableB && next){legB.moveOnTrajectory(5);}
                if (enableC && next){legC.moveOnTrajectory(5);}
                if (enableD && next){legD.moveOnTrajectory(5);}
                break;
            default:
                break;
        }
        break;
    default:
        break;
    }
    }
}

void Spider_Robot::continuosRotation(bool enableA, bool enableB, bool enableC, bool enableD, bool next, bool direction){

    if (direction != rotation_direction){
        toggleRotationDirection();
        setNewState(continuos_rotation_fsm, sm_idle);
        START_ROTATING = true;
    }

    int step = boolToInt(rotation_direction);

    if (START_ROTATING){
        setNewState(continuos_rotation_fsm, sm_idle);
    }

    if (continuos_rotation_fsm.state == sm_idle && START_ROTATING && next){
        continuos_rotation_fsm.new_state = sm_compute;
        pos_index = 0;
        START_ROTATING = false;
        legA.resetAllTrajectoryComputations();
        legB.resetAllTrajectoryComputations();
        legC.resetAllTrajectoryComputations();
        legD.resetAllTrajectoryComputations();
    } else if (continuos_rotation_fsm.state == sm_compute && next){
        continuos_rotation_fsm.new_state = sm_moving;
    } else if (continuos_rotation_fsm.state == sm_moving && legsOnDesiredPositions() && next){
        continuos_rotation_fsm.new_state = sm_compute;
        if(step){
            current_angle+=20.0;
        } else {
            current_angle-=20.0;
        }

        if (pos_index<4){
            pos_index++;
        } else {
            pos_index = 1;
        }
    }

    setNewState(continuos_rotation_fsm, continuos_rotation_fsm.new_state);

    Serial.println("Rotation Direction: " + String(rotation_direction) + ", step = " + String(step));

    switch (continuos_rotation_fsm.state)
    {
    case sm_idle:
        //Parado
        break;
    case sm_compute:
        if (next){
            Serial.println("Computing trajectory for pos " + String(pos_index) + " ----------");

            switch (pos_index)
            {
            case 0:
                if (enableA){
                    Serial.println("[LEG A] Desired Position = " + String(CR_posA[0].getX()) + ", " + String(CR_posA[0].getY()) + ", " + String(CR_posA[0].getZ()));
                    legA.computeTrajectory(CR_posA[0], 0, 9);
                } else {
                    Serial.println("[LEG A] LEG disabled");
                    legA.resetTrajectory();
                }

                if (enableB){
                    Serial.println("[LEG B] Desired Position = " + String(CR_posB[0].getX()) + ", " + String(CR_posB[0].getY()) + ", " + String(CR_posB[0].getZ()));
                    legB.computeTrajectory(CR_posB[0], 0, 9);
                } else {
                    Serial.println("[LEG B] LEG disabled");
                    legB.resetTrajectory();
                }
                
                if (enableC){
                    Serial.println("[LEG C] Desired Position = " + String(CR_posC[0].getX()) + ", " + String(CR_posC[0].getY()) + ", " + String(CR_posC[0].getZ()));
                    legC.computeTrajectory(CR_posC[0], 0, 9);
                } else {
                    Serial.println("[LEG C] LEG disabled");
                    legC.resetTrajectory();
                }

                if (enableD){
                    Serial.println("[LEG D] Desired Position = " + String(CR_posD[0].getX()) + ", " + String(CR_posD[0].getY()) + ", " + String(CR_posD[0].getZ()));
                    legD.computeTrajectory(CR_posD[0], 0, 9);
                } else {
                    Serial.println("[LEG D] LEG disabled");
                    legD.resetTrajectory();
                }
                
                // Reset all other trajectories because trajectory 9 was just used for case 0 to compute the initial position
                legA.resetAllTrajectoryComputations();
                legB.resetAllTrajectoryComputations();
                legC.resetAllTrajectoryComputations();
                legD.resetAllTrajectoryComputations();

                break;

            case 1: 
                if (enableA && !legA.checkTrajectoryComputation(6)){
                    Serial.println("[LEG A] Desired Position = " + String(CR_posA[1].getX()) + ", " + String(CR_posA[1].getY()) + ", " + String(CR_posA[1].getZ()));
                    legA.computeTrajectory(CR_posA[1], step, 6);
                    if(step){
                        Serial.println("[LEG A] Step = 1, so LEG A performed a shift");
                    } else {
                        Serial.println("[LEG A] Step = 0, so LEG A performed a step");
                    }
                } else {
                    Serial.println("[LEG A] LEG disabled");
                    legA.setDesiredFootPosition(CR_posA[1]);
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(6)){
                    Serial.println("[LEG B] Desired Position = " + String(CR_posB[1].getX()) + ", " + String(CR_posB[1].getY()) + ", " + String(CR_posB[1].getZ()));
                    legB.computeTrajectory(CR_posB[1], 1, 6);
                } else {
                    Serial.println("[LEG B] LEG disabled");
                    legB.setDesiredFootPosition(CR_posB[1]);
                    legB.resetTrajectory();
                }

                if (enableC && !legC.checkTrajectoryComputation(6)){
                    Serial.println("[LEG C] Desired Position = " + String(CR_posC[1].getX()) + ", " + String(CR_posC[1].getY()) + ", " + String(CR_posC[1].getZ()));
                    legC.computeTrajectory(CR_posC[1], 1, 6);
                } else {
                    Serial.println("[LEG C] LEG disabled");
                    legC.setDesiredFootPosition(CR_posC[1]);
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(6)){
                    Serial.println("[LEG D] Desired Position = " + String(CR_posD[1].getX()) + ", " + String(CR_posD[1].getY()) + ", " + String(CR_posD[1].getZ()));
                    legD.computeTrajectory(CR_posD[1], 1-step, 6);
                    if(step){
                        Serial.println("[LEG D] Step = 1, so LEG D performed a step");
                    } else {
                        Serial.println("[LEG D] Step = 0, so LEG D performed a shift");
                    }
                } else {
                    Serial.println("[LEG D] LEG disabled");
                    legD.setDesiredFootPosition(CR_posD[1]);
                    legD.resetTrajectory();
                }

                break;

            case 2:
                if(enableA && !legA.checkTrajectoryComputation(7)){
                    Serial.println("[LEG A] Desired Position = " + String(CR_posA[2].getX()) + ", " + String(CR_posA[2].getY()) + ", " + String(CR_posA[2].getZ()));
                    legA.computeTrajectory(CR_posA[2], 1, 7);
                } else {
                    Serial.println("[LEG A] LEG disabled");
                    legA.setDesiredFootPosition(CR_posA[2]);
                    legA.resetTrajectory();
                }

                if(enableB && !legB.checkTrajectoryComputation(7)){
                    Serial.println("[LEG B] Desired Position = " + String(CR_posB[2].getX()) + ", " + String(CR_posB[2].getY()) + ", " + String(CR_posB[2].getZ()));
                    legB.computeTrajectory(CR_posB[2], step, 7);
                } else {
                    Serial.println("[LEG B] LEG disabled");
                    legB.setDesiredFootPosition(CR_posB[2]);
                    legB.resetTrajectory();
                }

                if(enableC && !legC.checkTrajectoryComputation(7)){
                    Serial.println("[LEG C] Desired Position = " + String(CR_posC[2].getX()) + ", " + String(CR_posC[2].getY()) + ", " + String(CR_posC[2].getZ()));
                    legC.computeTrajectory(CR_posC[2], 1-step, 7);
                } else {
                    Serial.println("[LEG C] LEG disabled");
                    legC.setDesiredFootPosition(CR_posC[2]);
                    legC.resetTrajectory();
                }

                if(enableD && !legD.checkTrajectoryComputation(7)){
                    Serial.println("[LEG D] Desired Position = " + String(CR_posD[2].getX()) + ", " + String(CR_posD[2].getY()) + ", " + String(CR_posD[2].getZ()));
                    legD.computeTrajectory(CR_posD[2], 1, 7);
                } else {
                    Serial.println("[LEG D] LEG disabled");
                    legD.setDesiredFootPosition(CR_posD[2]);
                    legD.resetTrajectory();
                }
                break;
            case 3:
                if(enableA && !legA.checkTrajectoryComputation(8)){
                    Serial.println("[LEG A] Desired Position = " + String(CR_posA[3].getX()) + ", " + String(CR_posA[3].getY()) + ", " + String(CR_posA[3].getZ()));
                    legA.computeTrajectory(CR_posA[3], 1, 8);
                } else {
                    Serial.println("[LEG A] LEG disabled");
                    legA.setDesiredFootPosition(CR_posA[3]);
                    legA.resetTrajectory();
                }

                if(enableB && !legB.checkTrajectoryComputation(8)){
                    Serial.println("[LEG B] Desired Position = " + String(CR_posB[3].getX()) + ", " + String(CR_posB[3].getY()) + ", " + String(CR_posB[3].getZ()));
                    legB.computeTrajectory(CR_posB[3], 1-step, 8);
                } else {
                    Serial.println("[LEG B] LEG disabled");
                    legB.setDesiredFootPosition(CR_posB[3]);
                    legB.resetTrajectory();
                }

                if(enableC && !legC.checkTrajectoryComputation(8)){
                    Serial.println("[LEG C] Desired Position = " + String(CR_posC[3].getX()) + ", " + String(CR_posC[3].getY()) + ", " + String(CR_posC[3].getZ()));
                    legC.computeTrajectory(CR_posC[3], step, 8);
                } else {
                    Serial.println("[LEG C] LEG disabled");
                    legC.setDesiredFootPosition(CR_posC[3]);
                    legC.resetTrajectory();
                }

                if(enableD && !legD.checkTrajectoryComputation(8)){
                    Serial.println("[LEG D] Desired Position = " + String(CR_posD[3].getX()) + ", " + String(CR_posD[3].getY()) + ", " + String(CR_posD[3].getZ()));
                    legD.computeTrajectory(CR_posD[3], 1, 8);
                } else {
                    Serial.println("[LEG D] LEG disabled");
                    legD.setDesiredFootPosition(CR_posD[3]);
                    legD.resetTrajectory();
                }

                break;

            case 4:
                if (enableA && !legA.checkTrajectoryComputation(9)){
                    Serial.println("[LEG A] Desired Position = " + String(CR_posA[0].getX()) + ", " + String(CR_posA[0].getY()) + ", " + String(CR_posA[0].getZ()));
                    legA.computeTrajectory(CR_posA[0], 1-step, 9);
                } else {
                    Serial.println("[LEG A] LEG disabled");
                    legA.resetTrajectory();
                }

                if (enableB && !legB.checkTrajectoryComputation(9)){
                    Serial.println("[LEG B] Desired Position = " + String(CR_posB[0].getX()) + ", " + String(CR_posB[0].getY()) + ", " + String(CR_posB[0].getZ()));
                    legB.computeTrajectory(CR_posB[0], 1, 9);
                } else {
                    Serial.println("[LEG B] LEG disabled");
                    legB.resetTrajectory();
                }
              
                if (enableC && !legC.checkTrajectoryComputation(9)){
                    Serial.println("[LEG C] Desired Position = " + String(CR_posC[0].getX()) + ", " + String(CR_posC[0].getY()) + ", " + String(CR_posC[0].getZ()));
                    legC.computeTrajectory(CR_posC[0], 1, 9);
                } else {
                    Serial.println("[LEG C] LEG disabled");
                    legC.resetTrajectory();
                }

                if (enableD && !legD.checkTrajectoryComputation(9)){
                    Serial.println("[LEG D] Desired Position = " + String(CR_posD[0].getX()) + ", " + String(CR_posD[0].getY()) + ", " + String(CR_posD[0].getZ()));
                    legD.computeTrajectory(CR_posD[0], step, 9);
                } else {
                    Serial.println("[LEG D] LEG disabled");
                    legD.resetTrajectory();
                }
                break;
             default:
                break;

            }
        }
        break;   
    case sm_moving:
        Serial.println("Moving legs on Position " + String(pos_index) + " --------------------------------");
        if (pos_index == 0){
            if (enableA && next){legA.moveOnTrajectory(9);}
            if (enableB && next){legB.moveOnTrajectory(9);}
            if (enableC && next){legC.moveOnTrajectory(9);}
            if (enableD && next){legD.moveOnTrajectory(9);}
        } else {
            if (enableA && next){legA.moveOnTrajectory(pos_index + 5);}
            if (enableB && next){legB.moveOnTrajectory(pos_index + 5);}
            if (enableC && next){legC.moveOnTrajectory(pos_index + 5);}
            if (enableD && next){legD.moveOnTrajectory(pos_index + 5);}
        }
        Serial.println("Legs moved ---------------------------------------");
        break;
    default:
        break;
    }
}

void Spider_Robot::toggleRotationDirection(){

    rot_angle = 60.0 *(PI/180.0); 

    rotation_direction = !rotation_direction;

    Serial.println("CHANGING ROTATION DIRECTION TO: " + String(rotation_direction) + " ================================================================= ");

    if (!rotation_direction){   // Anti-Clockwise
        CR_posA[0] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posA[1] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posA[2] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posA[3] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);

        CR_posB[0] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z + 0.35);
        CR_posB[1] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.35);
        CR_posB[2] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.35);
        CR_posB[3] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.35);

        CR_posC[0] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
        CR_posC[1] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
        CR_posC[2] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
        CR_posC[3] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);

        CR_posD[0] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posD[1] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posD[2] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posD[3] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
    } else {    // Clockwise
        CR_posA[0] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posA[1] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posA[2] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posA[3] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);

        CR_posB[0] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);
        CR_posB[1] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);
        CR_posB[2] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);
        CR_posB[3] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z + 0.5);

        CR_posC[0] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
        CR_posC[1] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
        CR_posC[2] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);
        CR_posC[3] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z - 0.2);

        CR_posD[0] = Position(tan((2.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posD[1] = Position(tan(-(1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posD[2] = Position(tan((0.0)*rot_angle)*walk_y, walk_y, walk_z);
        CR_posD[3] = Position(tan((1.0/3.0)*rot_angle)*walk_y, walk_y, walk_z);

    }
}

void Spider_Robot::printCRPositions(){
    Serial.print("CR_posA[0] = (" + String(CR_posA[0].getX()) + ", " + String(CR_posA[0].getY()) + ", " + String(CR_posA[0].getZ()) + ")");
    Serial.print("; CR_posA[1] = (" + String(CR_posA[1].getX()) + ", " + String(CR_posA[1].getY()) + ", " + String(CR_posA[1].getZ()) + ")");
    Serial.print("; CR_posA[2] = (" + String(CR_posA[2].getX()) + ", " + String(CR_posA[2].getY()) + ", " + String(CR_posA[2].getZ()) + ")");
    Serial.println("; CR_posA[3] = (" + String(CR_posA[3].getX()) + ", " + String(CR_posA[3].getY()) + ", " + String(CR_posA[3].getZ()) + ")");

    Serial.print("CR_posB[0] = (" + String(CR_posB[0].getX()) + ", " + String(CR_posB[0].getY()) + ", " + String(CR_posB[0].getZ()) + ")");
    Serial.print("; CR_posB[1] = (" + String(CR_posB[1].getX()) + ", " + String(CR_posB[1].getY()) + ", " + String(CR_posB[1].getZ()) + ")");
    Serial.print("; CR_posB[2] = (" + String(CR_posB[2].getX()) + ", " + String(CR_posB[2].getY()) + ", " + String(CR_posB[2].getZ()) + ")");
    Serial.println("; CR_posB[3] = (" + String(CR_posB[3].getX()) + ", " + String(CR_posB[3].getY()) + ", " + String(CR_posB[3].getZ()) + ")");

    Serial.print("CR_posC[0] = (" + String(CR_posC[0].getX()) + ", " + String(CR_posC[0].getY()) + ", " + String(CR_posC[0].getZ()) + ")");
    Serial.print("; CR_posC[1] = (" + String(CR_posC[1].getX()) + ", " + String(CR_posC[1].getY()) + ", " + String(CR_posC[1].getZ()) + ")");
    Serial.print("; CR_posC[2] = (" + String(CR_posC[2].getX()) + ", " + String(CR_posC[2].getY()) + ", " + String(CR_posC[2].getZ()) + ")");
    Serial.println("; CR_posC[3] = (" + String(CR_posC[3].getX()) + ", " + String(CR_posC[3].getY()) + ", " + String(CR_posC[3].getZ()) + ")");

    Serial.print("CR_posD[0] = (" + String(CR_posD[0].getX()) + ", " + String(CR_posD[0].getY()) + ", " + String(CR_posD[0].getZ()) + ")");
    Serial.print("; CR_posD[1] = (" + String(CR_posD[1].getX()) + ", " + String(CR_posD[1].getY()) + ", " + String(CR_posD[1].getZ()) + ")");
    Serial.print("; CR_posD[2] = (" + String(CR_posD[2].getX()) + ", " + String(CR_posD[2].getY()) + ", " + String(CR_posD[2].getZ()) + ")");
    Serial.println("; CR_posD[3] = (" + String(CR_posD[3].getX()) + ", " + String(CR_posD[3].getY()) + ", " + String(CR_posD[3].getZ()) + ")");
}

void Spider_Robot::incline(bool enableA, bool enableB, bool enableC, bool enableD, bool next){}

bool Spider_Robot::lift(bool enableA, bool enableB, bool enableC, bool enableD, bool next){
    
    if (hight_index>=9){
        Serial.println("Already at the highest position");
        return false;
    }
    hight_index++;

    //Returns the position for the higher hights
    float y = higher_pos[hight_index][0];
    float z = higher_pos[hight_index][1];

    //Computes the X coordiante giving the Y and thetas
    float x_AB = -(y/cos(INNER_theta1 - PI/2))*sin(INNER_theta1 - PI/2);
    float x_CD = -(y/cos(OUTTER_theta1 - PI/2))*sin(OUTTER_theta1 - PI/2);

    Position new_posA = Position(x_AB, y, z);
    Position new_posB = Position(x_AB, y, z);
    Position new_posC = Position(x_CD, y, z);
    Position new_posD = Position(x_CD, y, z);

    //Moves each leg to the new position (quickMove)
    if (enableA && next){
        legA.setDesiredFootPosition(new_posA);
        legA.moveTo(new_posA);
        legA.computeFootPosition();
    }
    if (enableB && next){
        legB.setDesiredFootPosition(new_posB);  
        legB.moveTo(new_posB);
        legB.computeFootPosition();
    }
    if (enableC && next){
        legC.setDesiredFootPosition(new_posC);
        legC.moveTo(new_posC);
        legC.computeFootPosition();
    }
    if (enableD && next){
        legD.setDesiredFootPosition(new_posD);
        legD.moveTo(new_posD);
        legD.computeFootPosition();
    }    
    
    //Updates the walking X, Y, Z base values for walking, as well as the step sizes
    walk_y = y;
    walk_z = z;
    walk_x_IN = x_AB;
    walk_x_OUT = x_CD;

    BIG_f_step_size = BIG_f_step_size - 0.5;
    SMALL_f_step_size = SMALL_f_step_size - 0.5;

    updateWalkingPositions();

    return true;

}

bool Spider_Robot::lower(bool enableA, bool enableB, bool enableC, bool enableD, bool next){
    
    if (hight_index<=0){
        return false;
    }

    hight_index--;

    //Returns the position for the higher hights
    float y = higher_pos[hight_index][0];
    float z = higher_pos[hight_index][1];

    //Computes the X coordiante giving the Y and thetas
    float x_AB = - (y/cos(INNER_theta1 - PI/2))*sin(INNER_theta1 - PI/2);
    float x_CD = - (y/cos(OUTTER_theta1 - PI/2))*sin(OUTTER_theta1 - PI/2);

    Position new_posA = Position(x_AB, y, z);
    Position new_posB = Position(x_AB, y, z);
    Position new_posC = Position(x_CD, y, z);
    Position new_posD = Position(x_CD, y, z);

    //Moves each leg to the new position (quickMove)
    if (enableA && next){
        legA.setDesiredFootPosition(new_posA);
        legA.moveTo(new_posA);
        legA.computeFootPosition();
    }
    if (enableB && next){
        legB.setDesiredFootPosition(new_posB);  
        legB.moveTo(new_posB);
        legB.computeFootPosition();
    }
    if (enableC && next){
        legC.setDesiredFootPosition(new_posC);
        legC.moveTo(new_posC);
        legC.computeFootPosition();
    }
    if (enableD && next){
        legD.setDesiredFootPosition(new_posD);
        legD.moveTo(new_posD);
        legD.computeFootPosition();
    }    
    
    //Updates the walking X, Y, Z base values for walking, as well as the step sizes
    walk_y = y;
    walk_z = z;
    walk_x_IN = x_AB;
    walk_x_OUT = x_CD;

    BIG_f_step_size = BIG_f_step_size + 0.5;
    SMALL_f_step_size = SMALL_f_step_size + 0.5;

    updateWalkingPositions();

    return true;
}

bool Spider_Robot::higherPositionAvailable(){
    return (hight_index<9);
}

bool Spider_Robot::lowerPositionAvailable(){
    return (hight_index>0);
}

int Spider_Robot::getHeigthIndex(){
    return hight_index;
}

void Spider_Robot::updateWalkingPositions(){

    W_posA[0] = Position(walk_x_IN, walk_y, walk_z);   
    W_posA[1] = Position(walk_x_IN + BIG_f_step_size, walk_y, walk_z);       
    W_posA[2] = Position(walk_x_OUT, walk_y, walk_z);      

    W_posB[0] = Position(walk_x_OUT + SMALL_f_step_size, walk_y, walk_z + 0.5);        
    W_posB[1] = Position(walk_x_IN, walk_y, walk_z + 0.5);      
    W_posB[2] = Position(walk_x_OUT, walk_y, walk_z + 0.5);

    W_posC[0] = Position(walk_x_IN, walk_y, walk_z);         
    W_posC[1] = Position(walk_x_OUT + SMALL_f_step_size, walk_y, walk_z);        
    W_posC[2] = Position(walk_x_IN, walk_y, walk_z);       

    W_posD[0] = Position(walk_x_IN + BIG_f_step_size - 0.5, walk_y, walk_z);           
    W_posD[1] = Position(walk_x_OUT, walk_y - 0.5, walk_z);        
    W_posD[2] = Position(walk_x_IN, walk_y - 0.5, walk_z);    

    // Lateral Walking positions ----------------------------------------

    LW_posA[0] = Position(-0.3650 , walk_y - lat_step_size/2, walk_z);
    LW_posA[1] = Position(-0.0865, walk_y - lat_step_size, walk_z);
    LW_posA[2] = Position(-0.650, walk_y, walk_z);

    LW_posB[0] = Position(-1.210, walk_y + lat_step_size, walk_z + 0.35);
    LW_posB[1] = Position(-0.9315, walk_y + lat_step_size/2, walk_z + 0.35);
    LW_posB[2] = Position(-0.65, walk_y, walk_z + 0.35);

    LW_posC[0] = Position(2.6517, walk_y + lat_step_size/2, walk_z);
    LW_posC[1] = Position(1.0483, walk_y - lat_step_size/2, walk_z);
    LW_posC[2] = Position(1.85, walk_y, walk_z);

    LW_posD[0] = Position(2.6517, walk_y + lat_step_size/2, walk_z);
    LW_posD[1] = Position(1.0483, walk_y - lat_step_size/2, walk_z);
    LW_posD[2] = Position(1.85, walk_y, walk_z);

    // Rotation positions ----------------------------------------------
    
    float x_rot = walk_y * sin(rot_angle);
    float y_rot = walk_y * cos(rot_angle);

    R_posA[0] = Position(0, walk_y, walk_z);
    R_posA[1] = Position(-x_rot, y_rot, walk_z);
    R_posA[2] = Position(0, walk_y, walk_z);
    
    R_posB[0] = Position(0, walk_y, walk_z + 0.35);
    R_posB[1] = Position(x_rot, y_rot, walk_z + 0.35);
    R_posB[2] = Position(0, walk_y, walk_z + 0.35);
    
    R_posC[0] = Position(0, walk_y, walk_z);
    R_posC[1] = Position(-x_rot, y_rot, walk_z);
    R_posC[2] = Position(0, walk_y, walk_z);
    
    R_posD[0] = Position(0, walk_y, walk_z);
    R_posD[1] = Position(x_rot, y_rot, walk_z);
    R_posD[2] = Position(0, walk_y, walk_z);

    // Trajectory Reset -----------------------------------------------

    legA.resetAllTrajectoryComputations();
    legB.resetAllTrajectoryComputations();
    legC.resetAllTrajectoryComputations();
    legD.resetAllTrajectoryComputations();

}

void Spider_Robot::stabilise(float roll, float pitch){
    float Ly = 9.25;
    float Lx = 8.0;
    float L1 = 3.35;
    float L2 = 4.7;
    float L3 = 8.2;

    roll = roll *(PI/180.0);
    pitch = pitch *(PI/180.0);

    float thetaC = -(8/3)*roll;

    float delta_z_roll = tan(roll)*(Ly/2 + L1 + L2*cos(thetaC));
    float delta_z_pitch = (Lx/2)*sin(pitch);

    Position c_pos = legA.getDesiredFootPosition();

    Position higher_pos = Position(c_pos.getX(), c_pos.getY(), c_pos.getZ() + delta_z_roll);
    Position lower_pos = Position(c_pos.getX(), c_pos.getY(), c_pos.getZ() - delta_z_roll);

    Position new_A_pos = Position(c_pos.getX(), c_pos.getY(), c_pos.getZ() + delta_z_roll - delta_z_pitch);
    Position new_B_pos = Position(c_pos.getX(), c_pos.getY(), c_pos.getZ() + delta_z_roll + delta_z_pitch);
    Position new_C_pos = Position(c_pos.getX(), c_pos.getY(), c_pos.getZ() - delta_z_roll + delta_z_pitch);
    Position new_D_pos = Position(c_pos.getX(), c_pos.getY(), c_pos.getZ() - delta_z_roll - delta_z_pitch);

    legA.moveTo(new_A_pos);
    legB.moveTo(new_B_pos);
    legC.moveTo(new_C_pos);
    legD.moveTo(new_D_pos);

    Serial.println("Delta Z ROLL = " + String(delta_z_roll) + " | Delta Z PITCH = " + String(delta_z_pitch));


}

int Spider_Robot::boolToInt(bool b){
    if (b) {return 1;}
    else {return 0;}
}