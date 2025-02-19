#ifndef SPIDER_ROBOT_H
#define SPIDER_ROBOT_H
#include <cmath>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "leg.h"
#include "position.h"

#define INIT_LEG1_THETA1 90
#define INIT_LEG1_THETA2 150.67
#define INIT_LEG1_THETA3 149.27
#define INIT_LEG2_THETA1 90
#define INIT_LEG2_THETA2 150.67
#define INIT_LEG2_THETA3 149.27
#define INIT_LEG3_THETA1 90
#define INIT_LEG3_THETA2 150.67
#define INIT_LEG3_THETA3 149.27
#define INIT_LEG4_THETA1 90
#define INIT_LEG4_THETA2 150.67
#define INIT_LEG4_THETA3 149.27

class Spider_Robot{
private:

    Position W_posA[3], W_posB[3], W_posC[3], W_posD[3];
    Position LW_posA[3], LW_posB[3], LW_posC[3], LW_posD[3];
    Position R_posA[3], R_posB[3], R_posC[3], R_posD[3];
    Position CR_posA[4], CR_posB[4], CR_posC[4], CR_posD[4];
    Position I_posA[3], I_posB[3], I_posC[3], I_posD[3];

    Position Current_location, Desired_location;

    float current_angle;

    //Theta 1 for INITAL POSITION, in radians, for Legs A (Innner) and C (Outter)
    #define INNER_theta1  1.7424
    #define OUTTER_theta1 1.1125

    float higher_pos[10][2];
    int hight_index;

    int pos_index;
    float walk_z, walk_y, walk_x_OUT, walk_x_IN;
    float BIG_f_step_size, SMALL_f_step_size, lat_step_size;
    float rot_angle;

    float delta_z;

    float robot_step_size;

    void updateWalkingPositions();
    void stepForward();
    void toggleRotationDirection();
    bool rotation_direction; //true = clockwise, false = counterclockwise

    typedef struct {
        int state, new_state;
        unsigned long tes, tis;
    } fsm;

    fsm walk_fsm, lateral_walk_fsm, rotate_fsm, incline_fsm, lift_fsm, continuos_rotation_fsm;

    void setNewState(fsm& sm, int new_state);

    enum {
        sm_idle = 0,
        sm_compute,
        sm_moving
    };

    int boolToInt(bool b);

public:

    bool START_WALKING = false;
    bool START_LATERAL_WALKING = false;
    bool START_ROTATING = false;
    bool START_INCLINING = false;
    bool START_LIFTING = false;

    LEG legA;
    LEG legB; 
    LEG legC;
    LEG legD;
    //Spider_Robot(LEG& legA, LEG& legB, LEG& legC, LEG& legD);
    Spider_Robot();
    void attachLegs(LEG& legA, LEG& legB, LEG& legC, LEG& legD);
    void initializePositions(Position init_posA, Position init_posB, Position init_posC, Position init_posD);
    void reInitializePositions();

    void walkTo(bool enableA, bool enableB, bool enableC, bool enableD, bool next, Position Desired_location);
    void lateral_walk(bool enableA, bool enableB, bool enableC, bool enableD, bool next);
    void rotate(bool enableA, bool enableB, bool enableC, bool enableD, bool next);
    void continuosRotation(bool enableA, bool enableB, bool enableC, bool enableD, bool next, bool directio);
    void incline(bool enableA, bool enableB, bool enableC, bool enableD, bool next);
    bool lift(bool enableA, bool enableB, bool enableC, bool enableD, bool next);
    bool lower(bool enableA, bool enableB, bool enableC, bool enableD, bool next);
    bool higherPositionAvailable();
    bool lowerPositionAvailable();
    int getHeigthIndex();

    void stabilise(float roll, float pitch);

    void stop();
    bool legsOnDesiredPositions();
    int getPosIndex();

    Position getCurrentLocation();
    void setCurrentLocation(Position pos);

    Position getDesiredLocation();
    void setDesiredLocation(Position pos);

    float getCurrentAngle();

    bool DesiredLocationReached();

    void printCRPositions();

};

#endif