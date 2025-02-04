#ifndef LEG_H
#define LEG_H
#include <cmath>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "position.h"

class LEG {
private: 
        Position current_pos, desired_pos;

        char type;
        
        float current_theta1, current_theta2, current_theta3;
        float desired_theta1, desired_theta2, desired_theta3;

        #define wtp 10          //Walking Trajectory Points
        #define rtp 5           //Rotation Trajectory Points
        #define ltp 5           //Lifting Trajectory Points

        //Walking Trajectories --------------------------------------
        Position W_first_trajectory_points[wtp];         //index = 1
        Position W_second_trajectory_points[wtp];        //index = 2
        Position W_third_trajectory_points[wtp];         //index = 3
        bool W_trajectory1_computed, W_trajectory2_computed, W_trajectory3_computed;

        //Rotation Trajectories --------------------------------------
        Position R_first_trajectory_points[rtp];          //index = 4
        Position R_second_trajectory_points[rtp];         //index = 5
        bool R_trajectory1_computed, R_trajectory2_computed;


        Position CR_first_trajectory_points[rtp];         //index = 6
        Position CR_second_trajectory_points[rtp];        //index = 7
        Position CR_third_trajectory_points[rtp];         //index = 8
        Position CR_fourth_trajectory_points[rtp];        //index = 9
        bool CR_trajectory1_computed, CR_trajectory2_computed, CR_trajectory3_computed, CR_trajectory4_computed;

        //Lifting Trajectory --------------------------- [DESCONTINUED]
        Position L_first_trajectory_points[ltp];          
        bool L_trajectory_computed;

        int t_index;

        float delta_theta1, delta_theta2, delta_theta3;
        float LEG1_LENGTH, LEG2_LENGTH, LEG3_LENGTH;
        float theta1_offset, theta2_offset, theta3_offset;
        int servo1_id, servo2_id, servo3_id;
        bool isOnDesiredPosition, isOnTrajectory;

        bool compareValues(float t1, float t2, float df);
    
        //Cinemática ------------------------------------------------
        
        float rad2deg (float rad);
        float deg2rad (float deg);
        void computeInverseKinematics (float x, float y, float z, float& theta1, float& theta2, float& theta3);
        void computeForwardKinematics (float theta1, float theta2, float theta3, float& x, float& y, float& z);
        void computeBezierCurve(Position c_pos, Position d_pos, Position half_pos, Position (&trajectory)[20], int n_points);

        Adafruit_PWMServoDriver pwm_driver;

public:
    //Constructor
    LEG(int servo_id1, int servo_id2, int servo_id3, float Length1, float length2, float length3, float to1, float to2, float to3, char type);
    LEG();

    //Attach the servo driver
    void attachServoDriver(Adafruit_PWMServoDriver pwm_driver);

    //Get the current position of the foot
    Position getCurrentFootPosition();

    //Get the desired position of the foot
    Position getDesiredFootPosition();

    void setDesiredFootPosition(Position pos);

    //Computes the position of the foot based on the current joint angles
    void computeFootPosition();

    void quickMove(float it1, float it2, float it3);

    //Get the current joint angles
    void getCurrentJointAngles(float& t1, float& t2, float& t3);

    //Get the desired joint angles
    void getDesiredJointAngles(float& t1, float& t2, float& t3);

    //Computes the trajectory to reach the desired position
    void computeTrajectory(Position pos, int mode, int trajectory_index);

    //Checks if the respective trajectory has already been previously computed
    bool checkTrajectoryComputation(int trajectory_index);

    void resetTrajectoryComputation(int trajectory_index);

    void resetAllTrajectoryComputations();

    //Moves the foot to the desired position (already saved internally) in a step defined by df as a percentage of the total distance (deltas)
    void moveFootBy(float df);

    //Moves foot to the desired position;
    void moveTo(Position pos);

    //Moves the foot to the next point on the defined trajectory
    void moveOnTrajectory(int trajectory_index);
    
    //Stops the leg and forces the current angles
    void stop();

    //Computes the required joint angles to reach the desired position, and updates the internal angle, position and delta variables
    void computeJointAngles(float x, float y, float z, float& t1, float& t2, float& t3);
    
    //Checks if the foot is on the desired position, and updates the internal variables
    bool checkDesiredPosition();

    //Initializes the position of the leg and updates the internal variables
    void initializePosition(Position init_pos); 

    void hardReset();

    void forceJointAngle(int theta, float angle);

    void resetTrajectory();
};

#endif