// TOGGLE MODES

#define MAIN_CODE 1
#define DEBUG 0
#define TESTS 0
#define I2C_SCAN 0

// --------------------------------------------------------------

#if MAIN_CODE

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>
#include "leg.h"
#include "spider_robot.h"
#include "position.h"

uint32_t now, last_time;
uint32_t last_lift;

uint32_t intervals[] = {10, 50, 100, 250, 500, 1000};
int interval_index = 2;

float temp1;

#define LEG_A_THETA1_OFFSET -7.5
#define LEG_A_THETA2_OFFSET 4.5
#define LEG_A_THETA3_OFFSET -2.0

#define LEG_B_THETA1_OFFSET 0.0
#define LEG_B_THETA2_OFFSET -13.0
#define LEG_B_THETA3_OFFSET 8.0

#define LEG_C_THETA1_OFFSET -4.5
#define LEG_C_THETA2_OFFSET -.5
#define LEG_C_THETA3_OFFSET -4.5

#define LEG_D_THETA1_OFFSET 6.5
#define LEG_D_THETA2_OFFSET -8.0
#define LEG_D_THETA3_OFFSET -3.0

Adafruit_PWMServoDriver pwm_driver = Adafruit_PWMServoDriver();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

MPU6050 mpu;
#define PITCH_OFFSET 4.30
#define ROLL_OFFSET 177.10

float distance = 0.0;

const int MPU = 0x68; //I2C address of the MPU-6050

int16_t ax, ay, az, gx, gy, gz;
float accelX, accelY, accelZ, roll, pitch;

// Constants for servo control
#define FREQUENCY 50 // Frequency for servos (50Hz)

// For Testing Purposes ------------------------------------

float pulse_Length1 = map(90 + LEG_B_THETA1_OFFSET, 0.0, 180.0, 150.0, 580.0);
float pulse_Length2 = map(90 + LEG_B_THETA2_OFFSET, 0.0, 180.0, 150.0, 580.0);
float pulse_Length3 = map(90 + LEG_B_THETA3_OFFSET, 0.0, 180.0, 150.0, 580.0);  
float pulse_length4 = 0;

float yr, theta_1, L1, J3, J2, theta1, theta2, theta3, x, y, z;

float xp, yp, zp;
float delta_theta = 0;

float cx, cy, cz;

bool L, H, N;
bool A, B, C, D, E, G, J, K;

float tA, tB, tC, tD;
float aux1, aux2;
float t1, t2, t3;
int servo_test_id = 6;

Position test_pos (0.0, 0.0, 0.0);

//FSM VARIABLES --------------------------------------------

bool S, W, I, T, l, R, u, d, O, M, X, FASTER, SLOWER;

typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

enum {
  sm1_start = 0,
  sm1_init,
  sm1_test_pos,
  sm1_testA,
  sm1_testB,
  sm1_testC,
  sm1_testD,
  sm1_idle,
  sm1_walk_straight,
  sm1_walk_around_obstacles,
  sm1_rotate,
  sm1_lateral_walk,
  sm1_lift,
  sm1_lower,
  sm1_stabilize
};

enum{
  sm2_idle = 0,
  sm2_long_walk,
  sm2_rotate_cw1,
  sm2_small_walk1,
  sm2_rotate_acw1,
  sm2_small_walk2,
  sm2_rotate_acw2,
  sm2_small_walk3,
  sm2_rotate_cw2,
};

enum{
  sm3_idle = 0,
  sm3_faster, 
  sm3_slower
};

int previous_state = 0;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

fsm_t robot_fsm;
fsm_t walk_around_obstacles_fsm;
fsm_t speed_fsm;

// Robot Variables -------------------------------------------

Spider_Robot spider;

LEG legA(0, 1, 2, 3.65, 4.75, 8.2, LEG_A_THETA1_OFFSET, LEG_A_THETA2_OFFSET, LEG_A_THETA3_OFFSET, 'A');
LEG legB(3, 4, 5, 3.65, 5.3, 8.2, LEG_B_THETA1_OFFSET, LEG_B_THETA2_OFFSET, LEG_B_THETA3_OFFSET, 'B');
LEG legC(6, 7, 8, 3.35, 4.75, 8.2, LEG_C_THETA1_OFFSET, LEG_C_THETA2_OFFSET, LEG_C_THETA3_OFFSET, 'C');
LEG legD(9, 10, 11, 3.65, 5.3, 8.2, LEG_D_THETA1_OFFSET, LEG_D_THETA2_OFFSET, LEG_D_THETA3_OFFSET, 'D');

Position initLegA(1.85, 3.75, -5.9);
Position initLegB(1.85, 3.75, -5.2);
Position initLegC(-0.65, 3.75, -5.7);
Position initLegD(-0.65, 3.75, -5.2);

Position walk_pos(120,0,0);

bool rot_dir = false;
float dist_walked = 0.0;
float start_x = 0.0;

// FIR -------------------------------------------------------

typedef struct {
  float entries[5];
  float output;
} FIR;

void computeFIROutput(FIR& fir){
  float sum = 0;
  for (int i = 0; i < 5; i++){
    sum += fir.entries[i];
  }
  fir.output = sum/5;
}

void addEntry(FIR& fir, float new_entry){
  for (int i = 4; i > 0; i--){
    fir.entries[i] = fir.entries[i-1];
  }
  fir.entries[0] = new_entry;
  for(int j = 0; j<5;j++){
  }
  computeFIROutput(fir);
}

FIR TOF_FIR, IMU_PITCH_FIR, IMU_ROLL_FIR;

// Functions -------------------------------------------------

float rad2deg(float rad){
    return rad*180.0/PI;
}

float deg2rad(float deg){
    return deg*PI/180.0;
}

void readIMU(){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw accelerometer data to g-force
    accelX = ax / 16384.0;  // MPU6050 full-scale range ±2g
    accelY = ay / 16384.0;
    accelZ = az / 16384.0;

    // Calculate roll and pitch angles (in degrees)
    roll  = ROLL_OFFSET + atan2(accelY, accelZ) * 180.0 / PI;
    pitch = PITCH_OFFSET + atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

    if (roll > 180.0){roll = roll - 360.0;}

    if (abs(roll) < 2.0){roll = 0.0;}
    if (abs(pitch) < 2.0){pitch = 0.0;}

    addEntry(IMU_PITCH_FIR, pitch);
    addEntry(IMU_ROLL_FIR, roll);
}

void setup() {
    Wire.begin();
    pwm_driver.begin();
    pwm_driver.setPWMFreq(FREQUENCY);
    lox.begin();

    Serial.begin(9600);

    mpu.initialize(); // Initialize MPU6050

    // Check if the MPU6050 is connected
    while(!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        delay(50);
    }
    Serial.println("MPU6050 connected.");

    pinMode(25, OUTPUT); //Debugging LED

    last_time = 0;

    legA.attachServoDriver(Adafruit_PWMServoDriver());
    legB.attachServoDriver(Adafruit_PWMServoDriver());
    legC.attachServoDriver(Adafruit_PWMServoDriver());
    legD.attachServoDriver(Adafruit_PWMServoDriver());

    spider.attachLegs(legA, legB, legC, legD);

    set_state(robot_fsm, sm1_start);

    Serial.println("SET UP...");

    for (int i = 0; i < 5; i++){
      TOF_FIR.entries[i] = 0.0;
      IMU_PITCH_FIR.entries[i] = 0.0;
      IMU_ROLL_FIR.entries[i] = 0.0;
    }
    computeFIROutput(TOF_FIR);
    computeFIROutput(IMU_PITCH_FIR);
    computeFIROutput(IMU_ROLL_FIR);

  }

void loop() {
    
    uint8_t b;

    VL53L0X_RangingMeasurementData_t measure;
    
    //Check serial comm
    if (Serial.available()) {  
      b = Serial.read();   
      if (b == 'I') I = true;
      else if (b == 'W') W = true;
      else if (b == 'S') S = true;
      else if (b == 'T') T = true;
      else if (b == 'L') L = true;
      else if (b == 'H') H = true;
      else if (b == 'N') N = true;
      else if (b == 'A') A = true;
      else if (b == 'B') B = true;
      else if (b == 'C') C = true;
      else if (b == 'D') D = true;
      else if (b == 'E') E = true;
      else if (b == 'G') G = true;
      else if (b == 'J') J = true;
      else if (b == 'K') K = true;
      else if (b == 'l') l = true;
      else if (b == 'R') R = true;
      else if (b == 'u') u = true;
      else if (b == 'd') d = true;
      else if (b == 'O') O = true;
      else if (b == 'X') X = true;
      else if (b == '1') SLOWER = true;
      else if (b == '2') FASTER = true;
    } 

    now = millis();

    //Serial.print("[BEGINING] NOW = " + String(now) + ", Last time = " + String(last_time) + "\n");

    if ((now-last_time > intervals[interval_index])) {

      Serial.print("NEW CYCLE -------------------------------------------------------------|\n");
     
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        distance = float(measure.RangeMilliMeter) - 25.0;
        Serial.print("Distance (mm): " + String(distance)); 
        addEntry(TOF_FIR, distance);
        M = (TOF_FIR.output <= 200.0);
        Serial.println("| M = " + String(TOF_FIR.output));
      } else {
        Serial.println("TOF sensor out of range ");
        M = false;
      }

      //Read accelerometer and gyroscope values
      readIMU();

      // Print values
      Serial.print("Pitch: "); Serial.print(IMU_PITCH_FIR.output);
      Serial.print(" | Roll: "); Serial.println(IMU_ROLL_FIR.output);

      //(1/4) UPDATE TIME IN STATES
      uint32_t cur_time = now;   // Just one call to millis()
      robot_fsm.tis = cur_time - robot_fsm.tes;

      if (true){
      //(2/4) FIRE TRANSITIONS
      if (robot_fsm.state == sm1_start && I){
        robot_fsm.new_state = sm1_init;
      } else if (robot_fsm.state == sm1_start && T){
        robot_fsm.new_state = sm1_test_pos;
      } else if (robot_fsm.state == sm1_init){
        robot_fsm.new_state = sm1_idle;
      } else if (robot_fsm.state == sm1_test_pos && S){
        robot_fsm.new_state = sm1_start;
      } else if (robot_fsm.state == sm1_idle && W){
        robot_fsm.new_state = sm1_walk_straight;
        spider.START_WALKING = true;
      } else if (robot_fsm.state == sm1_idle && T){
        robot_fsm.new_state = sm1_test_pos;      
      } else if (robot_fsm.state == sm1_idle && l){
        robot_fsm.new_state = sm1_lateral_walk;
        spider.START_LATERAL_WALKING = true;
      } else if (robot_fsm.state == sm1_idle && R){
        robot_fsm.new_state = sm1_rotate;
        spider.START_ROTATING = true;
      } else if (robot_fsm.state == sm1_idle && u){
        robot_fsm.new_state = sm1_lift;
      } else if (robot_fsm.state == sm1_idle && d){
        robot_fsm.new_state = sm1_lower;
      } else if (robot_fsm.state == sm1_idle && O){
        robot_fsm.new_state = sm1_walk_around_obstacles;
        walk_around_obstacles_fsm.new_state = sm2_idle;
        spider.START_WALKING = true;
      } else if (robot_fsm.state == sm1_idle && X){
        robot_fsm.new_state = sm1_stabilize;
        spider.legA.moveTo(Position(0, 3.75, -5.7));
        spider.legB.moveTo(Position(0, 3.75, -5.3));
        spider.legC.moveTo(Position(0, 3.75, -5.7));
        spider.legD.moveTo(Position(0, 3.75, -5.7));
      } else if (robot_fsm.state == sm1_lift){
        robot_fsm.new_state = sm1_idle;
      } else if (robot_fsm.state == sm1_lower){
        robot_fsm.new_state = sm1_idle;
      } else if (robot_fsm.state == sm1_walk_straight && S){
        robot_fsm.new_state = sm1_idle;
        spider.START_WALKING = false;
      } else if (robot_fsm.state == sm1_lateral_walk && S){
        robot_fsm.new_state = sm1_idle;
        spider.START_LATERAL_WALKING = false;
      } else if (robot_fsm.state == sm1_rotate && S){
        robot_fsm.new_state = sm1_idle;
        spider.START_ROTATING = false;
      } else if (robot_fsm.state == sm1_walk_around_obstacles && S){
        robot_fsm.new_state = sm1_idle;
        walk_around_obstacles_fsm.new_state = sm2_idle;
        spider.START_WALKING = false;
      } else if (robot_fsm.state == sm1_walk_around_obstacles && spider.DesiredLocationReached() && walk_around_obstacles_fsm.state == sm2_idle){
        robot_fsm.new_state = sm1_idle;
      } else if (robot_fsm.state == sm1_stabilize && S){
        robot_fsm.new_state = sm1_idle;
      } else if (robot_fsm.state == sm1_test_pos && A){
        robot_fsm.new_state = sm1_testA;
      } else if (robot_fsm.state == sm1_test_pos && B){
        robot_fsm.new_state = sm1_testB;
      } else if (robot_fsm.state == sm1_test_pos && C){
        robot_fsm.new_state = sm1_testC;
      } else if (robot_fsm.state == sm1_test_pos && D){
        robot_fsm.new_state = sm1_testD;
      } else if (robot_fsm.state == sm1_testA && T){
        robot_fsm.new_state = sm1_test_pos;
      } else if (robot_fsm.state == sm1_testB && T){
        robot_fsm.new_state = sm1_test_pos;
      } else if (robot_fsm.state == sm1_testC && T){
        robot_fsm.new_state = sm1_test_pos;
      } else if (robot_fsm.state == sm1_testD && T){
        robot_fsm.new_state = sm1_test_pos;
      }
      }

      if (speed_fsm.state == sm3_idle && FASTER){
        speed_fsm.new_state = sm3_faster;
      } else if (speed_fsm.state == sm3_idle && SLOWER){
        speed_fsm.new_state = sm3_slower;
      } else if (speed_fsm.state == sm3_faster){
        speed_fsm.new_state = sm3_idle;
      } else if (speed_fsm.state == sm3_slower){
        speed_fsm.new_state = sm3_idle;
      }

      //(3/4) - UPDATE STATES
      set_state(robot_fsm, robot_fsm.new_state);
      set_state(speed_fsm, speed_fsm.new_state);

      Serial.println("'ROBOT' FSM STATE: " + String(robot_fsm.state) + " | INPUTS: W = " + String(W) + ", S = " + String(S) + ", I = " + String(I) + ", T = " + T + ", H = " + H + ", L = " + L + ", N = " + N + ", R = " + R + ", l = " + l + ", u = " + u + ", d = " + d);
      Serial.println("'WALK OVER OBSTACLES' FSM STATE: " + String(walk_around_obstacles_fsm.state));
      Serial.println("Speed interval (" + String(interval_index) + "): " + String(intervals[interval_index]) + " | FASTER = " + String(FASTER) + " | SLOWER = " + String(SLOWER));
      Serial.println("Robot Angle: " + String(spider.getCurrentAngle()));
      spider.legA.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG A: POS = (" + String(spider.legA.getCurrentFootPosition().getX()) + ", " + String(spider.legA.getCurrentFootPosition().getY()) + ", " + String(spider.legA.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      spider.legB.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG B: POS = (" + String(spider.legB.getCurrentFootPosition().getX()) + ", " + String(spider.legB.getCurrentFootPosition().getY()) + ", " + String(spider.legB.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      spider.legC.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG C: POS = (" + String(spider.legC.getCurrentFootPosition().getX()) + ", " + String(spider.legC.getCurrentFootPosition().getY()) + ", " + String(spider.legC.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      spider.legD.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG D: POS = (" + String(spider.legD.getCurrentFootPosition().getX()) + ", " + String(spider.legD.getCurrentFootPosition().getY()) + ", " + String(spider.legD.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");  
      Serial.println("ROBOT: POS = (" + String(spider.getCurrentLocation().getX()) + ", " + String(spider.getCurrentLocation().getY()) + ", " + String(spider.getCurrentLocation().getZ()) + ") | Desired POS = (" + String(spider.getDesiredLocation().getX()) + ", " + String(spider.getDesiredLocation().getY()) + ", " + String(spider.getDesiredLocation().getZ()) + ")");
      Serial.println("Height Index = " + String(spider.getHeigthIndex()));    

      //(4/4) - ACTIONS
      switch (robot_fsm.state) {
      case sm1_start:
        spider.legA.hardReset();
        spider.legB.hardReset();
        spider.legC.hardReset();
        spider.legD.hardReset();

        break;
      case sm1_init:
        Serial.print("[LEG A]");
        spider.legA.initializePosition(initLegA);
        spider.legA.getCurrentJointAngles(aux1, aux2, tA);
        Serial.print("[LEG B]");
        spider.legB.initializePosition(initLegB);
        spider.legB.getCurrentJointAngles(aux1, aux2, tB);
        Serial.print("[LEG C]");
        spider.legC.initializePosition(initLegC);
        spider.legC.getCurrentJointAngles(aux1, aux2, tC);
        Serial.print("[LEG D]");
        spider.legD.initializePosition(initLegD);
        spider.legD.getCurrentJointAngles(aux1, aux2, tD);    

        break;
      case sm1_idle:
        
        break;
      case sm1_walk_straight:
        spider.walkTo(true, true, true, true, true, walk_pos);
        break;
      case sm1_lateral_walk:
        spider.lateral_walk(true, true, true, true, N);
        break;
      case sm1_rotate:
        if (D) {rot_dir = !rot_dir;}
        spider.continuosRotation(true, true, true, true, true, rot_dir);
        break;
      case sm1_lift:
        spider.lift(true, true, true, true, true);
        break;
      case sm1_lower:
        spider.lower(true, true, true, true, true);
        break;
      case sm1_walk_around_obstacles:
        if (walk_around_obstacles_fsm.state == sm2_idle){
          walk_around_obstacles_fsm.new_state = sm2_long_walk;
        } else if (walk_around_obstacles_fsm.state == sm2_long_walk && M){
          walk_around_obstacles_fsm.new_state = sm2_rotate_cw1;
          spider.START_ROTATING = true;
          Serial.println("==================================================================================================");
          Serial.println("ROTATING CW1 =====================================================================================");
          Serial.println("==================================================================================================");
          spider.stop();
        } else if (walk_around_obstacles_fsm.state == sm2_rotate_cw1 && spider.getCurrentAngle() >= 80.0){
          walk_around_obstacles_fsm.new_state = sm2_small_walk1;
          spider.reInitializePositions();
          spider.START_WALKING = true;
          start_x = spider.getCurrentLocation().getX();
          dist_walked = 0.0;
          Serial.println("==================================================================================================");
          Serial.println("WALKING ==========================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_small_walk1 && (dist_walked >= 20.0)){
          walk_around_obstacles_fsm.new_state = sm2_rotate_acw1;
          spider.START_ROTATING = true;
          Serial.println("==================================================================================================");
          Serial.println("ROTATING ACW1 =====================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_rotate_acw1 && spider.getCurrentAngle() <= 0.0){
          walk_around_obstacles_fsm.new_state = sm2_small_walk2;
          spider.reInitializePositions();
          spider.START_WALKING = true;
          start_x = spider.getCurrentLocation().getX();
          dist_walked = 0.0;
          Serial.println("==================================================================================================");
          Serial.println("WALKING ==========================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_small_walk2 && (dist_walked >= 20.0)){
          walk_around_obstacles_fsm.new_state = sm2_rotate_acw2;
          spider.START_ROTATING = true;
          Serial.println("==================================================================================================");
          Serial.println("ROTATING ACW2 =====================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_rotate_acw2 && spider.getCurrentAngle() <= -80.0){
          walk_around_obstacles_fsm.new_state = sm2_small_walk3;
          spider.reInitializePositions();
          spider.START_WALKING = true;
          start_x = spider.getCurrentLocation().getX();
          dist_walked = 0.0;
          Serial.println("==================================================================================================");
          Serial.println("WALKING ==========================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_small_walk3 && (dist_walked >= 20.0)){
          walk_around_obstacles_fsm.new_state = sm2_rotate_cw2;
          spider.START_ROTATING = true;
          Serial.println("==================================================================================================");
          Serial.println("ROTATING CW2 =====================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_rotate_cw2 && spider.getCurrentAngle() >= 0.0){
          walk_around_obstacles_fsm.new_state = sm2_long_walk;
          spider.reInitializePositions();
          spider.START_WALKING = true;
          Serial.println("==================================================================================================");
          Serial.println("WALKING ==========================================================================================");
          Serial.println("==================================================================================================");
        } else if (walk_around_obstacles_fsm.state == sm2_long_walk && spider.DesiredLocationReached()){
          walk_around_obstacles_fsm.new_state = sm2_idle;
        }
        
        set_state(walk_around_obstacles_fsm, walk_around_obstacles_fsm.new_state);

        switch(walk_around_obstacles_fsm.state){
          case sm2_idle:
            walk_pos.setX(walk_pos.getX() + 100.0);
            break;
          case sm2_long_walk:
            spider.walkTo(true, true, true, true, true, walk_pos);
            break;
          case sm2_rotate_cw1:
            spider.continuosRotation(true, true, true, true, true, true);
            break;
          case sm2_small_walk1:
            spider.walkTo(true, true, true, true, true, walk_pos);
            dist_walked += abs(spider.getCurrentLocation().getX() - start_x);
            break;
          case sm2_rotate_acw1:
            spider.continuosRotation(true, true, true, true, true, false);
            break;
          case sm2_small_walk2:
            spider.walkTo(true, true, true, true, true, walk_pos);
            dist_walked += abs(spider.getCurrentLocation().getX() - start_x);
            break;
          case sm2_rotate_acw2:
            spider.continuosRotation(true, true, true, true, true, false);
            break;
          case sm2_small_walk3:
            spider.walkTo(true, true, true, true, true, walk_pos);
            dist_walked += abs(spider.getCurrentLocation().getX() - start_x);
            break;
          case sm2_rotate_cw2:
            spider.continuosRotation(true, true, true, true, true, true);
            break;
          default:
            break;
        }

        break;
      case sm1_stabilize:
        spider.stabilise(IMU_ROLL_FIR.output, IMU_PITCH_FIR.output);

        break;
      case sm1_test_pos:

        xp = 0.0;
        yp = 0.0;
        zp = 0.0;

        if (H) {delta_theta+=0.1;}
        if (L) {delta_theta-=0.1;}
        if (N) {servo_test_id++;delta_theta=0;}

        //addEntry(IMU_ROLL_FIR, roll);

        break;
      default:
        break;
      }

      switch(speed_fsm.state){
        case sm3_idle:
          break;
        case sm3_faster:
          if (interval_index > 0){
            interval_index--;
          } else {
            Serial.println("MAXIMUM SPEED REACHED!");
          }
          break;
        case sm3_slower:
          if (interval_index < 5){
            interval_index++;
          } else {
            Serial.println("MINIMUM SPEED REACHED!");
          }
          break;
        default:
          break;
      }

     Serial.println("End of Cycle ----------------------------------------------------------|");

      I = W = S = T = L = H = N = A = B = C = D = E = G = J = K = l = R = u = d = O = M = X = FASTER = SLOWER = false;
      
      last_time = now;
      
    }
     
    //Serial.print("[END] Now = " + String(now) + ", Last_time = " + String(last_time) + ", Interval = " + String(interval) + "\n");

}
#endif

// ------------------------------------------------------------------------------------------------

#if DEBUG
#include <Arduino.h>
void setup() {
    pinMode(25, OUTPUT); // Set GPIO 25 as an output
    Serial.begin(9600);
    
}


void loop() {
    digitalWrite(25, HIGH); // Turn the LED on
    delay(50);             // Wait for 500 milliseconds
    Serial.println("OLAAAAA");
    digitalWrite(25, LOW);  // Turn the LED off
    delay(50);             // Wait for 500 milliseconds
}
#endif

// ------------------------------------------------------------------------------------------------

#if TESTS

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>

// Define I2C bus for Raspberry Pi Pico

VL53L0X sensor;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Initialize I2C
    Wire.begin();
    
    // Initialize VL53L0X sensor
    if (!sensor.init(&Wire)) {
        Serial.println("Failed to initialize VL53L0X sensor!");
        while (1);
    }
    sensor.setTimeout(500);
    sensor.startContinuous();
    Serial.println("VL53L0X initialized successfully!");
}

void loop() {
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
        Serial.println("Sensor timeout!");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" mm");
    }
    delay(100);
}


#endif

#if I2C_SCAN

#include <Wire.h>
#include <stdio.h>
#include <Arduino.h>

void scan_i2c_bus() {
    Serial.println("Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.println("Device found at address 0x" + String(addr, HEX));
        }
    }
    Serial.println("Scan complete.\n");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000); // Allow time for serial connection
}

void loop() {
    scan_i2c_bus();
    delay(5000);
}

#endif