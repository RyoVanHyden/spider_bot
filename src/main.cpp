
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Adafruit_VL53L0X.h>
#include "leg.h"
#include "spider_robot.h"
#include "position.h"

uint32_t now, last_time;

uint32_t interval = 5;

float temp1;

#define LEG_A_THETA1_OFFSET -0.5
#define LEG_A_THETA2_OFFSET -4.5
#define LEG_A_THETA3_OFFSET -10.0

#define LEG_C_THETA1_OFFSET -12.5
#define LEG_C_THETA2_OFFSET -12.5
#define LEG_C_THETA3_OFFSET -10.0

#define LEG_B_THETA1_OFFSET -16.0
#define LEG_B_THETA2_OFFSET -16.5
#define LEG_B_THETA3_OFFSET -11.5

#define LEG_D_THETA1_OFFSET 4.0
#define LEG_D_THETA2_OFFSET -0.5
#define LEG_D_THETA3_OFFSET 0.0

Adafruit_PWMServoDriver pwm_driver = Adafruit_PWMServoDriver();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float distance = 0.0;

//RESTING POSITION: 80, 140, 140

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

Position test_pos (0.0, 0.0, 0.0);

//FSM VARIABLES --------------------------------------------

bool S, W, I, T, l, R, u, d;

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
  sm1_rotate,
  sm1_lateral_walk,
  sm1_lift,
  sm1_lower
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

// Robot Variables ------------------------------------------

Spider_Robot spider;

LEG legA(9, 10, 11, 3.65, 4.75, 8.2, LEG_A_THETA1_OFFSET, LEG_A_THETA2_OFFSET, LEG_A_THETA3_OFFSET, 'A');
LEG legB(3, 4, 5, 3.65, 5.3, 8.2, LEG_B_THETA1_OFFSET, LEG_B_THETA2_OFFSET, LEG_B_THETA3_OFFSET, 'B');
LEG legC(0, 1, 2, 3.35, 4.75, 8.2, LEG_C_THETA1_OFFSET, LEG_C_THETA2_OFFSET, LEG_C_THETA3_OFFSET, 'C');
LEG legD(6, 7, 8, 3.65, 5.3, 8.2, LEG_D_THETA1_OFFSET, LEG_D_THETA2_OFFSET, LEG_D_THETA3_OFFSET, 'D');

Position initLegA(-0.65, 3.75, -5.7);
Position initLegB(-0.65, 3.75, -5.2);
Position initLegC(1.85, 3.75, -5.7);
Position initLegD(1.85, 3.75, -5.7);

// Functions -------------------------------------------------

float rad2deg(float rad){
    return rad*180.0/PI;
}

float deg2rad(float deg){
    return deg*PI/180.0;
}

void setup() {
    Wire.begin();
    pwm_driver.begin();
    pwm_driver.setPWMFreq(FREQUENCY);
    lox.begin();

    Serial.begin(9600);

    pinMode(25, OUTPUT); //Debugging LED

    last_time = 0;

    legA.attachServoDriver(Adafruit_PWMServoDriver());
    legB.attachServoDriver(Adafruit_PWMServoDriver());
    legC.attachServoDriver(Adafruit_PWMServoDriver());
    legD.attachServoDriver(Adafruit_PWMServoDriver());

    spider.attachLegs(legA, legB, legC, legD);

    set_state(robot_fsm, sm1_start);

    Serial.println("SET UP...");

  }

void loop() {
    
    uint8_t b;

    VL53L0X_RangingMeasurementData_t measure;

    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    /*
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance = float(measure.RangeMilliMeter);
      Serial.println("Distance (mm): " + String(distance)); 
    } else {
      Serial.println("Sensor out of range ");
    }
    */
    
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
    } 

    now = millis();

    //Serial.print("[BEGINING] NOW = " + String(now) + ", Last time = " + String(last_time) + "\n");

    if ((now-last_time > interval)) {

      Serial.print("NEW CYCLE -------------------------------------------------------------|\n");

      //Check Serial Inputs
      Serial.println("FSM STATE: " + String(robot_fsm.state) + " | INPUTS: W = " + String(W) + ", S = " + String(S) + ", I = " + String(I) + ", T = " + T + ", H = " + H + ", L = " + L + ", N = " + N + ", R = " + R + ", l = " + l + ", u = " + u + ", d = " + d);
      
      
      spider.legA.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG A: POS = (" + String(spider.legA.getCurrentFootPosition().getX()) + ", " + String(spider.legA.getCurrentFootPosition().getY()) + ", " + String(spider.legA.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      spider.legB.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG B: POS = (" + String(spider.legB.getCurrentFootPosition().getX()) + ", " + String(spider.legB.getCurrentFootPosition().getY()) + ", " + String(spider.legB.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      spider.legC.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG C: POS = (" + String(spider.legC.getCurrentFootPosition().getX()) + ", " + String(spider.legC.getCurrentFootPosition().getY()) + ", " + String(spider.legC.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      spider.legD.getCurrentJointAngles(t1, t2, t3);
      Serial.print("LEG D: POS = (" + String(spider.legD.getCurrentFootPosition().getX()) + ", " + String(spider.legD.getCurrentFootPosition().getY()) + ", " + String(spider.legD.getCurrentFootPosition().getZ()) + ") | Angles =  " + String(t1) + ", " + String(t2) + ", " + String(t3) + ")\n");
      Serial.println("Height Index = " + String(spider.getHeigthIndex()));      

      //(1/4) UPDATE TIME IN STATES
      uint32_t cur_time = now;   // Just one call to millis()
      robot_fsm.tis = cur_time - robot_fsm.tes;


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

      //(3/4) - UPDATE STATES
      set_state(robot_fsm, robot_fsm.new_state);

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
        spider.walk(true, true, true, true, true);
        break;
      case sm1_lateral_walk:
        spider.lateral_walk(true, true, true, true, N);
        break;
      case sm1_rotate:
        spider.rotate(true, true, true, true, true);
        break;
      case sm1_lift:
        spider.lift(true, true, true, true, true);
        break;
      case sm1_lower:
        spider.lower(true, true, true, true, true);
        break;
      case sm1_test_pos:

        xp = 0.0;
        yp = 0.0;
        zp = 0.0;

        pulse_Length1 = map(90, 0.0, 180.0, 150.0, 580.0);
        pulse_Length2 = map(121.68 + LEG_A_THETA2_OFFSET, 0.0, 180.0, 150.0, 580.0);
        pulse_Length3 = map(123.71 + LEG_A_THETA3_OFFSET, 0.0, 180.0, 150.0, 580.0);
        //pulse_length4 = map(tB + LEG_B_THETA3_OFFSET, 0.0, 180.0, 150.0, 580.0);

        //pulse_Length2 = map(90, 0.0, 180.0, 150.0, 580.0);

        //Serial.print("Theta 3 - LEG A = " + String(tA) + " | Theta 3 - LEG B = " + String(tB) + " | Theta 3 - LEG C = " + String(tC)  + " | Theta 3 - LEG D = " + String(tD) + "\n");

        //pwm_driver.setPWM(9, 0, pulse_Length1);
        pwm_driver.setPWM(10, 0, pulse_Length1);
        //pwm_driver.setPWM(11, 0, pulse_Length3); 
        //pwm_driver.setPWM(5, 0, pulse_length4);

        //Serial.println("THETA 1 LEG D = 100.49 + " + String(delta_theta) + " = " + String(100.49 + delta_theta) + "\n");

        //pwm_driver.setPWM(9, 0, pulse_Length2);
    
        break;
      case sm1_testA:
        cx = spider.legA.getCurrentFootPosition().getX();
        cy = spider.legA.getCurrentFootPosition().getY();
        cz = spider.legA.getCurrentFootPosition().getZ();

        test_pos.setX(cx);
        test_pos.setY(cy);
        test_pos.setZ(cz);

        if (H) {test_pos.setX(cx + 0.025);}
        if (L) {test_pos.setX(cx - 0.025);}

        if (E) {test_pos.setY(cy + 0.025);}
        if (G) {test_pos.setX(cy - 0.025);}
        
        if (J) {test_pos.setZ(cz + 0.025);}
        if (K) {test_pos.setZ(cz - 0.025);}        

        spider.legA.moveTo(test_pos);

        break;
      case sm1_testB:
        cx = spider.legB.getCurrentFootPosition().getX();
        cy = spider.legB.getCurrentFootPosition().getY();
        cz = spider.legB.getCurrentFootPosition().getZ();

        test_pos.setX(cx);
        test_pos.setY(cy);
        test_pos.setZ(cz);

        if (H) {test_pos.setX(cx + 0.1);}
        if (L) {test_pos.setX(cx - 0.1);}

        if (E) {test_pos.setY(cy + 0.1);}
        if (G) {test_pos.setX(cy - 0.1);}
        
        if (J) {test_pos.setZ(cz + 0.1);}
        if (K) {test_pos.setZ(cz - 0.1);}        

        spider.legB.moveTo(test_pos);

        break;
      case sm1_testC:

        cx = spider.legC.getCurrentFootPosition().getX();
        cy = spider.legC.getCurrentFootPosition().getY();
        cz = spider.legC.getCurrentFootPosition().getZ();

        test_pos.setX(cx);
        test_pos.setY(cy);
        test_pos.setZ(cz);

        if (H) {test_pos.setX(cx + 0.025);}
        if (L) {test_pos.setX(cx - 0.025);}

        if (J) {test_pos.setY(cy + 0.025);}
        if (K) {test_pos.setX(cy - 0.025);}
        
        if (C) {test_pos.setZ(cz + 0.025);}
        if (D) {test_pos.setZ(cz - 0.025);}        

        spider.legC.moveTo(test_pos);

        break;
      case sm1_testD:
        cx = spider.legD.getCurrentFootPosition().getX();
        cy = spider.legD.getCurrentFootPosition().getY();
        cz = spider.legD.getCurrentFootPosition().getZ();

        test_pos.setX(cx);
        test_pos.setY(cy);
        test_pos.setZ(cz);

        if (H) {test_pos.setX(cx + 0.025);}
        if (L) {test_pos.setX(cx - 0.025);}

        if (J) {test_pos.setY(cy + 0.025);}
        if (K) {test_pos.setX(cy - 0.025);}
        
        if (C) {test_pos.setZ(cz + 0.025);}
        if (D) {test_pos.setZ(cz - 0.025);}        

        spider.legD.moveTo(test_pos);

        break;
      default:
        break;
      }

     Serial.println("End of Cycle ----------------------------------------------------------|");

      I = W = S = T = L = H = N = A = B = C = D = E = G = J = K = l = R = u = d = false;
      
      last_time = now;
      
    }
     
    //Serial.print("[END] Now = " + String(now) + ", Last_time = " + String(last_time) + ", Interval = " + String(interval) + "\n");
   

}


/*
#include <Arduino.h>
void setup() {
    pinMode(25, OUTPUT); // Set GPIO 25 as an output
    Serial.begin(115200);
    
}


void loop() {
    digitalWrite(25, HIGH); // Turn the LED on
    delay(50);             // Wait for 500 milliseconds
    Serial.println("OLAAAAA");
    digitalWrite(25, LOW);  // Turn the LED off
    delay(50);             // Wait for 500 milliseconds
}

*/
