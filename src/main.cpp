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
#include "leg.h"
#include "spider_robot.h"
#include "position.h"

uint32_t now, last_time;
uint32_t last_lift;

uint32_t interval = 200;

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
int servo_test_id = 6;

Position test_pos (0.0, 0.0, 0.0);

//FSM VARIABLES --------------------------------------------

bool S, W, I, T, l, R, u, d, O, M;

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
  sm1_walk_over_obstacles,
  sm1_rotate,
  sm1_lateral_walk,
  sm1_lift,
  sm1_lower
};

enum{
  sm2_idle = 0,
  sm2_walk,
  sm2_lift
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
fsm_t walk_over_obstacles_fsm;

// Robot Variables ------------------------------------------

Spider_Robot spider;

LEG legA(0, 1, 2, 3.65, 4.75, 8.2, LEG_A_THETA1_OFFSET, LEG_A_THETA2_OFFSET, LEG_A_THETA3_OFFSET, 'A');
LEG legB(3, 4, 5, 3.65, 5.3, 8.2, LEG_B_THETA1_OFFSET, LEG_B_THETA2_OFFSET, LEG_B_THETA3_OFFSET, 'B');
LEG legC(6, 7, 8, 3.35, 4.75, 8.2, LEG_C_THETA1_OFFSET, LEG_C_THETA2_OFFSET, LEG_C_THETA3_OFFSET, 'C');
LEG legD(9, 10, 11, 3.65, 5.3, 8.2, LEG_D_THETA1_OFFSET, LEG_D_THETA2_OFFSET, LEG_D_THETA3_OFFSET, 'D');

Position initLegA(1.85, 3.75, -5.7);
Position initLegB(1.85, 3.75, -5.2);
Position initLegC(-0.65, 3.75, -5.7);
Position initLegD(-0.65, 3.75, -5.2);

Position walk_pos(120,0,0);

// FIR ------------------------------------------------------

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
  computeFIROutput(fir);
}

FIR TOF_measurements;

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

    for (int i = 0; i < 5; i++){
      TOF_measurements.entries[i] = 0.0;
    }
    computeFIROutput(TOF_measurements);

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
    } 

    now = millis();

    //Serial.print("[BEGINING] NOW = " + String(now) + ", Last time = " + String(last_time) + "\n");

    if ((now-last_time > interval)) {

      Serial.print("NEW CYCLE -------------------------------------------------------------|\n");
     
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance = float(measure.RangeMilliMeter) - 25.0;
      Serial.print("Distance (mm): " + String(distance)); 
      addEntry(TOF_measurements, distance);
      M = (TOF_measurements.output <= 150.0);
      Serial.println("| M = " + String(M));
    } else {
      Serial.println("TOF sensor out of range ");
      M = false;
    }

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
      } else if (robot_fsm.state == sm1_idle && O){
        robot_fsm.new_state = sm1_walk_over_obstacles;
        walk_over_obstacles_fsm.new_state = sm2_idle;
        spider.START_WALKING = true;
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
      } else if (robot_fsm.state == sm1_walk_over_obstacles && S){
        robot_fsm.new_state = sm1_idle;
        walk_over_obstacles_fsm.new_state = sm2_idle;
        spider.START_WALKING = false;
      } else if (robot_fsm.state == sm1_walk_over_obstacles && spider.DesiredLocationReached() && walk_over_obstacles_fsm.state == sm2_idle){
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

      //(3/4) - UPDATE STATES
      set_state(robot_fsm, robot_fsm.new_state);

      Serial.println("'ROBOT' FSM STATE: " + String(robot_fsm.state) + " | INPUTS: W = " + String(W) + ", S = " + String(S) + ", I = " + String(I) + ", T = " + T + ", H = " + H + ", L = " + L + ", N = " + N + ", R = " + R + ", l = " + l + ", u = " + u + ", d = " + d);
      Serial.println("'WALK OVER OBSTACLES' FSM STATE: " + String(walk_over_obstacles_fsm.state));
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
        spider.rotate(true, true, true, true, true);
        break;
      case sm1_lift:
        spider.lift(true, true, true, true, true);
        break;
      case sm1_lower:
        spider.lower(true, true, true, true, true);
        break;
      case sm1_walk_over_obstacles:
        if (walk_over_obstacles_fsm.state == sm2_idle){
          walk_over_obstacles_fsm.new_state = sm2_walk;
        } else if (walk_over_obstacles_fsm.state == sm2_walk && M){
          walk_over_obstacles_fsm.new_state = sm2_lift;
          spider.stop();
        } else if (walk_over_obstacles_fsm.state == sm2_walk && spider.DesiredLocationReached()){
          walk_over_obstacles_fsm.new_state == sm2_idle;
        } else if (walk_over_obstacles_fsm.state == sm2_lift && !M){
          walk_over_obstacles_fsm.new_state = sm2_walk;
          spider.lift(true, true, true, true, true);
        } 

        set_state(walk_over_obstacles_fsm, walk_over_obstacles_fsm.new_state);

        switch(walk_over_obstacles_fsm.state){
          case sm2_idle:
            walk_pos.setX(walk_pos.getX() + 10.0);
            break;
          case sm2_walk:
            spider.walkTo(true, true, true, true, true, walk_pos);
            break;
          case sm2_lift:
            now = millis();
            if (last_lift - now > 1000){
              if (spider.higherPositionAvailable()){
                spider.lift(true, true, true, true, true);
                last_lift = now;
              } else {
                Serial.println("NO HIGHER POSITION AVAILABLE! ");
              }
              Serial.print("HEIGHT INDEX = " + String(spider.getHeigthIndex() + " @" + String(last_lift)));
            } else {
              Serial.println("Waiting until next lift");
            }
            
            break;
        }

        break;
      case sm1_test_pos:

        xp = 0.0;
        yp = 0.0;
        zp = 0.0;

        if (H) {delta_theta+=0.5;}
        if (L) {delta_theta-=0.5;}
        if (N) {servo_test_id++;delta_theta=0;}

        pulse_Length1 = map(90 + delta_theta, 0.0, 180.0, 150.0, 580.0);
     
        pwm_driver.setPWM(servo_test_id, 0, pulse_Length1);

        Serial.println("Servo ID offset " + String(servo_test_id) + " = " + String(delta_theta) + "º ");
    
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

      I = W = S = T = L = H = N = A = B = C = D = E = G = J = K = l = R = u = d = O = M = false;
      
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