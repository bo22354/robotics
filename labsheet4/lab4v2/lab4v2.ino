#include "Motors.h"         
#include "PID.h"            
#include "LineSensors.h"    

#include "Magnetometer.h" // Labsheet 2 
#include <Wire.h>
#include <LIS3MDL.h>

// Labsheet 3: Calibrate and use kinematics to allow 
//             your robot to turn and travel between
//             locations.
#include "Kinematics.h"  

// Encoders.h does not need modifying.
#include "Encoders.h"     // For encoder counts


#include "lcd.h"
LCD_c display(0, 1, 14, 17, 13, 30);

#define BUZZER_PIN 6

Motors_c motors;    
LineSensors_c line_sensors; 
Magnetometer_c magnetometer;

// Instance of a class to estimate the pose
// of the robot.  You will need to calibrate
// this, and potentially improve it.
// See Labsheet 3.
Kinematics_c pose; 




LIS3MDL mag;
long mag_ts;
long mag_ms = 100;

boolean motorsRunning = true;
unsigned long motors_ms = 1000;
unsigned long motors_ts;

unsigned long speed_est_ts; // timestamp for speed estimation
#define SPEED_EST_MS 10     // estimate speed every 10ms
long last_e0; //right
float speed_e0;
long last_e1; //left
float speed_e1;

unsigned long display_ts;
float smoothing_factor = 0.5;
float last_speed0 = 0.0;
float last_speed1 = 0.0;

PID_c left_pid;
PID_c right_pid;
#define PID_UPDATE_MS 50
unsigned long pid_update_ts;

float left_demand = 0.0;
float right_demand = 0.0;

long turn_ts;
long turn_ms = 1000;
bool turning = false;

bool detected = false;
float detect_threshold = 3;


long pose_ts;
long pose_ms = 20;
unsigned long test_ts;

float rotation_target = 0;
bool is_rotating = false;
float turn_gain = 0.6; // Start small, e.g., 0.2 to 0.5
float max_turn_speed = 0.5;

float target_x = 0;
float target_y = 0;
bool is_driving = false;
float drive_gain = 0.3;
float max_drive_speed = 0.5;

bool is_navigating = false;
float nav_drive_gain = 0.5; 
float nav_turn_gain = 0.5; 
float nav_threshold = 3;

struct Waypoint {
  float x;
  float y;
};
// Define your path (e.g., a triangle)
Waypoint path[] = {
  {265, -55},     // Location 1
  {220, -265},    // Location 2
  {77, -160},     // Location 6
  {278, -425},    // Location 3
  {105, -420},     // Location 4
  {-11, -357},      // Location 5
  {0, 0}
};
//
//Waypoint path[] = {
//  {-11,-357},
//  {0, -400},
//  {10, -357},
//  {0,0}
//};

Waypoint return_path_loc2[] = {
  {265, -55},   // Location 1
  {77, -160},   // Location 6
  {105, -420},  // Location 4
  {278, -425},  // Location 3
  {220, -265},  // Location 2
  {0, 0}        // Home
};
int return_step = 0;


//Assessment 3 Waypoints order:
// 5,4,3,2,1,2,6


int current_waypoint = 0;
int total_waypoints = sizeof(path) / sizeof(path[0]);
bool path_following = false;
unsigned long waypoint_wait_ts;
bool waiting = false;

unsigned long waiting_ts;
#define WAITING_MS 4000

enum RobotState {
  SEARCHING,       // Following the waypoint path
  POSITION_2_REVERSE,
  SECURE_CAPSULE,  // Briefly driving forward to seat the puck in whiskers
  RETURNING_HOME,  // Navigating back via hubs or straight lines
  DROP_OFF,        // Releasing the puck at (0,0)
  HOME_PAUSE       // Waiting before the next search
};

RobotState currentState = SEARCHING;

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );

  Wire.begin();
  mag_ts = millis();
  
  left_pid.initialise( 60.0, 0.1, 0.0 );
  right_pid.initialise( 60.0, 0.1, 0.0);
  
  setupEncoder0();
  setupEncoder1();
  
  setupEncoder0();
  setupEncoder1();
  motors.initialise();

  speed_est_ts = millis();
  pid_update_ts = millis();
  pose_ts = millis();

  left_pid.reset();
  right_pid.reset();

  pose.initialise(0, 0, 0);

  line_sensors.initialiseForADC(); // Ensure IR LEDs are ON

  
  // Wait for Button A press to start
  Serial.begin(9600);
  Serial.println(" *** READY *** ");
  delay(1000);
  
  if (!magnetometer.initialise() ) {  // no..? :(
    while(1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }
  magnetometer.mag.enableDefault();
  
  Serial.println("Press Button A to calibrate Line Sensors");
  pinMode(14, INPUT_PULLUP);
  while(digitalRead(14) == HIGH) delay(10);
  calibration();

  float temp = millis();
  speed_est_ts = temp;
  pid_update_ts = temp;
  pose_ts = temp;

  path_following = true;
  setGoTo(path[0].x, path[0].y);

  left_pid.reset();
  right_pid.reset();
}

void calc_right_speed(unsigned long elapsed_time){
    long diff_e0 = count_e0 - last_e0;
    last_e0 = count_e0;
    speed_e0 = (float)diff_e0 / (float)elapsed_time;
    last_speed0 = (smoothing_factor*speed_e0)+((1.0-smoothing_factor)*last_speed0);
}
void calc_left_speed(unsigned long elapsed_time){
  long diff_e1 = count_e1 - last_e1;
  last_e1 = count_e1;
  speed_e1 = (float)diff_e1 / (float)elapsed_time;
  last_speed1 = (smoothing_factor*speed_e1)+((1.0-smoothing_factor)*last_speed1);
}


void calibration() {
  Serial.println("Starting Line Sensor Calibration...");
  line_sensors.initialiseCalibration();
  magnetometer.initialiseCalibration();

  left_demand = 0.5;
  right_demand = -0.5;
  left_pid.reset();
  right_pid.reset();
  
  unsigned long startTime = millis();
  unsigned long calibrationDuration = 4000; // Rotate for 4 seconds
  
  while (millis() - startTime < calibrationDuration) {
    unsigned long current_ts = millis();
    updatePID(current_ts);
    line_sensors.calibrate();
    magnetometer.calibrate();
    delay(1); 
  }

  motors.setPWM(0, 0);
  line_sensors.setScaling();
  magnetometer.finalCalibrate();
  
  Serial.println("Calibration Complete.");
  delay(1000);
  pose.update();  
  Serial.println("Clear! Entering main loop.");
}

void readMag(){
  magnetometer.calcCalibrated();
  float m = sqrt( sq(magnetometer.calibrated[0]) + sq(magnetometer.calibrated[1]) + sq(magnetometer.calibrated[2]) );
//  if(m > detect_threshold){
  if(m > 3){
    detected = true;
    analogWrite( BUZZER_PIN, 1);
  }
  else{
    analogWrite( BUZZER_PIN, 0);
  }
  display.gotoXY(0, 1); // Column 0, Row 1
  display.print(m);
  mag_ts = millis();
}

float getSmallestAngleDiff(float target, float source) {
  float diff = target - source;

  // Use fmod to keep the difference within [-2*PI, 2*PI]
  diff = fmod(diff, TWO_PI);

  // This is the magic part: if the diff is more than 180 degrees, 
  // it forces the robot to take the shorter path the other way.
  if (diff > PI)  diff -= TWO_PI;
  if (diff < -PI) diff += TWO_PI;

  return diff;
}

void updatePID(unsigned long current_ts){
  if (current_ts - speed_est_ts >= SPEED_EST_MS) {
    calc_left_speed(current_ts - speed_est_ts);
    calc_right_speed(current_ts - speed_est_ts);
    speed_est_ts = current_ts;
  }
  if (current_ts - pid_update_ts >= PID_UPDATE_MS) {
    pid_update_ts = current_ts;
    
    if (left_demand == 0 && right_demand == 0) {
        motors.setPWM(0, 0); 
        left_pid.reset();  // Keep the I-term at zero
        right_pid.reset(); // Keep the I-term at zero
    } else {
        // Only run PID math if we actually want to move
        float l_pwm = left_pid.update(left_demand, last_speed1);
        float r_pwm = right_pid.update(right_demand, last_speed0);
        motors.setPWM(l_pwm, r_pwm);
    }
  }
}

void setGoTo(float x, float y) {
  target_x = x;
  target_y = y;
  is_navigating = true;
  left_pid.reset();
  right_pid.reset();
}

bool checkGoTo() {
  if (!is_navigating) return false;

  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  float distance_error = sqrt(sq(dx) + sq(dy));
  float target_theta = atan2(dy, dx);
  float steering_error = getSmallestAngleDiff(target_theta, pose.theta);

  display.gotoXY(0,0);
  display.print(steering_error);

  if (distance_error < nav_threshold) {
    left_demand = 0;
    right_demand = 0;
    left_pid.reset();
    right_pid.reset();
    is_navigating = false;
    return false;
  }

  float forward_demand = distance_error * nav_drive_gain;
  if (forward_demand > 0 && forward_demand < 0.15) forward_demand = 0.15;
  if (forward_demand < 0 && forward_demand > -0.15) forward_demand = -0.15;
  forward_demand = constrain(forward_demand, -max_drive_speed, max_drive_speed);
  float turn_component = steering_error * nav_turn_gain;

  if (abs(steering_error) > (PI / 6.0)) {
      forward_demand = 0; 
  }
  
  left_demand = forward_demand - turn_component;
  right_demand = forward_demand + turn_component;
  return true;
}


void updatePathFollowing() {
  if (!path_following) return;

  if (!waiting) {
    if (checkGoTo() == false) {
      waiting = true;
      waypoint_wait_ts = millis();
    }
  } else {
    if (millis() - waypoint_wait_ts > 200) { // Wait for physical vibrations to stop
      waiting = false;
      current_waypoint++;
      if (current_waypoint >= total_waypoints) {
          current_waypoint = 0; 
          // path_following = false; // Uncomment to stop after one lap
      }
      setGoTo(path[current_waypoint].x, path[current_waypoint].y);
    }
  }
}


void loop() {
  unsigned long current_ts = millis();
  
  // 1. SENSE (Always run)
  line_sensors.readSensorsADC();
  line_sensors.calcCalibratedADC();
  if (current_ts - pose_ts > pose_ms) {
    pose.update();
    pose_ts = current_ts;
  }
  if (current_ts - mag_ms > mag_ts) {
    readMag();
  }

  // 2. THINK (State Machine)
  switch (currentState) {

    case SEARCHING:
      updatePathFollowing(); 
      if (detected) {

        if(current_waypoint == 1){
          currentState = POSITION_2_REVERSE;
        }
        else{
          currentState = SECURE_CAPSULE;
          test_ts = millis(); // Start a small timer
        }        
      }
      break;
      
  case POSITION_2_REVERSE:
    // Calculate the distance to Location 1
    float dx = return_path_loc2[0].x - pose.x;
    float dy = return_path_loc2[0].y - pose.y;
    float dist = sqrt(sq(dx) + sq(dy));
    
    // Debugging: Print distance to LCD to see if it's decreasing
    display.gotoXY(0,0);
    display.print("Dist: ");
    display.print(dist);

    if (dist < 20.0) { // Increased threshold slightly for reliability
       // Stop motors before switching states to prevent a "jerk"
       left_demand = 0;
       right_demand = 0;
       
       return_step = 1; // Target Location 6
       setGoTo(return_path_loc2[return_step].x, return_path_loc2[return_step].y);
       currentState = RETURNING_HOME; 
    } else {
       // We manually set reverse demands. 
       // IMPORTANT: Ensure pose.update() is running in loop() (which it is)
       left_demand = -0.25;
       right_demand = -0.25;
    }
    break;
      
    case SECURE_CAPSULE:
      // Drive forward for 200ms to ensure puck is inside the whiskers 
      left_demand = 0.2;
      right_demand = 0.2;
      if (millis() - test_ts > 500) {
        // Lower the turn gain globally for the return trip to prevent dropping
        nav_turn_gain = 0.25; 
        
        // Decide return path based on where we are
        // Locations 3, 4, 5 benefit from a "Hub" approach 
//        if (current_waypoint >= 4) { 
//           setGoTo(150, -200); // Temporary "Hub" coordinate
//        } else {
//           setGoTo(0, 0);
//        }

        setGoTo(0, 0);
        currentState = RETURNING_HOME;
      }
      break;

    case RETURNING_HOME:
      // If checkGoTo returns false, we reached our current target waypoint
      if (checkGoTo() == false) {
        
        // If we are on the specialized path (return_step is 1 or more)
        if (return_step > 0 && return_step < (sizeof(return_path_loc2) / sizeof(return_path_loc2[0]) - 1)) { 
          return_step++;
          Serial.print("Next Return Step: ");
          Serial.println(return_step);
          
          setGoTo(return_path_loc2[return_step].x, return_path_loc2[return_step].y);
        } 
        // If we are not on a specialized path OR we just finished the last step
        else {
          currentState = DROP_OFF;
          test_ts = millis();
          return_step = 0; // Reset for next search
        }
      }
      
      // Safety: If magnet signal disappears, the puck was dropped!
      if (!detected) {
        currentState = SEARCHING;
        nav_turn_gain = 0.5; 
        return_step = 0;
      }
      break;

    case DROP_OFF:
      // Back up to leave the puck in the home zone
      left_demand = -0.3;
      right_demand = -0.3;
      if (millis() - test_ts > 1000) {
        left_demand = 0;
        right_demand = 0;
        detected = false;
        nav_turn_gain = 0.5; // Restore turn speed for searching
        waiting_ts = current_ts;
        currentState = HOME_PAUSE;
      }
      break;

    case HOME_PAUSE:
      if (current_ts - waiting_ts >= WAITING_MS) {
        currentState = SEARCHING;
        // Move to the next waypoint in the search sequence 
        setGoTo(path[current_waypoint].x, path[current_waypoint].y);
      }
      break;
  }

  // 3. ACT
  updatePID(current_ts);
}
