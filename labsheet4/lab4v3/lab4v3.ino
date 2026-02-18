 #include "Motors.h"         
#include "PID.h"            
#include "LineSensors.h"    
#include "Magnetometer.h" 
#include <Wire.h>
#include <LIS3MDL.h>
#include "Kinematics.h"  
#include "Encoders.h"     
#include "lcd.h"
LCD_c display(0, 1, 14, 17, 13, 30);
#define BUZZER_PIN 6
Motors_c motors;    
LineSensors_c line_sensors; 
Magnetometer_c magnetometer;
Kinematics_c pose; 

LIS3MDL mag;
long mag_ts;
long mag_ms = 100;

unsigned long speed_est_ts; 
#define SPEED_EST_MS 10     
float speed_e0;
long last_e1; 
float speed_e1;

unsigned long display_ts;
float smoothing_factor = 0.5;
float last_speed0 = 0.0;
float last_speed1 = 0.0;

#define PID_UPDATE_MS 50
PID_c left_pid;
PID_c right_pid;
unsigned long pid_update_ts;
float left_demand = 0.0;
float right_demand = 0.0;

bool detected = false;
float detect_threshold = 3;
long pose_ts;
long pose_ms = 20;

unsigned long test_ts;

float rotation_target = 0;
bool is_rotating = false;
float turn_gain = 0.6; 
float max_turn_speed = 0.5;

float target_x = 0;
float target_y = 0;
float max_drive_speed = 0.5;

bool is_navigating = false;
float nav_drive_gain = 0.5; 
float nav_turn_gain = 0.3; 
float nav_threshold = 10;

struct Waypoint {
  float x;
  float y;
};
Waypoint path[] = {
  {265, -55},     // Location 1
  {223, -270},    // Location 2
  {77, -155},     // Location 6
  {247, -438},    // Location 3
  {110, -425},    // Location 4
  {-11, -357},    // Location 5
  {0, 0}          // Origin
};

int current_waypoint = 0;
int total_waypoints = sizeof(path) / sizeof(path[0]);
unsigned long waypoint_wait_ts;
Waypoint orbit_points[3]; 

bool path_following = false;


#define WAITING_MS 4000
unsigned long waiting_ts;
bool waiting = false;

enum RobotState {
  SEARCHING,       
  REPOSITIONING,
  SECURE_CAPSULE,  
  RETURNING_HOME,  
  DROP_OFF,        
  HOME_PAUSE,       
  RESETTING,
  ZERO_ZERO,
  COMPLETE
};
RobotState currentState = RESETTING;

int reset_stage = 0;
int reposition_stage = 0;
float circle_radius = 300;

unsigned long start_time;
unsigned long last_screen_update;
const unsigned long TIME_LIMIT_MS = 240000;

void setup() {
  pinMode( BUZZER_PIN, OUTPUT ); 
  Wire.begin();
  mag_ts = millis();

  setupEncoder0();
  setupEncoder1();

  left_pid.initialise( 50.0, 0.2, 0.0 );
  right_pid.initialise( 50.0, 0.2, 0.0);
  motors.initialise();
  pose.initialise(0, 0, 0);
  line_sensors.initialiseForADC();

  speed_est_ts = millis();
  pid_update_ts = millis();
  pose_ts = millis();
  left_pid.reset();
  right_pid.reset();

  // Ensure IR LEDs are ON
  // Wait for Button A press to start
  Serial.begin(9600);
  Serial.println(" *** READY *** ");
  delay(1000);
  if (!magnetometer.initialise() ) {
    while(1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }
  magnetometer.mag.enableDefault();
  pinMode(14, INPUT_PULLUP);
  while(digitalRead(14) == HIGH) delay(10); //Wait here until A is pressed

  calibration();

  float temp_ts = millis();
  speed_est_ts = temp_ts;
  pid_update_ts = temp_ts;
  pose_ts = temp_ts;
  path_following = true;

  setGoTo(path[0].x, path[0].y);
  left_pid.reset();
  right_pid.reset();
}

void calibration() {
  start_time = millis();
  last_screen_update = start_time;
  left_demand = 0.5;
  right_demand = -0.5;
  left_pid.reset();
  right_pid.reset();
  unsigned long calibrationDuration = 4000; 
  unsigned long current_ts;
  
  line_sensors.initialiseCalibration();
  magnetometer.initialiseCalibration();
  
  unsigned long startTime = millis();
  while (millis() - startTime < calibrationDuration) {
    current_ts = millis();
    if (current_ts - last_screen_update > 1000) {
      updateTime(current_ts);
    }
    updatePID(current_ts);
    line_sensors.calibrate();
    magnetometer.calibrate();
    delay(1); 
  }

  motors.setPWM(0, 0);
  line_sensors.setScaling();
  magnetometer.finalCalibrate();
  pose.update();  
}

void updateTime(unsigned long current_ts){
  last_screen_update = current_ts;
  unsigned long elapsed_ms = current_ts - start_time;
  int minutes = (elapsed_ms / 60000); 
  int seconds = (elapsed_ms % 60000) / 1000; 

  if (minutes > 4) { minutes = 4; seconds = 0; }

  display.gotoXY(0, 0); 
  if (minutes < 10) display.print("0");
  display.print(minutes);
  display.print(":");

  if (seconds < 10) display.print("0");
  display.print(seconds);
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

void readMag(){
  magnetometer.calcCalibrated();
  float m = sqrt( sq(magnetometer.calibrated[0]) + sq(magnetometer.calibrated[1]) + sq(magnetometer.calibrated[2]) );
  if(m > detect_threshold){
    detected = true;
    analogWrite( BUZZER_PIN, 1);
  }
  else{
    analogWrite( BUZZER_PIN, 0);
  }
  mag_ts = millis();
}


float getSmallestAngleDiff(float target, float source) {
  float diff = target - source;
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
        left_pid.reset();  
        right_pid.reset(); 
    } else {
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
    if (millis() - waypoint_wait_ts > 200) { 
      waiting = false;
      current_waypoint++;
      if (current_waypoint >= total_waypoints) {
          waiting_ts = millis();
          currentState = HOME_PAUSE;
          current_waypoint = 0; 
      }
      setGoTo(path[current_waypoint].x, path[current_waypoint].y);
    }
  }
}


void startReposition(float puck_x, float puck_y) {
  float angle_to_puck = atan2(puck_y, puck_x);

  target_x = puck_x + (circle_radius * cos(angle_to_puck));
  target_y = puck_y + (circle_radius * sin(angle_to_puck));
  
  orbit_points[0].x = puck_x + (circle_radius * cos(angle_to_puck + PI/3.0));
  orbit_points[0].y = puck_y + (circle_radius * sin(angle_to_puck + PI/3.0));

  orbit_points[1].x = puck_x + (circle_radius * cos(angle_to_puck + PI/6.0));
  orbit_points[1].y = puck_y + (circle_radius * sin(angle_to_puck + PI/6.0));

  orbit_points[2].x = puck_x + (circle_radius * cos(angle_to_puck));
  orbit_points[2].y = puck_y + (circle_radius * sin(angle_to_puck));
  
  reposition_stage = 0; 
  currentState = REPOSITIONING;
  test_ts = millis();
}

void setRotate(float target_rad) {
  rotation_target = target_rad;
  is_rotating = true;
  left_pid.reset();
  right_pid.reset();
}
bool checkRotate() {
  if (!is_rotating) return false;
  float heading_error = getSmallestAngleDiff(rotation_target, pose.theta);

  float turn_demand = heading_error * turn_gain;
  turn_demand = constrain(turn_demand, -max_turn_speed, max_turn_speed);

  left_demand = -turn_demand;
  right_demand = turn_demand;

  if (abs(heading_error) < 0.01) { 
    left_demand = 0;
    right_demand = 0;
    left_pid.reset();
    right_pid.reset();
    is_rotating = false;
    return false;
  }
  return true; 
}


void loop() {
  unsigned long current_ts = millis();
  line_sensors.readSensorsADC();
  line_sensors.calcCalibratedADC();

  if (current_ts - last_screen_update > 1000) {
      updateTime(current_ts);
      if (current_ts - start_time >= TIME_LIMIT_MS) {
        currentState = COMPLETE;
      }
  }
    
  if (current_ts - pose_ts > pose_ms) {
    pose.update();
    while (pose.theta > PI)  pose.theta -= TWO_PI;
    while (pose.theta < -PI) pose.theta += TWO_PI;
    pose_ts = current_ts;
  }

  if (current_ts - mag_ms > mag_ts) {
    readMag();
  }

  // State Machine
  switch (currentState) {
    case SEARCHING:
      updatePathFollowing(); 
      if (detected) {
        left_demand = -0.3;
        right_demand = -0.3;
        if (current_waypoint == 0 || current_waypoint == 1 || current_waypoint == 3 || current_waypoint == 4) {
            startReposition(path[current_waypoint].x, path[current_waypoint].y);
            currentState = REPOSITIONING;
            test_ts = millis();
        } else {
          currentState = SECURE_CAPSULE;
          test_ts = millis(); 
        }
      }
      break;

    case REPOSITIONING:
      if (reposition_stage == 0) {
        left_demand = -0.3;
        right_demand = -0.3;
        if (millis() - test_ts > 3000) {
          setGoTo(orbit_points[0].x, orbit_points[0].y);
          reposition_stage = 1;
        }
      } 
      else if (reposition_stage == 1) {
        if (checkGoTo() == false) {
          setGoTo(orbit_points[1].x, orbit_points[1].y);
          reposition_stage = 2;
        }
      }
      else if (reposition_stage == 2) {
        if (checkGoTo() == false) {
          setGoTo(orbit_points[2].x, orbit_points[2].y);
          reposition_stage = 3;
        }
      }
      else if (reposition_stage == 3) {
        if (checkGoTo() == false) {
          currentState = RETURNING_HOME;
          float dist = sqrt(sq(pose.x) + sq(pose.y));
          if (dist > 55.0) {
             float ratio = 75 / dist;
             float stop_x = pose.x * ratio;
             float stop_y = pose.y * ratio;
             setGoTo(stop_x, stop_y);
          } else {
             setGoTo(pose.x, pose.y); 
          }
          nav_turn_gain = 0.2;
        }
      }
      break;

    case SECURE_CAPSULE:
      left_demand = 0.2;
      right_demand = 0.2;

      if (millis() - test_ts > 500) {
        nav_turn_gain = 0.2; 
        float dist = sqrt(sq(pose.x) + sq(pose.y));

        if (dist > 55.0) {
           float ratio = 75 / dist;
           float stop_x = pose.x * ratio;
           float stop_y = pose.y * ratio;
           setGoTo(stop_x, stop_y);
        } else {
           setGoTo(pose.x, pose.y); 
        }
        currentState = RETURNING_HOME;
      }
      break;

    case RETURNING_HOME:
      if(checkGoTo() == false){
        reposition_stage = 0;
        currentState = DROP_OFF;
        test_ts = millis();
      }
      break;

    case DROP_OFF:
      if (reposition_stage == 0) {
        left_demand = 0;
        right_demand = 0;
        
        if (millis() - test_ts > 500) {
          reposition_stage = 1;
          test_ts = millis();
        }
      } 
      else if (reposition_stage == 1) {
        left_demand = -0.3;
        right_demand = -0.3;
        
        if (millis() - test_ts > 1200) { 
          left_demand = 0;
          right_demand = 0;
          reposition_stage = 2;
          test_ts = millis();
        }
      }
      else if (reposition_stage == 2) {        
        detected = false;
        nav_turn_gain = 0.3; 
        waiting_ts = millis();
        currentState = HOME_PAUSE;
      }
      break;

    case HOME_PAUSE:
      if (current_ts - waiting_ts >= WAITING_MS) {
        currentState = RESETTING;
        reset_stage = 0;
        test_ts = millis();
      }
      break;

    case RESETTING:
      if (reset_stage == 0) {
        setRotate(1.57); 
        reset_stage = 1;
      } 
      else if (reset_stage == 1) {
        if (checkRotate() == false) { 
          float target_h = 1.57;
          float steering_error = getSmallestAngleDiff(target_h, pose.theta);
          float turn_correction = steering_error * nav_turn_gain;
          
          left_demand  = 0.2 - turn_correction;
          right_demand = 0.2 + turn_correction;
          
          if (line_sensors.allOnLine()) { 
            left_demand = 0;
            right_demand = 0;
            reset_stage = 2;
          }
          if(pose.y > 100){
            setGoTo(50,50);
            reset_stage == 5;
          }
        }
      }
      else if (reset_stage == 2) {
        setRotate(3.14);
        reset_stage = 3;
      }
      else if (reset_stage == 3) {
        if (checkRotate() == false) {
          float target_h = 3.14;
          float steering_error = getSmallestAngleDiff(target_h, pose.theta);
          float turn_correction = steering_error * nav_turn_gain;
          
          left_demand  = 0.2 - turn_correction;
          right_demand = 0.2 + turn_correction;
          
          if (line_sensors.allOnLine() && pose.x <=0) {
            left_demand = 0;
            right_demand = 0;
            setRotate(0.0);
            reset_stage = 4;
          }
          else if(pose.x < -100){
            setGoTo(50,50);
            reset_stage == 5;
          }
        }
      }
      else if (reset_stage == 4) {
        if (checkRotate() == false) {
          pose.initialise(-24, 45, 0.0);
          left_pid.reset();
          right_pid.reset();
  
          detected = false;
          currentState = SEARCHING;
          current_waypoint = 0;
          setGoTo(path[current_waypoint].x, path[current_waypoint].y);
        }
      }
      break;
    case COMPLETE:
      left_demand = 0;
      right_demand = 0;
      left_pid.reset();
      right_pid.reset();
      motors.setPWM(0, 0); 
      display.gotoXY(0, 1);
      display.print("TIME UP!");
        break;
  }
  updatePID(current_ts);
} 