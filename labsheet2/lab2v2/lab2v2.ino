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

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );

  Wire.begin();
  mag_ts = millis();
  
  left_pid.initialise( 60.0, 0.5, 0.0 );
  right_pid.initialise( 60.0, 0.5, 0.0);
  
  setupEncoder0();
  setupEncoder1();
  
  setupEncoder0();
  setupEncoder1();
  motors.initialise();

  speed_est_ts = millis();
  pid_update_ts = millis();

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
  runLineSensorCalibration();
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
void monitorSpeed(){
    Serial.print( speed_e0, 4 );
    Serial.print( ", " );
    Serial.println( speed_e1, 4 );
    Serial.print( ", " );
    Serial.print( last_speed0, 4 );
    Serial.print( ", " );
    Serial.println( last_speed1, 4 );
}
void outputSpeed(){
    display_ts = millis();
    display.clear(); // Wipe the screen for fresh text
    // Row 0: Left Motor Speed
    display.gotoXY(0, 0); // Column 0, Row 0
    display.print("R: ");
    display.print(last_speed0, 4); // 2 decimal places
    // Row 1: Right Motor Speed
    display.gotoXY(0, 1); // Column 0, Row 1
    display.print("L: ");
    display.print(last_speed1, 4);
}



void runLineSensorCalibration() {
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

    if (current_ts - speed_est_ts >= SPEED_EST_MS) {
      calc_left_speed(current_ts - speed_est_ts);
      calc_right_speed(current_ts - speed_est_ts);
      speed_est_ts = current_ts;
    }
    if (current_ts - pid_update_ts >= PID_UPDATE_MS) {
      pid_update_ts = current_ts;
      float l_pwm = left_pid.update(left_demand, last_speed1);
      float r_pwm = right_pid.update(right_demand, last_speed0);
      motors.setPWM(l_pwm, r_pwm);
    }

    line_sensors.calibrate();
    magnetometer.calibrate();
    delay(1); 
  }

  motors.setPWM(0, 0);
  line_sensors.setScaling();
  magnetometer.finalCalibrate();
  
  Serial.println("Calibration Complete.");
  delay(1000);
  
  Serial.println("Moving off the calibration area...");
  left_pid.reset();  // Clear any old errors from the spin
  right_pid.reset();
  left_demand = 0.5; // Set the speed for loop() to follow
  right_demand = 0.5;
  turning = false;

  unsigned long driveOffStart = millis();
  unsigned long driveOffDuration = 1500; // Adjust this time if it's still hitting the line

  while (millis() - driveOffStart < driveOffDuration) {
    unsigned long current_ts = millis();

    // We MUST update speed and PID here, otherwise the robot won't move
    if (current_ts - speed_est_ts >= SPEED_EST_MS) {
      calc_left_speed(current_ts - speed_est_ts);
      calc_right_speed(current_ts - speed_est_ts);
      speed_est_ts = current_ts;
    }

    if (current_ts - pid_update_ts >= PID_UPDATE_MS) {
      pid_update_ts = current_ts;
      float l_pwm = left_pid.update(left_demand, last_speed1);
      float r_pwm = right_pid.update(right_demand, last_speed0);
      motors.setPWM(l_pwm, r_pwm);
    }
  }
  
  Serial.println("Clear! Entering main loop.");
}

void outputSensor(){
//  display.clear();
  display.gotoXY(0, 0); // Column 0, Row 1
  display.print(line_sensors.calibrated[0]);
}


void setTurn() {
  turn_ts = millis();
  // To spin on the spot with PID:
  left_demand = 0.3;   // Positive
  right_demand = -0.3; // Negative
  turning = true;
  left_pid.reset();    // Reset to prevent I-term carryover
  right_pid.reset();

  while(detected){
    analogWrite( BUZZER_PIN, 0);
    motors.setPWM(0,0);
  }
}

bool checkTurn() {
  if (millis() - turn_ts > turn_ms) {
    turning = false;
    return false;
  }
  return true;
}

void readMag(){
  magnetometer.calcCalibrated();

  // Print the readings over serial. m.x = x axis, etc.
  float m = sqrt( sq(magnetometer.calibrated[0]) + sq(magnetometer.calibrated[1]) + sq(magnetometer.calibrated[2]) );
  
//  for (int i = 0; i < MAX_AXIS; i++){
//    Serial.print(magnetometer.calibrated[i]);
//    if(i != 2) Serial.print(",");
//  }
//  Serial.print(m);
//  Serial.print("\n");


  if(m > detect_threshold){
    detected = true;
//    motors.setPWM(0,0);
    analogWrite( BUZZER_PIN, 1);
  }
  else{
    analogWrite( BUZZER_PIN, 0);
  }

  display.gotoXY(0, 1); // Column 0, Row 1
  display.print(m);
  mag_ts = millis();
}



void loop() {
  unsigned long current_ts = millis();
    // --- 1. SENSE ---
    line_sensors.readSensorsADC();
    line_sensors.calcCalibratedADC();
  
    // Update speed estimation
    if (current_ts - speed_est_ts >= SPEED_EST_MS) {
      calc_left_speed(current_ts - speed_est_ts);
      calc_right_speed(current_ts - speed_est_ts);
      speed_est_ts = current_ts;
    }
  
    // --- 2. THINK (State Machine) ---
    if (line_sensors.isOnLine() && !turning) {
      setTurn();
    }
  
    if (checkTurn()) {
      // We are currently turning; setTurn() already set the demands.
    } else {
      // Not turning; drive forward.
      left_demand = 0.5;
      right_demand = 0.5;
    }
  
    // --- 3. ACT (PID Update) ---
    if (current_ts - pid_update_ts >= PID_UPDATE_MS) {
      pid_update_ts = current_ts;
  
      float l_pwm = left_pid.update(left_demand, last_speed1);
      float r_pwm = right_pid.update(right_demand, last_speed0);
  
      motors.setPWM(l_pwm, r_pwm);
    }

  outputSensor();
  if(current_ts - mag_ms > mag_ts){
    readMag();
  }
  
}






//Exercise 2:
//void loop() {
//   // Get latest readings.
//   line_sensors.calcCalibratedADC();
//
//   // Access the updated readings variable within the
//   // line_sensors class instance.
//// Make sure the indices [0] through [4] are all different!
//    for(int i = 0; i < 5;i++){
//      Serial.print( line_sensors.calibrated[i] );
//      if(i != 4) Serial.print( "," );
//    }
//   Serial.println(); // Use println() to end the line properly
//   outputSensor();
//   delay(20); 
//}
