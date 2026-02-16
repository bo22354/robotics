#include "PID.h"
#include "Motors.h"        
#include "Encoders.h"

Motors_c motors;
PID_c left_pid; 
PID_c right_pid;

// Timing and State
#define FORWARD_MS 10000
unsigned long mission_start_ts;
bool is_moving = true;

// Shared variables from your previous labs
float demand = 0.5; 
unsigned long speed_est_ts;
#define SPEED_EST_MS 10
long last_e0, last_e1;
float last_speed0 = 0, last_speed1 = 0;
float smoothing_factor = 0.2;

#define PID_UPDATE_MS 50
unsigned long pid_update_ts;

void setup() {
  Serial.begin(9600);
  
  // Use your calibrated values here
  left_pid.initialise(160.0, 0.1, 0.0);
  right_pid.initialise(220.0, 0.1335, 0.0);

  setupEncoder0();
  setupEncoder1();
  motors.initialise();
  
  left_pid.reset();
  right_pid.reset();
  
  // Initialize all timestamps
  unsigned long start_time = millis();
  speed_est_ts = start_time;
  pid_update_ts = start_time;
  mission_start_ts = start_time;
  
  Serial.println("Mission Start: Forward for 2 Seconds");
}

void loop() {
  unsigned long current_ts = millis();

  // 1. Check if 2 seconds have passed
  if (is_moving && (current_ts - mission_start_ts >= FORWARD_MS)) {
    is_moving = false;
    motors.setPWM(0, 0); // Hard stop
    Serial.println("Mission Complete: Stopped.");
  }

  // 2. Only run PID and Speed math if we are still in the 'moving' state
  if (is_moving) {
    
    // --- Speed Calculation (Same as before) ---
    if (current_ts - speed_est_ts >= SPEED_EST_MS) {
      long diff0 = count_e0 - last_e0;
      long diff1 = count_e1 - last_e1;
      last_e0 = count_e0; last_e1 = count_e1;
      
      float s0 = (float)diff0 / (float)(current_ts - speed_est_ts);
      float s1 = (float)diff1 / (float)(current_ts - speed_est_ts);
      
      last_speed0 = (smoothing_factor * s0) + ((1.0 - smoothing_factor) * last_speed0);
      last_speed1 = (smoothing_factor * s1) + ((1.0 - smoothing_factor) * last_speed1);
      speed_est_ts = current_ts;
    }

    // --- PID Update (Same as before) ---
    if (current_ts - pid_update_ts >= PID_UPDATE_MS) {
      pid_update_ts = current_ts;
      
      float l_pwm = left_pid.update(demand, last_speed1);
      float r_pwm = right_pid.update(demand, last_speed0);
      
      motors.setPWM(l_pwm, r_pwm);
    }
  }
}
