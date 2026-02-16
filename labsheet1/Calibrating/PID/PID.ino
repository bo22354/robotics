#include "PID.h"
#include "Motors.h"        
#include "Encoders.h"   
#include "lcd.h"

LCD_c display(0, 1, 14, 17, 13, 30);
Motors_c motors;    
PID_c left_pid; 
PID_c right_pid;

// Control Variables
float demand = 0.5; 
unsigned long speed_est_ts; 
#define SPEED_EST_MS 10  
long last_e0, last_e1; 
float last_speed0 = 0, last_speed1 = 0;
float smoothing_factor = 0.15;

#define PID_UPDATE_MS 50
long pid_update_ts;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  
  pinMode(14, INPUT_PULLUP); // Button A
  Serial.println("Place robot on blocks. Press Button A to start Dual Calibration...");
  
  while(digitalRead(14) == HIGH) delay(10); // Wait for press

  // Run Calibration for Left then Right
  float left_kp = calibrate_P(1);  // 1 = Left
  float right_kp = calibrate_P(0); // 0 = Right
  
  // Apply found gains
  left_pid.initialise(left_kp, 0.1, 0.0);
  right_pid.initialise(right_kp, 0.1, 0.0);
  
  left_pid.reset();
  right_pid.reset();
  
  speed_est_ts = millis();
  pid_update_ts = millis();
  Serial.println("Calibration Done! Starting PI Control...");
  Serial.println("--- FINAL CALIBRATION RESULTS ---");
  Serial.print("Left Kp: "); Serial.println(left_kp);
  Serial.print("Right Kp: "); Serial.println(right_kp);
}

// Function to find the best Kp for a specific motor
float calibrate_P(int side) {
  float found_kp = 50.0; // Default fallback
  float target = 0.5;
  
  for (float kp = 20; kp <= 250; kp += 15) {
    motors.setPWM(0, 0);
    delay(200);
    unsigned long start = millis();
    
    while(millis() - start < 1000) {
      updateSystem();
      float pwr = (target - (side ? last_speed1 : last_speed0)) * kp;
      if (side == 1) motors.setPWM(pwr, 0); else motors.setPWM(0, pwr);
    }

    float result = side ? last_speed1 : last_speed0;
//    Serial.print(side ? "Left " : "Right ");
//    Serial.print("Kp: "); Serial.print(kp);
//    Serial.print(" Speed: "); Serial.println(result);

    if (result >= (target * 0.8)) { // 80% threshold
      found_kp = kp;
      break;
    }
  }
  return found_kp;
}

// Helper to keep math running during calibration loops
void updateSystem() {
  unsigned long cur = millis();
  if(cur - speed_est_ts >= SPEED_EST_MS) {
    long diff0 = count_e0 - last_e0;
    long diff1 = count_e1 - last_e1;
    last_e0 = count_e0; last_e1 = count_e1;
    
    float s0 = (float)diff0 / (float)(cur - speed_est_ts);
    float s1 = (float)diff1 / (float)(cur - speed_est_ts);
    
    last_speed0 = (smoothing_factor * s0) + ((1.0 - smoothing_factor) * last_speed0);
    last_speed1 = (smoothing_factor * s1) + ((1.0 - smoothing_factor) * last_speed1);
    speed_est_ts = cur;
  }
}

void loop() {
  updateSystem();
  unsigned long cur = millis();

  if (cur - pid_update_ts >= PID_UPDATE_MS) {
    pid_update_ts = cur;
    
    float l_pwm = left_pid.update(demand, last_speed1);
    float r_pwm = right_pid.update(demand, last_speed0);
    
    motors.setPWM(l_pwm, r_pwm);
    
    // Plotter output
    Serial.print(demand * 10); Serial.print(",");
    Serial.print(last_speed1 * 10); Serial.print(",");
    Serial.println(last_speed0 * 10);
  }
}
