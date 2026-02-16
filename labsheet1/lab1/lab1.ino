// These #include commands essentially "copy and paste" 
// the above .h files (tabs above) into your code here.

// Labsheet 1: Build a class to operate your motors.
#include "Motors.h"         

// Labsheet 1: Tune PID to control wheel speed.
#include "PID.h"            

// Labsheet 2: Build a class to read the line sensors.
#include "LineSensors.h"    

// Labsheet 2: Operate the magnetometer to detect the 
//             puck.  You will need to follow the steps
//             to install the software library.
//#include "Magnetometer.h" // Labsheet 2 

// Labsheet 3: Calibrate and use kinematics to allow 
//             your robot to turn and travel between
//             locations.
#include "Kinematics.h"  

// Encoders.h does not need modifying.
#include "Encoders.h"     // For encoder counts


// If you want to use one of the Display modules for
// your robot, please read Supplementary Labsheet 4.
// You can only use one of the following.  Uncomment
// the correct one for your display module.  If you
// have not followed Supplementary Labsheet 4, these 
// will cause a compile error in your code.  

// Uncomment the next two lines if you will use
// the GREEN display with 2 rows of pins.
#include "lcd.h"
LCD_c display(0, 1, 14, 17, 13, 30);

// Uncomment the next two lines if you will use
// the BLUE display with 1 row of pins.
//#include "oled.h"
//OLED_c display(1, 30, 0, 17, 13);


// Used for Labsheet 0. Defines which pin the buzzer
// is attached to. #define works like a 
// find-and-replace through your code.
// See Labsheet 0.
#define BUZZER_PIN 6

// Instance of a class to operate motors.
// A class is like a template, and we name
// an "instance" to use.
// You can recognise a class type in the
// code by the convention of "_c"
// You will need to complete the class.
// See Labsheet 1.
Motors_c motors;    

// Instance of a class to operate the line
// sensors to measure the surface reflectance.
// You will need to complete the class.
// See Labsheet 2.
LineSensors_c line_sensors; 

// Instance of a class to operate the magnetometer.
// Completing the class is a later exercise in 
// Labsheet 2, so you can leave this commented
// out.
// See Labsheet 2.
//Magnetometer_c magnetometer;

// Instance of a class to estimate the pose
// of the robot.  You will need to calibrate
// this, and potentially improve it.
// See Labsheet 3.
Kinematics_c pose; 

// The setup() function runs only once when the
// robot is powered up (either by plugging in
// the USB cable, or activating the motor power.
// Use this function to do "once only" setup 
// and configuration of your robot.





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

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );
  motors.initialise();

  setupEncoder0();
  setupEncoder1();
  last_e0 = count_e0;
  speed_e0 = 0.0;
  last_e1 = count_e1;
  speed_e1 = 0.0;
  speed_est_ts = millis();

  pose.initialise(0, 0, 0);

  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** READY *** ");
    
  setMotor();
  motorsRunning = true;
  motors_ts = millis();
}

void setMotor(){
  motors.setPWM( 25, 25);
}

void checkMotor(){
  if((millis() - motors_ts) > motors_ms){
     motors.setPWM( 0, 0);
     motorsRunning = false;
  }
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
//    Serial.print( speed_e0, 4 );
//    Serial.print( ", " );
//    Serial.println( speed_e1, 4 );
//    Serial.print( ", " );
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


// 
void loop() {

  // Create an if() block here that uses the above
  // speed_est_ts and SPEED_EST_MS variables to
  // only conduct a speed estimate every 10ms
  unsigned long current_ts = millis();
  unsigned long elapsed_time = current_ts - speed_est_ts;
  
  if( elapsed_time >= SPEED_EST_MS ) {

    calc_left_speed(elapsed_time);
    calc_right_speed(elapsed_time);
    speed_est_ts = current_ts; // Update the timestamp!
    monitorSpeed();
  }
  if (millis() - display_ts > 100) {
    outputSpeed();
  }
}








// TASK 3 Non blocking move forward for set duration
//void loop() {
////  motors.setPWM( 25, 25 );
////  delay(1000);
////  motors.setPWM( 0, 0 );
//  if (motorsRunning){
//    checkMotor();
//  }
//}
