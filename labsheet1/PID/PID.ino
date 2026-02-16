#include "PID.h"
#include "Motors.h"       
#include "Encoders.h"  
#include "lcd.h"

LCD_c display(0, 1, 14, 17, 13, 30);
#define BUZZER_PIN 6
Motors_c motors;   
PID_c left_pid; // Let's just work with one motor to start
PID_c right_pid;

float demand = 0.5;   // Demand wheel speed.
unsigned long test_ts;
#define TEST_MS 2000

//Speed Calculating Variables
unsigned long speed_est_ts; // timestamp for speed estimation
#define SPEED_EST_MS 10     // estimate speed every 10ms
long last_e0; //right
float speed_e0 = 0.0; //current speed (before smoothing) (right)
long last_e1; //left
float speed_e1 = 0.0; //current speed (before smoothing) (left)
float last_speed0 = 0.0; //Speed for last check (after smoothing) (right)
float last_speed1 = 0.0; //Speed for last check (after smoothing) (left)
unsigned long display_ts;
float smoothing_factor = 0.2;

#define PID_UPDATE_MS 50
long pid_update_ts;

void setup() {
  Serial.begin(9600);
  
  // Initialise all gains to zero for now.
//  left_pid.initialise( -160.0, 0.0, 0.0 );
  left_pid.initialise( 135.0, 0.1, 0.0 );
  right_pid.initialise( 160.0, 0.1, 0.0);

  setupEncoder0();
  setupEncoder1();
  motors.initialise();

  speed_est_ts = millis();
  test_ts = millis();
  pid_update_ts = millis();

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

void loop() {
  long current_ts = millis(); 
  
  if( current_ts - test_ts > TEST_MS ) {
    test_ts = current_ts;
    demand = demand * -1.0;
  }

  // You will need to ensure you are calculating
  // an estimate of wheel speed.
  unsigned long elapsed_time = current_ts - speed_est_ts;
  if( elapsed_time >= SPEED_EST_MS ) {
    calc_left_speed(elapsed_time);
    calc_right_speed(elapsed_time);
    speed_est_ts = current_ts; // Update the timestamp!
  }
  
  unsigned long pid_elapsed = current_ts - pid_update_ts;
  if ( pid_elapsed >= PID_UPDATE_MS){
    pid_update_ts = current_ts;
    
    float l_pwm = left_pid.update( demand, last_speed1 );
    float r_pwm = right_pid.update( demand, last_speed0 );
    
    motors.setPWM( l_pwm, r_pwm );
//    motors.setPWM( 0,0);
    // Serial prints to help us debug and tune
    Serial.print( demand*20 );
    Serial.print( "," );
    Serial.print( last_speed0*20 );
    Serial.print(",");
    Serial.print(right_pid.i_term);

//    Serial.print( demand*20 );
    Serial.print( "," );
    Serial.print( last_speed1*20 );
//    Serial.print(",");
//    Serial.print(left_pid.i_term);
    Serial.print( "\n" );
  }
  
 }
