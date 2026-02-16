//#include "Motors.h"         
//#include "PID.h"            
//#include "LineSensors.h"    
//#include "Magnetometer.h" // Labsheet 2 
//#include "Kinematics.h"  
//#include "Encoders.h"     // For encoder counts
#define BUZZER_PIN 6
//Motors_c motors;    
//LineSensors_c line_sensors; 
//Magnetometer_c magnetometer;
//Kinematics_c pose; 


boolean ledOn = false;
boolean buzzerOn = false;
unsigned long beep_ts;
unsigned long buzzer_ts; // _ts, for "timestamp"
unsigned long buzzer_ms;
unsigned long led_ts;
unsigned long led_ms;

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );
  pinMode(LED_BUILTIN, OUTPUT);

  buzzer_ms = 2000; // every 500ms, or 2hz
  led_ms = 1000; // every 500ms, or 2hz
  
  buzzer_ts = millis();
  led_ts = buzzer_ts;

}
void setBeep(){
  analogWrite( BUZZER_PIN, 1 ); // on
  buzzer_ts = millis();
  buzzerOn = true;
}
void checkBeep(){
  if((millis() - buzzer_ts) > 250){
    analogWrite( BUZZER_PIN, 0);
    buzzer_ts = millis();  
    buzzerOn = false;
  }
}

void loop() {
  unsigned long elapsed_time = millis() - led_ts;
  if(elapsed_time > led_ms){
    if(ledOn == true){
      digitalWrite(LED_BUILTIN, HIGH); 
      ledOn = false;
    }
    else{
      digitalWrite(LED_BUILTIN, LOW); 
      ledOn = true;
    }
    led_ts = millis();
  }

  elapsed_time = millis() - buzzer_ts;
  if(elapsed_time > buzzer_ms && !buzzerOn){
    setBeep();
  }
  if(buzzerOn){
    checkBeep();
  }
}
