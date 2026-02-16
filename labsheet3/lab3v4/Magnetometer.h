/***************************************
 ,        .       .           .     ,-.  
 |        |       |           |        ) 
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /  
 |    | | | | `-. | | |-' |-' |      /   
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--' 
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings 
    // from the magnetometer
    float readings[ MAX_AXIS ];

    float minimum[ MAX_AXIS ];
    float maximum[ MAX_AXIS ];
    float range[ MAX_AXIS ];
    float offset[ MAX_AXIS ];
    float scaling[ MAX_AXIS ];
    float calibrated[ MAX_AXIS];

    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {

      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if ( !mag.init() ) {
        return false;
      } else {
        return true;
      }
    } // End of initialise()

    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()

    void initialiseCalibration(){
      for(int i = 0; i < MAX_AXIS; i++){
        minimum[i] = 9999.9;    
        maximum[i] = -9999.9; 
      }
    }
    void calibrate(){
      getReadings();

      for(int i = 0; i < MAX_AXIS; i++){
        if(readings[i] > maximum[i]){
          maximum[i] = readings[i];
        }
        if(readings[i] < minimum[i]){
          minimum[i] = readings[i];
        }
      }
    }
    void finalCalibrate(){
      for(int i = 0; i < MAX_AXIS; i++){
        range[i] = maximum[i] - minimum[i];
        if (range[i] > 0) {
          offset[i] = minimum[i] + (range[i] / 2.0);
          scaling[i] = 1.0 / (range[i] / 2.0);
        } else {
          offset[i] = 0;
          scaling[i] = 1.0; 
        }
      }
    }
    void calcCalibrated(){
      getReadings();
      for (int i = 0; i < MAX_AXIS; i++){
        calibrated[i] = (readings[i] - offset[i]) * scaling[i];
      }
    }

}; // End of Magnetometer_c class definition

#endif
