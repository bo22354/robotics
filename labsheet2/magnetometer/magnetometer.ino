// Wire.h provides functions to use the I2C protocol.
#include <Wire.h>

// LISM3MDL provides functions to read the magnetometer
#include <LIS3MDL.h>


// LIS3MDL provides a class.  We create an instance of this
// class called "mag".  We will then use "mag" to access the
// the magnetomer device by method functions and variables
// inside the class.  You should be familiar with this mode
// or interaction with a class from Labsheet 1 and 2.
LIS3MDL mag;


void setup() {

  // Start the wire library for i2c.  Note: do not add this
  // command into a class constructor. It must occur (or be
  // called) from setup().  
  Wire.begin();

  // Serial for debug output
  Serial.begin(9600);
  Serial.println("***RESET***");
  delay(1000);


  // If you have a problem with your magnetometer, your code
  // will get stuck here and print the below message.
  if (!mag.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // magnetometer, we put the robot into an infinite
    // while loop and report the error.
    while(1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }

  // Initialisation was ok.  Set magnetometer to default settings.
  mag.enableDefault();

} // end of setup()


// Short loop to report magnetometer readings over Serial Print().
void loop() {

  // Calling this function from our instance of the LIS3MDL class
  // gets the latest readings from the magnetometer sensor.
  mag.read();

  // Print the readings over serial. m.x = x axis, etc.
  Serial.print(mag.m.x);
  Serial.print(",");
  Serial.print(mag.m.y);
  Serial.print(",");
  Serial.print(mag.m.z);

  // Remember to finish comma seperated values with a newline (\n)
  Serial.print("\n");

  // Short delay to keep things slow enough to observe on the
  // Serial Plotter.
  // There is a limit to how fast you
  // can make i2c readings.
  delay(100);

} // end of loop()
