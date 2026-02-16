/************************************
,        .       .           .      , 
|        |       |           |     '| 
|    ,-: |-. ,-. |-. ,-. ,-. |-     | 
|    | | | | `-. | | |-' |-' |      | 
`--' `-` `-' `-' ' ' `-' `-' `-'    ' 
*************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions.  By using #define we can
// switch the number here, and everywhere the
// text appears (i.e. L_PWM) it will be
// replaced.
#define L_PWM 10 // This is correct.
#define L_DIR 16 // This is the wrong pin! 
#define R_PWM 9 // This is the wrong pin! 
#define R_DIR 15 // This is the wrong pin! 

// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
#define MAX_PWM 180.0

// Class to operate the motors.
class Motors_c {

  public:

    // Constructor, must exist.
    Motors_c() {
      // Leave empty. Ensure initialise() is called
      // instead.
    }

    // Use this function to initialise the pins that
    // will control the motors, and decide what first
    // value they should have.
    void initialise() {

      // Uncomment and replace the ???? with the correct
      // values, e.g  pinMode( <pin>, <INPUT or OUTPUT> );
       pinMode( 10 , OUTPUT ); //left motor
       pinMode( 16 , OUTPUT ); //left direction
       pinMode( 9 , OUTPUT ); //right motor
       pinMode( 15 , OUTPUT ); //right direction

      // Uncomment and replace ???? with the correct
      // values, e.g. digitalWrite( <pin>, <HIGH or LOW> );
      // Which pins will be either HIGH or LOW? What
      // purpose do they serve?
            digitalWrite( 16, HIGH );
            digitalWrite( 15, HIGH );

      // Uncomment and replace ???? with the correct
      // values, e.g. analogWrite( <pin>, < 0 : 255 > );
      // Which pins will take a value in range [0:255]?
      // What purpose do they serve?
       analogWrite( 10 , 0 );
       analogWrite( 9 , 0 );

//       delay(1000);
//       analogWrite( 10, 0);
//       analogWrite( 9, 0);

    } // End of initialise()


    // This function will be used to send a power value
    // to the motors.
    //
    // The power sent to the motors is created by the
    // analogWrite() function, which is producing PWM.
    // analogWrite() is intended to use a range between
    // [0:255].
    //
    // This function takes two input arguments: "left_pwr"
    // and "right_pwr", (pwr = power) and they are of the
    // type float. A float might be a value like 0.01, or
    // -150.6
    void setPWM( float left_pwr, float right_pwr ) {

      // What should happen if the request for left_pwr
      // is less than 0? Recall, how are these motors
      // operated in terms of the pins used?
      if ( left_pwr < 0 ) {
         digitalWrite( 16, HIGH );
         left_pwr = -left_pwr;
      } else {
         digitalWrite( 16, LOW );
      }

      // What should happen if the request for right_pwr
      // is less than 0? Recall, how are these motors
      // operated in terms of the pins used?
      if ( right_pwr < 0 ) {
         digitalWrite( 15, HIGH );
         right_pwr = -right_pwr;
      } else {
         digitalWrite( 15, LOW );
      }


      // analogWrite() requires a value in the range
      // [0:255], and note this is positive only!
      // Write some code here to take the value of
      // left_pwr and ensure it is:
      // - positive only
      // - within the range 0 to 255
      // Note, we have used the sign to determine
      // the direction - so we don't care about the
      // sign of the value any more.
      

      // analogWrite() requires a value in the range
      // [0:255], and note this is positive only!
      // Write some code here to take the value of
      // right_pwr and ensure it is:
      // - positive only
      // - within the range 0 to 255
      // Note, we have used the sign to determine
      // the direction - so we don't care about the
      // sign of the value any more.
      if ( left_pwr > MAX_PWM )  left_pwr = MAX_PWM;
      if ( right_pwr > MAX_PWM ) right_pwr = MAX_PWM;
      

      // Lastly, write the requested power value to
      // the motors as a PWM signal.
      // Without the code above, this is likely to
      // produce very unexpected behaviours.
      analogWrite( L_PWM, left_pwr );
      analogWrite( R_PWM, right_pwr );

      // Done!
      return;

    } // End of setPWM()


}; // End of Motors_c class definition.



#endif
