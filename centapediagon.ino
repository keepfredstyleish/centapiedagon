/* B3_10_HoloSquare.ino
   This is a test sketch for autonomous driving mode of
   the three-wheeled drive Bread Board Bot (BBbot, B^3)

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; December 2, 2014
   Revisions: 2015, May 25:   Changed for new motor configuration. DLE
              2015, June 12:  Changed into B3_ code style for GoSciTech 2015. DLE
              2015, July 9: Name change, cleaned up and additional comments added. DLE
              2015, July 10: Change default BACK motor port
              2019, July 5: Added the bump sensors to start each run. DLE
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <BreadBoardBot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants
const float cos30sin60 = sqrt(3.0) / 2.0; // cos(30 deg) = sin(60 deg), need for wheel
// vector calcs.

// IO Pins used
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
const byte BACK_MOTOR_PORT = 2;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);
Adafruit_DCMotor *motorBack = AFMS.getMotor(BACK_MOTOR_PORT);

// Define global variables
float direction;       // Velocity Vector Angle (DEGREES) from forward to drive
float magnitude;       // Magnitude (0-100) of movement vector in given direction
float duration;        // Duration to drive at given velocity vector

byte motorLeftdir;     // Clockwise or Counter clockwise for the 3 wheels
byte motorBackdir;
byte motorRightdir;

void setup(void) {
  Serial.begin(9600);  //Begin serial communcation
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors
  motorLeft->run(RELEASE);
  motorBack->run(RELEASE);
  motorRight->run(RELEASE);
  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);
  // Now wait for the RIGHT bumpsensor to be pressed
  while (digitalRead(RIGHT_BUMP_PIN)) {};
  while (!digitalRead(RIGHT_BUMP_PIN)) {};
  delay(600); // Bump pin triggered and released, just give 0.6 seconds to get hands out of the way.
}

void loop(void) {
  // Section for taking commands from Serial Input
  // N.B.  Need to comment out one bracket at end for the autonomous loop below
  /* if (Serial.available() > 0) { */
  /*   direction = Serial.parseInt(); */
  /*   magnitude = Serial.parseInt(); */
  /*   duration = Serial.parseInt(); */
  /* }  */

  /* Autonomous loop for driving in a square */
  for ( byte leg = 1; leg < 45; leg++ ) {   // 5 times through loop for a square, why?
    duration = 2;                          // Constants per leg: Two seconds/leg
    magnitude = 50;                        //                    50% max power
    bool brake = false;                    //                    No braking
    switch (leg) {
      case 0: // Move forward
        direction = 0.;
        break;
      case 1: // Move forward
        direction = 180.;
        break; 
         case 2: // Move forward 
        direction = -45.;
        break;
         case 3: // Move forward
        direction = 0.;
        break;
         case 4: // Move forward
        direction = 180.;
        break;
         case 5: // Move forward
        direction = -45.;
        break;
         case 6: // Move forward
        direction = 0.;
        break;
         case 7: // Move forward
        direction = 180.;
        break;
         case 8: // Move forward
        direction = -45.;
        break;
         case 9: // Move forward
        direction = 0.;
        break;
         case 10: // Move forward
        direction = 180.;
        break;
         case 11: // Move forward
        direction = -45.;
        break;
         case 12: // Move forward
        direction = 0.;
        break;
         case 13: // Move forward
        direction = 180.;
        break;
         case 14: // Move forward
        direction = -45.;
        break;
         case 15: // Move forward
        direction = 0.;
        break;
         case 16: // Move forward
        direction = 180.;
        break;
         case 17: // Move forward
        direction = -45.;
        break;
         case 18: // Move forward
        direction = 0.;
        break;
         case 19: // Move forward
        direction = 180.;
        break;
         case 20: // Move forward
        direction = 180.;
        break;
         case 21: // Move forward
        direction = -90.;
        break;
        case 22: // Move forward
        direction = 180.;
        break;
        case 23: // Move forward
        direction = 0.;
        break;
        case 24: // Move forward
        direction = 45.;
          case 25: // Move forward
        direction = 180.;
        break;
        case 26: // Move forward
        direction = 0.;
        break;
        case 27: // Move forward
        direction = 45.;
          case 28: // Move forward
        direction = 180.;
        break;
        case 29: // Move forward
        direction = 0.;
        break;
        case 30: // Move forward
        direction = 45.;
          case 31: // Move forward
        direction = 180.;
        break;
        case 32: // Move forward
        direction = 0.;
        break;
        case 33: // Move forward
        direction = 45.;
          case 34: // Move forward
        direction = 180.;
        break;
        case 35: // Move forward
        direction = 0.;
        break;
        case 36: // Move forward
        direction = 45.;
          case 37: // Move forward
        direction = 180.;
        break;
        case 38: // Move forward
        direction = 0.;
        break;
        case 39: // Move forward
        direction = 45.;
          case 40: // Move forward
        direction = 180.;
        break;
        case 41: // Move forward
        direction = 0.;
        break;
        case 42: // Move forward
        direction = 45.;
        break;
           case 43 : // Move forward
        direction = 0.;
        break;
      default: // Stop and pause for 4 seconds at starting point
        magnitude = 0;
        duration = 4;
        direction = 0;
        brake = 1; // hard stop
    }

    if ( duration > 0 ) {
      Serial.print("direction = ");
      Serial.print(direction);
      Serial.print(", magnitude = ");
      Serial.print(magnitude);
      Serial.print(" and duration = ");
      Serial.println(duration);

      float xVector = magnitude * sin((M_PI * direction) / 180.);
      float yVector = magnitude * cos((M_PI * direction) / 180.);
      Serial.print("xVector, yVector = ");
      Serial.print(xVector);
      Serial.print(", ");
      Serial.println(yVector);

      // Find relative power needed for each wheel based on the target velocity vector
      float backPower = xVector;  // Multiply by fudge factor to prevent rotation if needed
      float leftPower = 0.5 * xVector - cos30sin60 * yVector;
      float rightPower = 0.5 * xVector + cos30sin60 * yVector;

      // Find the actual motor speeds, 0-255, needed.  N.B. still need motor direction!
      byte backSpeed  = map(abs(backPower),  0, 100, 0, 255);
      byte leftSpeed  = map(abs(leftPower),  0, 100, 0, 255);
      byte rightSpeed = map(abs(rightPower), 0, 100, 0, 255);

      // Set the speeds
      motorBack-> setSpeed(backSpeed);
      motorLeft-> setSpeed(leftSpeed);
      motorRight->setSpeed(rightSpeed);

      /* Set Motor directions.  For Adafruit V2 Motorshield:
           1 is Clockwise (Positive motor direction, FORWARD)
           2 is Counterclockwise (Negative vector direction, BACKWARD)
           3 is Brake (Doesn't work at present)
           4 is Release = stop power, not driving, but not brake

         We can use a trinary operator to set direction within run call
      */
      motorBack-> run((backPower  > 0) ? FORWARD : BACKWARD );
      motorLeft-> run((leftPower  > 0) ? BACKWARD : FORWARD );
      motorRight->run((rightPower > 0) ? FORWARD : BACKWARD );

      //Print out motor control details
      Serial.print("Speeds Back,Left,Right = ");
      Serial.print(copysign(backPower, backSpeed));
      Serial.print(", ");
      Serial.print(copysign(leftPower, leftSpeed));
      Serial.print(", ");
      Serial.println(copysign(rightPower, rightSpeed));

      // Run motors for the duration needed, converting from seconds to milliseconds
      delay(1000 * duration);
      if (brake) {            // Not a real brake, but set power = 0, stop driving motors
        motorBack->setSpeed(0);
        motorLeft->setSpeed(0);
        motorRight->setSpeed(0);
        motorBack-> run(RELEASE);
        motorLeft-> run(RELEASE);
        motorRight->run(RELEASE);
      }
    }
    else {                    // no duration entered, so stop all motors
      motorBack->setSpeed(0);
      motorLeft->setSpeed(0);
      motorRight->setSpeed(0);
    }
  }
  // Loop complete, so stop until LEFT bumper triggered and released, then rerun
  while (digitalRead(LEFT_BUMP_PIN)) {}; // Wait until pushed
  while (!digitalRead(LEFT_BUMP_PIN)) {}; // and released
  delay (600);                           // and 0.6 seconds to get out of the way
}
