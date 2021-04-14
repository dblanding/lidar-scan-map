/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * Sends distance (cm) in response to any request string
 * Typical time to run = 0.132 seconds
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
Encoder myEnc(12, 13);
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100); // Defaults to 1000 mSec
}

long encoderPosn  = 0;
float dist = 0;
String inString;

void loop() {
  encoderPosn = myEnc.read();
  dist = encoderPosn * 3.14159 / 288.0; //cm

  if (Serial.available() > 0) {
    delay(10); // give time for entire string to arrive
    // Read incoming string from RasPi
    inString = Serial.readString();
    delay(10);
    Serial.println(dist);
  }
}
