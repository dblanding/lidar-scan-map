/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
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
}

long oldPosition  = -999;
float dist = 0;
bool flow = false; // data flow

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    if (flow) {
      dist = newPosition * 3.14159 / 288; //cm
      Serial.println(dist);
    }
  }
  if (Serial.available() > 0) {
    // Read incoming string from RasPi
    String inString = Serial.readString();
    delay(10);
    if (inString == "stop\n") {
      flow = false;
    }
    else {
      flow = true;
    }
  }
}
