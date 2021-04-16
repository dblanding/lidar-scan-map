/* 5 cm dia odometer wheel on incremental encoder
 * 1440 pulses per rev 
 * Reports integer (0-255) distance (cm) 
 * to Serial and to 8-bit DAC pin 25.
 * 0 cm = 0.117 V
 * 255 cm (full scale) = 3.135 V
 */

#include <Encoder.h>

Encoder myEnc(16, 17);

void setup() {
  Serial.begin(9600);
}

int oldDist  = -999;

void loop() {
  long encoderPosn = myEnc.read();
  int dist = encoderPosn * 3.14159 / 288.0; //cm
  if (dist != oldDist) {
    dacWrite(25, dist);
    Serial.println(dist);
    oldDist = dist;
  }    
}
