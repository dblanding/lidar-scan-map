/*
 * This sketch combines:
 * Serial communication with onboard RasPi of ZT car (in main loop)
 * Operation of lidar rotor scan & home (as functions)
 * 
 * The RasPi sends commands as 3 comma separated substrings: "str1,str2,str3"
 * The first two are left and right wheel motor speeds between -255 & +255
 * The third is an integer designating a rotor function to run
 *
 * Scan distance is measured by a TFMini lidar module
 * The (UNO) circuit:
 * Uno RX is digital pin 10 (connect to TX (grn) of TF Mini)
 * Uno TX is digital pin 11 (connect to RX (wht) of TF Mini)
 * 
 * A shaft angle encoder measures scan angle (0-5V out <-> 0-360 deg)
 * connected to pin A0
*/

#include <SoftwareSerial.h>
#include "TFMini.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Setup software serial port: Uno RX (TFMINI TX), Uno TX (TFMINI RX)
SoftwareSerial mySerial(10, 11);
TFMini tfmini;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *mtrL = AFMS.getMotor(1);
Adafruit_DCMotor *mtrR = AFMS.getMotor(2);
Adafruit_DCMotor *mtrM = AFMS.getMotor(3);

int Lspd; // left wheel motor speed
int Rspd; // right wheel motor speed
int angleValue; // lidar rotor encoder count value
int error; // encoder count value w/r/t home position
int rotation = -1; // (+) for CW, (-) for CCW (lidar rotor)
bool scanMode; // lidar rotor in scan mode

void setup() {
  Serial.begin(115200);  // For communication w/ controlling computer
  Serial.setTimeout(10); // serial.readString will terminate if it doesn't complete in 10 mSec
  while (!Serial) {
    Serial.println ("Initializing...");
  }
  Serial.println("Hello from ZeroTurnCar");
  Serial.flush();
  AFMS.begin();
  mtrL->setSpeed(0);
  mtrR->setSpeed(0);
  mtrM->setSpeed(0);
  mtrM->run(FORWARD);
  mySerial.begin(TFMINI_BAUDRATE);
  tfmini.begin(&mySerial);
}
 
void loop() {
  // Read serial input:
  if (Serial.available() > 0) {
    //delay(1); // give time for entire string to arrive
    String inString = Serial.readString();
    String str1 = getSubString(inString, ',', 0);
    String str2 = getSubString(inString, ',', 1);
    String str3 = getSubString(inString, ',', 2);
    int Lspd = str1.toInt();
    int Rspd = str2.toInt();
    int doFunction = str3.toInt();
    if (doFunction == 1) {
      homeRotor();
    }
    if (doFunction == 2) {
      scanRotor();
    }
    if (Lspd < 0) {
      mtrL->run(BACKWARD);
    }
    else {
      mtrL->run(FORWARD);
    }
    if (Rspd < 0) {
      mtrR->run(BACKWARD);
    }
    else {
      mtrR->run(FORWARD);
    }
    if (Lspd > 255) {
      Lspd = 255;
    }
    if (Rspd > 255) {
      Rspd = 255;
    }
    if (Lspd < -255) {
      Lspd = -255;
    }
    if (Rspd < -255) {
      Rspd = -255;
    }
    mtrL->setSpeed(abs(Lspd));
    mtrR->setSpeed(abs(Rspd));
    // Send a character to ask for more data
    Serial.println("A");
    Serial.flush();
  }
}

// Split string into substrings, return substring by index
String getSubString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Move rotor to home position (512)
void homeRotor() {
  while (abs(analogRead(A0) - 512) > 0) {
    angleValue = analogRead(A0);
    if (angleValue < 512) { 
    mtrM->run(BACKWARD);
    }
    if (angleValue > 512) {
      mtrM->run(FORWARD);
    }
    error = abs(angleValue - 512);
    if (error > 255) {
      error = 255;
    }
    if (error < 30) {
      error = 30;
    }
    mtrM->setSpeed(error);
  }
  mtrM->setSpeed(0);
}

void scanRotor() {
  scanMode = true;
  mtrM->run(FORWARD);
  mtrM->setSpeed(255);
  rotation = -1;
  while (scanMode) {
    // Take one TF Mini distance measurement
    uint16_t dist = tfmini.getDistance();
    // uint16_t strength = tfmini.getRecentSignalStrength();
    // read shaft angle in bits
    angleValue = analogRead(A0);
    if (angleValue < 128) { // 45 deg beyond left
      mtrM->run(BACKWARD);
      rotation = 1;
    }
    if (angleValue > 768) { // looking right
      scanMode = false;
      homeRotor();
    }
    if (dist > 1200) {  // 12 m max indoor distance
      dist = 1200;
    }
    if (scanMode and (rotation > 0) and (angleValue > 255)) {
      Serial.print(rotation);
      Serial.print(", ");
      Serial.print(dist);
      Serial.print(", ");
      Serial.println(angleValue);
      Serial.flush();
      delay(4);
    }
  }
}
