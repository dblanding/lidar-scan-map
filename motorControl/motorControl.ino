/*
 * Motor control code for 5 motors on 2 stacked mtr shields
 * Listen on serial port for 6 comma separated integer values
 * First int value is a flag.
 * if flag == 0:  just send back sensor data
 * if flag == 1:  command 4 wheel motors with next 4 values
 * if flag == 2:  command lidar rotor motor with last value
 * regardless of flag value always send back sensor data
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MotorShield.h>
#define trigPin1 2
#define echoPin1 3
#define trigPin2 4
#define echoPin2 5
#define trigPin3 6
#define echoPin3 7

// Global variables needed by SR04 sensors
long duration, distance, RightSensor, FrontSensor, LeftSensor;

/* Create object anad Assign a unique ID to HMC5883L. */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Global variables needed by lidar sensor
#define UL unsigned long
#define US unsigned short

// Initialize Adafruit motor shield (wheel motors)
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x61); 
Adafruit_DCMotor *mtr1 = AFMS1.getMotor(1); // mtr1 is a pointer
Adafruit_DCMotor *mtr2 = AFMS1.getMotor(2);
Adafruit_DCMotor *mtr3 = AFMS1.getMotor(3);
Adafruit_DCMotor *mtr4 = AFMS1.getMotor(4);
// another shield with up to 4 more motors
Adafruit_MotorShield AFMS0 = Adafruit_MotorShield(0x60); 
Adafruit_DCMotor *mtr5 = AFMS0.getMotor(1);
Adafruit_DCMotor *mtr6 = AFMS0.getMotor(2);
Adafruit_DCMotor *mtr7 = AFMS0.getMotor(3);
Adafruit_DCMotor *mtr8 = AFMS0.getMotor(4);

int mtr = 0;  // ?
int spd = 0;  // ?

void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1; // cm
}

void CommandWheels(int spd1, int spd2, int spd3, int spd4)
{
  if (spd1 < 0) {
    mtr1->run(BACKWARD);
  }
  else {
    mtr1->run(FORWARD);
  }
  if (spd2 < 0) {
    mtr2->run(BACKWARD);
  }
  else {
    mtr2->run(FORWARD);
  }
  if (spd3 < 0) {
    mtr3->run(BACKWARD);
  }
  else {
    mtr3->run(FORWARD);
  }
  if (spd4 < 0) {
    mtr4->run(BACKWARD);
  }
  else {
    mtr4->run(FORWARD);
  }
  if (spd1 > 255) {
    spd1 = 255;
  }
  if (spd2 > 255) {
    spd2 = 255;
  }
  if (spd3 > 255) {
    spd3 = 255;
  }
  if (spd4 > 255) {
    spd4 = 255;
  }
  if (spd1 < -255) {
    spd1 = -255;
  }
  if (spd2 < -255) {
    spd2 = -255;
  }
  if (spd3 < -255) {
    spd3 = -255;
  }
  if (spd4 < -255) {
    spd4 = -255;
  }
  mtr1->setSpeed(abs(spd1));
  mtr2->setSpeed(abs(spd2));
  mtr3->setSpeed(abs(spd3));
  mtr4->setSpeed(abs(spd4));
}

void CommandRotor(int spd)
{
  if (spd < 0) {
    mtr7->run(BACKWARD);
  }
  else {
    mtr7->run(FORWARD);
  }
  if (spd > 255) {
    spd = 255;
  }
  if (spd < -255) {
    spd = -255;
  }
  mtr7->setSpeed(abs(spd));
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

void setup()
{
  Serial.begin(9600); // For communication w/ controlling computer
  Serial.setTimeout(10);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello from Arduino :-)");
  Serial.flush();
  AFMS0.begin();
  AFMS1.begin();
  // use mtrx->setSpeed (not mtrx.setSpeed) because mtrx is a pointer
  mtr1->setSpeed(0); // wheel 1 motor
  mtr2->setSpeed(0); // wheel 2 motor
  mtr3->setSpeed(0); // wheel 3 motor
  mtr4->setSpeed(0); // wheel 4 motor
  mtr5->setSpeed(0);
  mtr6->setSpeed(0);
  mtr7->setSpeed(0); // lidar rotor
  mtr8->setSpeed(0);


  /* Initialize pins for ultrasonic sensors */
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}
 
void loop() {
  // Get ultrasonic sensor data.
  SonarSensor(trigPin1, echoPin1);
  FrontSensor = distance;
  /*
  SonarSensor(trigPin2, echoPin2);
  LeftSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  RightSensor = distance;
  */
  if (Serial.available() > 0) {
    // Read incoming string from RasPi
    String inString = Serial.readString();
    // send acknowledgement back to RasPi
    Serial.println("100,0,0");
    // Parse incoming data and command motors accordingly
    String str0 = getSubString(inString, ',', 0);
    String str1 = getSubString(inString, ',', 1);
    String str2 = getSubString(inString, ',', 2);
    String str3 = getSubString(inString, ',', 3);
    String str4 = getSubString(inString, ',', 4);
    String str5 = getSubString(inString, ',', 5);
    int flag = str0.toInt();
    int spd1 = str1.toInt();
    int spd2 = str2.toInt();
    int spd3 = str3.toInt();
    int spd4 = str4.toInt();
    int spd5 = str5.toInt();
    if (flag == 1)
    {
      CommandWheels(spd1, spd2, spd3, spd4);
    }
    if (flag == 2)
    {
      CommandRotor(spd5);
    }
  }
  //if (FrontSensor <= 10)
  //{
    //CommandWheels(0, 0, 0, 0);
  //}
}
