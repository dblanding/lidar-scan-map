/*
 * Tutorial: RASPBERRY PI AND ARDUINO CONNECTED USING I2C
 * https://oscarliang.com/raspberry-pi-arduino-connected-i2c/
*/

#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Ready!");
}

void loop() {
  // nothing needed here since we're doing event based code
  delay(100);
}

// callback for received data
void receiveData(int byteCount){

  while(Wire.available()) {
    number = Wire.read();
    Serial.print("data received: ");
    Serial.println(number);
    
    if (number == 1){
    
      if (state == 0){
        digitalWrite(LED_BUILTIN, HIGH); // set the LED on
        state = 1;
      }
      else{
        digitalWrite(LED_BUILTIN, LOW); // set the LED off
        state = 0;
      }
    }
  }
}

// callback for sending data
void sendData(){
Wire.write(number);
}
