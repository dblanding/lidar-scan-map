/*
 * This sketch controls 3 motors using the adafruit motor shield v2.3.
 * An unsigned short (2 bytes) are received on the SPI bus.
 * The lower byte (0-255) is used to set motor speed.
 * The lowest 2 bits of the upper byte specify the motor (1, 2 or 3)
 * The 3rd bit of the upper byte signifies reverse direction (if set).
 * bits 4-8 of the upper byte are unused.
*/

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define SCK_PIN   13  // D13 = pin19 = PortB.5
#define MISO_PIN  12  // D12 = pin18 = PortB.4
#define MOSI_PIN  11  // D11 = pin17 = PortB.3
#define SS_PIN    10  // D10 = pin16 = PortB.2

#define UL unsigned long
#define US unsigned short

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *mtrL = AFMS.getMotor(1);
Adafruit_DCMotor *mtrR = AFMS.getMotor(2);
Adafruit_DCMotor *mtrM = AFMS.getMotor(3);

int mtr = 0;
int spd = 0;
bool rev = false;

void SlaveInit(void) {
  // Set MISO output, all others input
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);  // (only if bidirectional mode needed)
  pinMode(SS_PIN, INPUT);

  /*  Setup SPI control register SPCR
  SPIE - Enables the SPI interrupt when 1
  SPE - Enables the SPI when 1
  DORD - Sends data least Significant Bit First when 1, most Significant Bit first when 0
  MSTR - Sets the Arduino in master mode when 1, slave mode when 0
  CPOL - Sets the data clock to be idle when high if set to 1, idle when low if set to 0
  CPHA - Samples data on the trailing edge of the data clock when 1, leading edge when 0
  SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz) 11 is slowest (250KHz)   */
 
  // enable SPI subsystem and set correct SPI mode
  // SPCR = (1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(1<<SPR0);
}

// SPI status register: SPSR
// SPI data register: SPDR

// ================================================================
// read in short as two bytes, with high-order byte coming in first
// ================================================================
unsigned short Read2Bytes(void) {
  union {
  unsigned short svar;
  byte c[2];
  } w;        // allow access to 2-byte word, or separate bytes
 
  while(!(SPSR & (1<<SPIF))) ; // SPIF bit set when 8 bits received
  w.c[1] = SPDR;               // store high-order byte
  while(!(SPSR & (1<<SPIF))) ; // SPIF bit set when 8 bits received
  w.c[0] = SPDR;               // store low-order byte
  return (w.svar); // send back unsigned short value
}

void setup() {
  Serial.begin(115200);  // For communication w/ controlling computer
  Serial.println("Hello from ZeroTurnCar");
  SlaveInit();  // set up SPI slave mode
  AFMS.begin();
  mtrL->setSpeed(0);
  mtrR->setSpeed(0);
  mtrM->setSpeed(0);
  delay(10);
}

// ============================================================
// main loop: read in short word (2 bytes) from external SPI master
// use low byte for speed; high byte for motor and fwd/rev.
// On 16 MHz Arduino, works at > 500 words per second
// ============================================================
void loop() {
  unsigned short word1;
  byte flag1;

  // SS_PIN = Digital_10 = ATmega328 Pin 16 =  PORTB.2
  // Note: digitalRead() takes 4.1 microseconds
  // NOTE: SS_PIN cannot be properly read this way while SPI module is active!
  while (digitalRead(SS_PIN)==1) {} // wait until SlaveSelect goes low (active)
   
  SPCR = (1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(1<<SPR0);  // SPI on
  word1 = Read2Bytes();  // read unsigned short value
  SPCR = (0<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(1<<SPR0);  // SPI off
 
  //    float seconds = millis()/1000.0;
  //    time stamp takes more serial time, of course
  //    Serial.print(seconds,3);  
  //    Serial.print(",");
  uint8_t low = word1 & 0xff;
  uint8_t high = (word1 >> 8);
  mtr = (high & 0x03);
  rev = (high & 0x4);
  spd = low;
  Serial.print("mtr: ");
  Serial.print(mtr);
  Serial.print("; speed: ");
  Serial.print(spd);
  Serial.print("; reverse ?: ");
  Serial.println(rev);
  if (mtr == 1) {
    if (rev) {
      mtrL->run(BACKWARD);
    }
    else {
      mtrL->run(FORWARD);
    }
    mtrL->setSpeed(spd);
  }
  if (mtr == 2) {
    if (rev) {
      mtrR->run(BACKWARD);
    }
    else {
      mtrR->run(FORWARD);
    }
    mtrR->setSpeed(spd);
  }
  if (mtr == 3) {
    if (rev) {
      mtrM->run(BACKWARD);
    }
    else {
      mtrM->run(FORWARD);
    }
    mtrM->setSpeed(spd);
  }
}
