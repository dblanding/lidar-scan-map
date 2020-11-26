/*
 * Motor control code for up to 8 motors w/ 2 stacked mtr shields
 * Accepts input (from Raspberry Pi) on SPI bus (2 bytes).
 * Lower byte: motor speed (0-255)
 * Upper byte: bits 1,2,3 (0x1 = mtr1 ... 0x7 = mtr7); 4th bit = REVERSE rotation
*/

#include <Adafruit_MotorShield.h>

#define SCK_PIN   13  // D13 = pin19
#define MISO_PIN  12  // D12 = pin18
#define MOSI_PIN  11  // D11 = pin17
#define SS_PIN    10  // D10 = pin16

#define UL unsigned long
#define US unsigned short

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x60); 
Adafruit_DCMotor *mtr1 = AFMS1.getMotor(1); // mtr1 is a pointer
Adafruit_DCMotor *mtr2 = AFMS1.getMotor(2);
Adafruit_DCMotor *mtr3 = AFMS1.getMotor(3);
Adafruit_DCMotor *mtr4 = AFMS1.getMotor(4);

Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x61); 
Adafruit_DCMotor *mtr5 = AFMS2.getMotor(1); // mtr5 is a pointer
Adafruit_DCMotor *mtr6 = AFMS2.getMotor(2);
Adafruit_DCMotor *mtr7 = AFMS2.getMotor(3);
Adafruit_DCMotor *mtr8 = AFMS2.getMotor(4);

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

void setup()
{
  Serial.begin(115200); // For communication w/ controlling computer
  Serial.setTimeout(50);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello from Arduino motorControl");
  SlaveInit();  // set up SPI slave mode
  AFMS1.begin();
  AFMS2.begin();
  mtr1->setSpeed(0); // use '->' (not '.') because mtr1 is a pointer
  mtr2->setSpeed(0);
  mtr3->setSpeed(0);
  mtr4->setSpeed(0);
  mtr5->setSpeed(0);
  mtr6->setSpeed(0);
  mtr7->setSpeed(0); // lidar rotor
  mtr8->setSpeed(0);
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
  mtr = (high & 0x07); // lowest 3 bits of upper byte
  rev = (high & 0x8);  // 4th bit of upper byte (1 = reverse)
  spd = low; // motor speed (0-255)
  Serial.print("mtr: ");
  Serial.print(mtr);
  Serial.print("; speed: ");
  Serial.print(spd);
  Serial.print("; reverse ?: ");
  Serial.println(rev);
  if (mtr == 1) {
    if (rev) {
      mtr1->run(BACKWARD);
    }
    else {
      mtr1->run(FORWARD);
    }
    mtr1->setSpeed(spd);
  }
  if (mtr == 2) {
    if (rev) {
      mtr2->run(BACKWARD);
    }
    else {
      mtr2->run(FORWARD);
    }
    mtr2->setSpeed(spd);
  }
  if (mtr == 3) {
    if (rev) {
      mtr3->run(BACKWARD);
    }
    else {
      mtr3->run(FORWARD);
    }
    mtr3->setSpeed(spd);
  }
  if (mtr == 4) {
    if (rev) {
      mtr4->run(BACKWARD);
    }
    else {
      mtr4->run(FORWARD);
    }
    mtr4->setSpeed(spd);
  }
  if (mtr == 5) {
    if (rev) {
      mtr5->run(BACKWARD);
    }
    else {
      mtr5->run(FORWARD);
    }
    mtr5->setSpeed(spd);
  }
  if (mtr == 6) {
    if (rev) {
      mtr6->run(BACKWARD);
    }
    else {
      mtr6->run(FORWARD);
    }
    mtr6->setSpeed(spd);
  }
  if (mtr == 7) {
    if (rev) {
      mtr7->run(BACKWARD);
    }
    else {
      mtr7->run(FORWARD);
    }
    mtr7->setSpeed(spd);
  }
  if (mtr == 8) {
    if (rev) {
      mtr8->run(BACKWARD);
    }
    else {
      mtr8->run(FORWARD);
    }
    mtr8->setSpeed(spd);
  }
}
