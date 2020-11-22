"""This program accesses all the functions of the zero turn car.

  * Motor drive by command to the arduino (then through motor shield)
  * Access to TFminiPlus data
  * Access to rotary angle encoder (through ADC)

The TFminiPlus uses the RasPi's only serial bus for data transfer.
The Adafruit motor shield (v2.3) uses the I2C bus to talk to the Arduino.
Attempts to set up the RasPi and Arduino communication over the I2C bus have
been unsuccessful because of an interference with the motor shield to Arduino
communucation. Therefore, the SPI bus is used for Raspberry Pi - Arduino
communication. The Raspberry Pi is configured as SPI master, and the Arduino
as a slave.
"""

import Adafruit_ADS1x15
import serial
import spidev
import time

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1

ser = serial.Serial("/dev/ttyS0", 115200)

spi = spidev.SpiDev()  # Enable SPI
bus = 0  # We only have SPI bus 0 available to us on the Pi
device = 0  #Device is the chip select pin. Set to 0 or 1
spi.open(bus, device)  # Open a connection
spi.max_speed_hz = 500000  # Set SPI speed and mode
spi.mode = 0

cc = []  # counter counter
distance = 0  # global value gets updated
def read_dist():  # Update global value of distance, return counter
    global distance
    counter = ser.in_waiting # number of bytes on serial port
    cc.append(counter)
    if counter > 8:
        bytes_serial = ser.read(9)
        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
            distance = bytes_serial[2] + bytes_serial[3]*256
            strength = bytes_serial[4] + bytes_serial[5]*256
            temperature = bytes_serial[6] + bytes_serial[7]*256
            temperature = (temperature/8) - 256
        ser.flushInput()  # Keep the buffer empty (purge stale data)
    return counter

# Turn scan motor on
msg = [3, 255]
result = spi.xfer(msg)

# collect scan data
enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
last_time = time.time()
data = []
while enc_val < 32767:  # continue as values increase
    enc_val = adc.read_adc(0, gain=GAIN)
    if 10000 < enc_val < 30000:
        counter = read_dist()
        cntr = str(counter)
        dist = str(distance)
        e_val = str(enc_val)
        now = time.time()
        delta_t = str(now - last_time)
        delt = "{:.6}".format(delta_t)
        data.append(e_val+' '+dist+' '+cntr+' '+delt+'\n')
        last_time = now

while enc_val == 32767:  # continue to 360 / 0 transition
    enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)

# Turn scan motor off
msg = [3, 0]
result = spi.xfer(msg)

# save scan data
print("Number of data points: ", len(data))
with open('scan_data', 'w') as f:
    f.writelines(data)
spi.close()
ser.close()
