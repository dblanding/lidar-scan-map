"""This program accesses all the functions of the omni wheel car.

  * Motor drive by command to the arduino (then through motor shield)
  * Access TFminiPlus data through serial bus
  * Access to rotary angle encoder (through ADC)

The TFminiPlus uses the RasPi's only serial bus for data transfer.
The Adafruit motor shield (v2.3) uses the Arduino I2C bus.
Attempts to set up the RasPi and Arduino communication over the I2C
bus have been unsuccessful because of an interference with the Arduino
to motor shield communucation. To sidestep the problem, the SPI bus is
used for communication between the Raspberry Pi and Arduino.
The Raspberry Pi is configured as SPI master, and the Arduino as slave.
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
device = 0  # Device is the chip select pin. Set to 0 or 1
spi.open(bus, device)  # Open a connection
spi.max_speed_hz = 500000  # Set SPI speed and mode
spi.mode = 0


class OmniCar():
    
    def __init__(self):
        self.distance = 0

    def go_fwd(self, spd):
        """spd is an int between 0-255"""
        msg4 = [4, spd]  # High byte, Low byte
        msg3 = [3+8, spd]  # 4th bit in high byte -> reverse dir
        msg1 = [1, spd]
        msg2 = [2+8, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(.005)

    def go_back(self, spd):
        """spd is an int between 0-255"""
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(.005)

    def go_left(self, spd):
        """spd is an int between 0-255"""
        msg4 = [4, spd]
        msg3 = [3+8, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(.005)

    def go_right(self, spd):
        """spd is an int between 0-255"""
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(.005)

    def run_mtr(self, mtr, spd, rev=False):
        """mtr is an int between 1-5, spd is an int 0-255"""
        if rev:
            msg = [mtr+8, spd]
        else:
            msg = [mtr, spd]
        _ = spi.xfer(msg)

    def stop_wheels(self):
        """stops all wheel motors (1,2,3,4)"""
        for n in range(1, 5):
            _ = spi.xfer([n, 0])
            time.sleep(.005)

    def read_dist(self):
        counter = ser.in_waiting # bytes available on serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                self.distance = bytes_serial[2] + bytes_serial[3]*256
                self.strength = bytes_serial[4] + bytes_serial[5]*256
                temperature = bytes_serial[6] + bytes_serial[7]*256
                self.temperature = (temperature/8) - 256
            ser.flushInput()  # Keep the buffer empty (purge stale data)
        return counter

    def scan(self):
        """Return list tuples of scan data values."""
        enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
        # If scan rotor isn't near BDC (back dead cntr), go to BDC
        if enc_val > 3000:
            self.run_mtr(5, 255)  # start scan mtr
            while enc_val < 32767:  # continue as values increase to max
                enc_val = adc.read_adc(0, gain=GAIN)
            while enc_val == 32767:  # continue to back dead cntr
                enc_val = adc.read_adc(0, gain=GAIN)
        else:
            self.run_mtr(5, 255)  # start scan mtr
            enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
        last_time = time.time()
        data = []
        while enc_val < 32767:  # continue as values increase to max
            enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
            if 10000 < enc_val < 30000:  # 20000 is 'straight ahead'
                counter = self.read_dist()
                now = time.time()
                delta_t = str(now - last_time)
                last_time = now
                data_item = (enc_val, self.distance, counter, delta_t)
                data.append(data_item)

        while enc_val == 32767:  # continue to back dead cntr
            enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)

        self.run_mtr(5, 0)  # stop scan mtr
        return data


    def close(self):
        spi.close()
        ser.close()
