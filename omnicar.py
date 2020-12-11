"""This program accesses all the functions of the omni wheel car.

  * Motor drive by command to the Arduino (then through motor shield)
  * Access TFminiPlus data through serial bus
  * Access to rotary angle encoder (through ADC)

The TFminiPlus uses the RasPi's only serial bus for data transfer.
Communication between the Arduino and the Adafruit motor shield (v2.3)
uses the I2C bus.
Attempts to set up the RasPi and Arduino communication over the I2C
bus have been unsuccessful because it interferes with the Arduino
to motor shield communication. To sidestep this problem, commands
are sent from the Raspberry Pi to the Arduino on the SPI bus, with
the Raspberry Pi configured as SPI master, and the Arduino as slave.

Wheel motors:
For an explanation of how the wheel motors are driven, see omni-wheels.md
"""

import Adafruit_ADS1x15
import serial
import spidev
import time

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1  #ADC gain

ser = serial.Serial("/dev/ttyS0", 115200)

spi = spidev.SpiDev()  # Enable SPI
bus = 0  # We only have SPI bus 0 available to us on the Pi
device = 0  # Device is the chip select pin. Set to 0 or 1
spi.open(bus, device)  # Open a connection
spi.max_speed_hz = 500000  # Set SPI speed and mode
spi.mode = 0
spi_wait = .006  # wait time (sec) between successive spi commands


class OmniCar():

    def __init__(self):
        """
        class OmniCar
        Access functions of omni-wheel car.
        """
        self.distance = 0  # distance (cm) last measured by LiDAR module

    def go_FR(self, spd):
        """Drive forward + right at spd (int between 0-255)."""
        msg1 = [1, spd]
        msg2 = [2+8, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_FL(self, spd):
        """Drive forward + left at spd (int between 0-255)."""
        msg1 = [4, spd]
        msg2 = [3+8, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_BL(self, spd):
        """Drive back + left at spd (int between 0-255)."""
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_BR(self, spd):
        """Drive back + right at spd (int between 0-255)."""
        msg1 = [4+8, spd]
        msg2 = [3, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_F(self, spd, trim=None):
        """Drive car forward at speed = spd (int: 0-255).
        trim (int) is used to null out any unwanted spin."""
        if not trim:
            trim = 0
        msg4 = [4, spd + trim]  # High byte, Low byte
        msg3 = [3+8, spd - trim]  # 4th bit in high byte -> reverse dir
        msg1 = [1, spd + trim]
        msg2 = [2+8, spd - trim]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_B(self, spd):
        """Drive car backward at speed = spd (int: 0-255)."""
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_L(self, spd):
        """Drive car left at speed = spd (int: 0-255)."""
        msg4 = [4, spd]
        msg3 = [3+8, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_R(self, spd):
        """Drive car right at speed = spd (int: 0-255)."""
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1, spd]
        msg2 = [2+8, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def spin_ccw(self, spd):
        """Spin car CCW at speed = spd (int between 0-255)."""
        for n in range(1, 5):
            _ = spi.xfer([n, spd])
            time.sleep(spi_wait)

    def spin_cw(self, spd):
        """Spin car CW at speed = spd (int between 0-255)."""
        for n in range(1, 5):
            _ = spi.xfer([n+8, spd])
            time.sleep(spi_wait)

    def run_mtr(self, mtr, spd, rev=False):
        """
        Drive motor = mtr (int 0-7) at speed = spd (int 0-255)
        in reverse direction if rev=True.
        """
        if rev:
            msg = [mtr+8, spd]
        else:
            msg = [mtr, spd]
        _ = spi.xfer(msg)

    def stop_wheels(self):
        """Stop all wheel motors (mtr numbers: 1 thrugh 4)."""
        for n in range(1, 5):
            _ = spi.xfer([n, 0])
            time.sleep(spi_wait)

    def read_dist(self):
        """
        Set self.distance = distance (cm) read from LiDAR module.
        Return number of bytes that were waiting on serial port.
        """
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

    def scan_mtr_start(self, spd=None):
        """Turn scan motor on at speed = spd (int between 0-255)."""
        if not spd:
            spd = 200  # default value
        self.run_mtr(7, spd)

    def scan_mtr_stop(self):
        """Turn scan motor off."""
        self.run_mtr(7, 0)

    def scan(self):
        """Return list of tuples of scan data values.
        """
        enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
        # If scan rotor isn't near BDC (back dead cntr), go to BDC
        if enc_val > 3000:
            self.scan_mtr_start()
            while enc_val < 32767:  # continue as values increase to max
                enc_val = adc.read_adc(0, gain=GAIN)
            while enc_val == 32767:  # continue to back dead cntr
                enc_val = adc.read_adc(0, gain=GAIN)
        else:
            self.scan_mtr_start()
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
        self.scan_mtr_stop()
        return data

    def close(self):
        spi.close()
        ser.close()
