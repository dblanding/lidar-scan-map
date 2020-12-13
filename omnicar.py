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
See omni-wheels.md for an explanation of wheel motor drive.
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
    """
    class OmniCar: Access all functions of omni-wheel car.
    """

    def __init__(self):
        """
        Constructor for omni-wheel car.
        Store distance as attribute.
        """
        self.distance = 0  # distance (cm) of last measured LiDAR value

    def run_mtr(self, mtr, spd, rev=False):
        """
        Drive motor = mtr (int 0-7) at speed = spd (int 0-255)
        in reverse direction if rev=True.
        """
        if not rev:
            msg = [mtr, spd]  # High byte, Low byte
        else:
            msg = [mtr+8, spd]  # 4th bit in high byte -> reverse dir
        _ = spi.xfer(msg)

    def go_F(self, spd, trim=None):
        """Drive forward at speed = spd with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg4 = [4, spd + trim]
        msg3 = [3+8, spd - trim]
        msg1 = [1, spd + trim]
        msg2 = [2+8, spd - trim]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_B(self, spd, trim=None):
        """Drive backward at speed = spd with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_L(self, spd, trim=None):
        """Drive left at speed = spd with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg4 = [4, spd]
        msg3 = [3+8, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_R(self, spd, trim=None):
        """Drive right at speed = spd with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg4 = [4+8, spd - trim]
        msg3 = [3, spd + trim]
        msg1 = [1, spd + trim]
        msg2 = [2+8, spd - trim]
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

    def govern(self, spd, trim):
        """limit max values of spd and trim.
        spd + trim must not exceed 255.
        """
        if spd > 240:
            spd = 240
        if trim > 15:
            trim = 15
        if trim < -15:
            trim = -15
        return spd, trim

    def stop_wheels(self):
        """Stop all wheel motors (mtr numbers: 1 thrugh 4)."""
        for n in range(1, 5):
            _ = spi.xfer([n, 0])
            time.sleep(spi_wait)

    def go_FR(self, spd, trim=None):
        """Drive diagonally forward + right at speed = spd
        with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg1 = [1, spd + trim]
        msg2 = [2+8, spd - trim]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_FL(self, spd, trim=None):
        """Drive diagonally forward + left at speed = spd
        with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg1 = [4, spd + trim]
        msg2 = [3+8, spd - trim]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_BL(self, spd, trim=None):
        """Drive diagonally back + left at speed = spd
        with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg1 = [1+8, spd - trim]
        msg2 = [2, spd + trim]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_BR(self, spd, trim=None):
        """Drive diagonally back + right at speed = spd
        with steering trim."""
        if not trim:
            trim = 0
        spd, trim = self.govern(spd, trim)
        msg1 = [4+8, spd - trim]
        msg2 = [3, spd + trim]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
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

    def get_enc_val(self):
        """Return encoder value from LiDAR rotor angular encoder."""
        return adc.read_adc(0, gain=GAIN, data_rate=250)

    def scan_mtr_start(self, spd=None):
        """Turn scan motor on at speed = spd (int between 0-255)."""
        if not spd:
            spd = 200  # default value
        self.run_mtr(7, spd)

    def scan_mtr_stop(self):
        """Turn scan motor off."""
        self.run_mtr(7, 0)

    def scan(self, low_enc_val=10000, hi_enc_val=30000):
        """Return list of tuples of scan data for encoder values between
        low_enc_val and hi_enc_val.
        """
        enc_val = self.get_enc_val()
        # If scan rotor isn't near BDC (back dead cntr), go to BDC
        if enc_val > 3000:
            self.scan_mtr_start()
            while enc_val < 32767:  # continue as values increase to max
                enc_val = self.get_enc_val()
            while enc_val == 32767:  # continue to back dead cntr
                enc_val = self.get_enc_val()
        else:
            self.scan_mtr_start()
            enc_val = self.get_enc_val()
        last_time = time.time()
        data = []
        while enc_val < 32767:  # continue as values increase to max
            enc_val = self.get_enc_val()
            if low_enc_val < enc_val < hi_enc_val:  # 20000 is 'straight ahead'
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
