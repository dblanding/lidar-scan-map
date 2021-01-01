"""This program accesses all the functions of the omni wheel car.

  * Motor drive by command to the Arduino through SPI bus
  * Access TFminiPlus data through serial port
  * Access to rotary angle encoder through ADC on I2C bus
  * Access to compass heading through HMC5883L on I2C bus

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
import logging
import math
import os
import serial
import smbus
import sys
import time
import Adafruit_ADS1x15

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1  #ADC gain

# communicate with Arduino (bi-directional)
ports = ['/dev/ttyACM0', '/dev/ttyACM1']
for port in ports:
    if os.path.exists(port):
        ser0 = serial.Serial(port, 9600, timeout=.05)
        ser0.flush()
        break
logger.info(f"Serial port = {port}")

# get data from lidar module
ser1 = serial.Serial("/dev/ttyUSB0", 115200)

i2cbus = smbus.SMBus(1)
HMC5883_ADDRESS = 0x1e   # 0x3c >> 1
# HMC5883L Registers and their Address
REGISTER_A = 0x00  # Address of Configuration register A
REGISTER_B = 0x01  # Address of configuration register B
REGISTER_MODE = 0x02  # Address of mode register
X_AXIS_H = 0x03  # Address of X-axis MSB data register
Z_AXIS_H = 0x05  # Address of Z-axis MSB data register
Y_AXIS_H = 0x07  # Address of Y-axis MSB data register

def convert_polar_to_rect(r, theta):
    """Convert polar coords (r, theta) to rectangular coords (x,y)
    theta is in radians."""
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return (x, y)

class OmniCar():
    """
    Access all functions of omni-wheel car.
    """

    def __init__(self):
        """Configure HMC5883L, store most recent sensor data."""

        # write to Configuration Register A
        # average 8 samples per measurement output
        i2cbus.write_byte_data(HMC5883_ADDRESS, REGISTER_A, 0x70)

        # Write to Configuration Register B for gain
        # Use default gain
        i2cbus.write_byte_data(HMC5883_ADDRESS, REGISTER_B, 0x20)

        # Write to mode Register to specify mode
        # 0x01 (single measurement mode) is default
        # 0x00 (continuous measurenent mode)
        i2cbus.write_byte_data(HMC5883_ADDRESS, REGISTER_MODE, 0x00)

        self.distance = 0  # last measured distance (cm) from lidar
        
    def _read_serial_data(self):
        line = "No serial data available"
        if ser0.in_waiting:
            in_bytes = ser0.readline()
            in_string = in_bytes.decode("utf-8")
            line = in_string.strip().split(',')
            '''
            try:
                read_line = ser0.read_until()   # byte type
                line = read_line.decode("utf-8").strip()
            except UnicodeDecodeError:
                line = None
            '''
        return line


    def _xfer_data(self, send_data):
        """Xfer data w/ Arduino: Send motor spd values, get sensor data

        Send a tuple of 6 int values: (flag, m1, m2, m3, m4, m5)
        flag = 0 (ignore motor values, just get sensor data)
        flag = 1 (apply motor values m1 thru m4 and get sensor data)
        flag = 2 (apply motor value m5 and get sensor data)
        """
        # convert integers to strings for sending on serial
        send_data_str = (str(item) for item in send_data)
        logger.debug(f"Data to send: {send_data}")
        out_string = ",".join(send_data_str)
        logger.debug(f"String being sent: {out_string}")
        ser0.write(out_string.encode())
        time.sleep(.05)  # wait for incoming sensor data
        sensor_data = 1234
        sensor_data = self._read_serial_data()
        logger.debug(f"serial data read: {sensor_data}")
        return sensor_data

    def read_raw_data(self, addr):
        """Read from HMC5883L data registers"""

        # Read raw 16-bit value
        high = i2cbus.read_byte_data(HMC5883_ADDRESS, addr)
        low = i2cbus.read_byte_data(HMC5883_ADDRESS, addr+1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # get signed value from module
        if value > 32768:
            value = value - 65536
        return value

    def heading(self):
        """Return magnetic compass heading of car (degrees)."""

        # Read raw value
        x = self.read_raw_data(X_AXIS_H)
        z = self.read_raw_data(Z_AXIS_H)
        y = self.read_raw_data(Y_AXIS_H)
        # print(x, y, z)
        # working in radians...
        heading = math.atan2(y, x)

        # calibration worked out experimentally
        heading = heading + .44 * math.cos((heading + .175)) + .0175
        heading = heading - math.pi / 2

        # check for sign
        if heading < 0:
            heading += 2 * math.pi

        # convert into angle
        heading_angle = int(heading * 180 / math.pi)

        return heading_angle

    def go(self, speed, theta, spin=0):
        """Drive at speed (int) in relative direction theta (rad)
        while simultaneoulsy spinning CCW at rate = spin (int).
        return sensor_data."""

        # convert from polar coordinates to omni_car's 'natural' coords
        # where one coaxial pair of wheels drives u and the other drives v
        u, v = convert_polar_to_rect(speed, theta - math.pi/4)

        # motor numbers
        m1, m2, m3, m4 = (1, 2, 3, 4)

        # convert 0-100 motor speed to 0-200 integer, adding spin
        m1spd = int(spin + u) * 2
        m2spd = int(spin - u) * 2
        m3spd = int(spin - v) * 2
        m4spd = int(spin + v) * 2

        # friction keeps motors from running at speeds below ~50
        if 0 < m1spd < 50:
            m1spd = 50
        if 0 < m2spd < 50:
            m2spd = 50
        if 0 < m3spd < 50:
            m3spd = 50
        if 0 < m4spd < 50:
            m4spd = 50

        if -50 < m1spd < 0:
            m1spd = -50
        if -50 < m2spd < 0:
            m2spd = -50
        if -50 < m3spd < 0:
            m3spd = -50
        if -50 < m4spd < 0:
            m4spd = -50

        # motors can't handle values > 255 or < -255
        if m1spd > 255:
            m1spd = 255
        if m1spd < -255:
            m1spd = -255
        if m2spd > 255:
            m2spd = 255
        if m2spd < -255:
            m2spd = -255
        if m3spd > 255:
            m3spd = 255
        if m3spd < -255:
            m3spd = -255
        if m4spd > 255:
            m4spd = 255
        if m4spd < -255:
            m4spd = -255

        sensor_data = self._xfer_data((1, m1spd, m2spd, m3spd, m4spd, 0))
        return sensor_data
    
    def spin(self, spd):
        """Spin car (about its own axis) at speed = spd (-255 to +255) (CCW = +)."""
        sensor_data = self._xfer_data((1, spd, spd, spd, spd, 0))
        return sensor_data

    def stop_wheels(self):
        """Stop all wheel motors (mtr numbers: 1 thrugh 4)."""
        sensor_data = self._xfer_data((1, 0, 0, 0, 0, 0))
        return sensor_data

    def read_dist(self):
        """
        Set self.distance = distance (cm) read from LiDAR module.
        Return number of bytes that were waiting on serial port.
        """
        counter = ser1.in_waiting # bytes available on serial port
        if counter > 8:
            bytes_serial = ser1.read(9)
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                self.distance = bytes_serial[2] + bytes_serial[3]*256
                self.strength = bytes_serial[4] + bytes_serial[5]*256
                temperature = bytes_serial[6] + bytes_serial[7]*256
                self.temperature = (temperature/8) - 256
            ser1.flushInput()  # Keep the buffer empty (purge stale data)
        return counter

    def get_enc_val(self):
        """Return encoder value from LiDAR rotor angular encoder."""
        return adc.read_adc(0, gain=GAIN, data_rate=250)

    def scan_mtr_start(self, spd):
        """Turn scan motor on at speed = spd (int between 0-255)."""
        sensor_data = self._xfer_data((2, 0, 0, 0, 0, spd))

    def scan_mtr_stop(self):
        """Turn scan motor off."""
        sensor_data = self._xfer_data((2, 0, 0, 0, 0, 0))

    def scan(self, spd=200, lev=10000, hev=30000):
        """Run scan mtr at spd (100-255) and return list of tuples of
        scan data for encoder values between lev (low encoder value) and
        hev (high encoder value).
        """
        enc_val = self.get_enc_val()
        # If scan rotor isn't near BDC (back dead cntr), go to BDC
        if enc_val > 3000:
            _ = self.scan_mtr_start(spd)
            while enc_val < 32767:  # continue as values increase to max
                enc_val = self.get_enc_val()
            while enc_val == 32767:  # continue to back dead cntr
                enc_val = self.get_enc_val()
        else:
            _ = self.scan_mtr_start(spd)
            enc_val = self.get_enc_val()
        last_time = time.time()
        data = []
        while enc_val < 32767:  # continue as values increase to max
            enc_val = self.get_enc_val()
            if lev < enc_val < hev:  # 20000 is 'straight ahead'
                counter = self.read_dist()
                now = time.time()
                delta_t = str(now - last_time)
                last_time = now
                data_item = (enc_val, self.distance, counter, delta_t)
                data.append(data_item)
        _ = self.scan_mtr_stop()
        return data

    def close(self):
        ser0.close()
        ser1.close()
        i2cbus.close()

if __name__ == "__main__":
    car = OmniCar()
    while True:
        print(car.heading())
        time.sleep(1)
