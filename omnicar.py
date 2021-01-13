"""This program accesses all the motors & sensors of the omni wheel car.

RasPi's only serial port used for bi-directional communication w/ Arduino.
 * All 5 motors are driven through 2 Adafruit Arduino motor shields.
 * HC-S04 sonar sensors are attached to the Arduino.
 * scan rotor angle encoder data via ADC on RasPi I2C bus
 * HMC5883L compass on RasPi I2C bus
 * TFminiPlus lidar data attached to FT232 USB serial device 

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

LEV = 10000  # Low Encoder Value (car -X direction)
MEV = 20000  #19680  # Mid Encoder Value (car +Y direction)
HEV = 30000  #29360  # High Encoder Value (car +X direction)

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1  #ADC gain

#  For bi-directional communication with Arduino
ports = ['/dev/ttyACM0', '/dev/ttyACM1']
for port in ports:
    if os.path.exists(port):
        ser0 = serial.Serial(port, 9600, timeout=0.5)
        #ser0.flush()
        break
logger.info(f"Serial port = {port}")

# For access to lidar distance sensor
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
    """Control motors & access sensors of omni-wheel car."""

    def __init__(self):
        """Configure HMC5883L (compass) & store most recent lidar value."""

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
        
    def _read_raw_data(self, addr):
        """Read raw data from HMC5883L data registers."""

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
        x = self._read_raw_data(X_AXIS_H)
        z = self._read_raw_data(Z_AXIS_H)
        y = self._read_raw_data(Y_AXIS_H)
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

    def _read_serial_data(self):
        """Read and return one line from serial port"""
        in_string = None
        while not in_string:
            if ser0.in_waiting:
                in_bytes = ser0.readline()
                in_string = in_bytes.decode("utf-8").rstrip()
        return in_string


    def _xfer_data(self, send_data):
        """Xfer data to Arduino: Send motor spd values, get sensor data

        send_data is a tuple of 6 integers: (flag, m1, m2, m3, m4, m5)
        flag = 0 (ignore motor values, just get sensor data)
        flag = 1 (apply motor values m1 thru m4 and get sensor data)
        flag = 2 (apply motor value m5 and get sensor data)
        """
        # convert integers to strings for sending on serial
        send_data_str = (str(item) for item in send_data)
        logger.debug(f"Data to send: {send_data}")
        out_string = ",".join(send_data_str)
        out_string += '\n'
        logger.debug(f"String being sent: {out_string}")
        ser0.write(out_string.encode())
        #ser0.flush()
        time.sleep(.2)  # wait for incoming sensor data
        #ser0.reset_input_buffer()
        snsr_str = 'No sensor data'
        snsr_str = self._read_serial_data()
        logger.debug(f"serial data read: {snsr_str}")
        distances = [int(item) for item in snsr_str.split(',')]
        return distances

    def get_sensor_data(self):
        """Get sensor data from Arduino without affecting motors.
        Return distances: [front_dist, left_dist, right_dist]"""
        distances = self._xfer_data((0, 0, 0, 0, 0, 0))
        return distances

    def go(self, speed, theta, spin=0):
        """Drive at speed (int) in relative direction theta (rad)
        while simultaneoulsy spinning CCW at rate = spin (int).
        Return distances: [front_dist, left_dist, right_dist]"""

        # convert from polar coordinates to omni_car's 'natural' coords
        # where one coaxial pair of wheels drives u and the other drives v
        u, v = convert_polar_to_rect(speed, theta - math.pi/4)

        # motor numbers
        m1, m2, m3, m4 = (1, 2, 3, 4)

        # combine motor speed with spin
        m1spd = int(spin + u)
        m2spd = int(spin - u)
        m3spd = int(spin - v)
        m4spd = int(spin + v)

        # friction keeps motors from running smoothly at speeds below ~50
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

        distances = self._xfer_data((1, m1spd, m2spd, m3spd, m4spd, 0))
        return distances
    
    def spin(self, spd):
        """Spin car (about its own axis) at spd (int) (CCW = +)
        Return distances: [front_dist, left_dist, right_dist]"""
        distances = self._xfer_data((1, spd, spd, spd, spd, 0))
        return distances

    def stop_wheels(self):
        """Stop all wheel motors (mtr numbers: 1 thrugh 4).
        Return distances: [front_dist, left_dist, right_dist]"""
        distances = self._xfer_data((1, 0, 0, 0, 0, 0))
        return distances

    def read_dist(self):
        """Set self.distance = distance (cm) read from lidar module.
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

    def scan(self, spd=200, lev=LEV, hev=HEV):
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
            if lev < enc_val < hev:
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
