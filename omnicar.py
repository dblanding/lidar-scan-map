"""This program accesses all the motors & sensors of the omni wheel car.

RasPi's USB serial port used for bi-directional communication w/ Arduino.
 * RasPi commands all motors, connected to Arduino's 2 motor shields.
 * HC-S04 sonar sensors, attached to the Arduino, send data back to RasPi.
BNO085 IMU attached on RasPi UART RX pin, sends heading data.
TFminiPlus lidar data attached to FT232 USB serial device.
Scan rotor angle encoder data via ADC on RasPi I2C bus.

See omni-wheels.md for an explanation of wheel motor drive.
"""
import logging
import math
import os
from pprint import pprint
import RPi.GPIO as GPIO
import serial
import smbus
import sys
import time
import Adafruit_ADS1x15
from adafruit_bno08x_rvc import BNO08x_RVC
from constants import *
import geom_utils as geo

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

# Adafruit BNO085 IMU
uart = serial.Serial("/dev/serial0", 115200)
rvc = BNO08x_RVC(uart)
RESET_PIN = 18  # Broadcom pin 18 (Pi pin 12)
GPIO.setmode(GPIO.BCM)
GPIO.setup(RESET_PIN, GPIO.OUT)
GPIO.output(RESET_PIN, GPIO.HIGH)

#  Bi-directional communication with Arduino
ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyS0']
for port in ports:
    if os.path.exists(port):
        ser0 = serial.Serial(port, 9600, timeout=0.5)
        #ser1 = serial.Serial(port, 115200)
        break
    else:
        port = None
logger.info(f"Serial port = {port}")

# Lidar distance sensor
usbports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
for usbport in usbports:
    if os.path.exists(usbport):
        #ser0 = serial.Serial(usbport, 9600, timeout=0.5)
        ser1 = serial.Serial(usbport, 115200)
        break
    else:
        usbport = None
logger.info(f"USB_Serial port = {usbport}")

# 16 bit ADC (for encoder values)
i2cbus = smbus.SMBus(1)
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1  #ADC gain


class OmniCar():
    """Control motors & access sensors of omni-wheel car.

    Also produce rotating scan with paired distance, angle values.
    """

    def __init__(self):
        self.distance = 0  # last measured distance (cm) from lidar
        self.points = None  # scan data (list of dictionaries)

    @property
    def heading(self):
        """Return instantaneous heading (gyro yaw) in degrees.

        BNO085 sends data @ 100 reading/sec on uart in RVC mode.
        yaw has a range of +/- 180˚ in 0.01˚ increments.
        """
        uart.reset_input_buffer()  # purge stale data
        yaw, *_ = rvc.heading
        return yaw

    def reset_heading(self):
        """Reset heading to 0 degrees."""
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(RESET_PIN, GPIO.HIGH)

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
        flag = 2 (apply motor value m5 and get sensor data)"""
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

    def go(self, speed, angle, spin=0):
        """Drive at speed (int) in relative direction angle (degrees)
        while simultaneoulsy spinning CCW at rate = spin (int).
        Return distances: [front_dist, left_dist, right_dist]"""

        # convert from polar coordinates to omni_car's 'natural' coords
        # where one coaxial pair of wheels drives u and the other drives v
        u, v = geo.p2r(speed, angle + 45)

        # motor numbers
        # m1, m2, m3, m4 = (1, 2, 3, 4)

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

    def resync(self):
        """
        Resync read with serial data by reading one byte at a time
        looking for a pair of start bytes, then reading next 7 bytes.
        """
        first = False
        n = 0
        while True:
            n += 1
            bytes_serial = ser1.read(1)
            if bytes_serial[0] == 0x59 and not first:
                first = True
            elif bytes_serial[0] == 0x59 and first:
                _ = ser1.read(7)
                n += 7
                break
        logger.debug(f"Number of bytes read to resync = {n}")

    def read_dist(self):
        """Read lidar module distance (cm), update self.distance
        Return number of bytes waiting on serial port when read.
        """
        # Prior to first read, purge buildup of 'stale' data
        if ser1.in_waiting > 100:
            ser1.reset_input_buffer()

        # Wait for serial port to accumulate 9 bytes of 'fresh' data
        while ser1.in_waiting < 9:
            time.sleep(.0005)

        # Now read 9 bytes of data on serial port
        counter = ser1.in_waiting
        bytes_serial = ser1.read(9)
        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
            distance = bytes_serial[2] + bytes_serial[3]*256
            # subtract module to mirror distance
            self.distance = distance - VLEG
            #self.strength = bytes_serial[4] + bytes_serial[5]*256
            #temperature = bytes_serial[6] + bytes_serial[7]*256
            #self.temperature = (temperature/8) - 256
        else:  # read out of sync with data
            logger.debug("LiDAR read being re-synced")
            self.resync()
        return counter

    def get_enc_val(self):
        """Return encoder value from LiDAR rotor angular encoder."""
        return adc.read_adc(0, gain=GAIN, data_rate=250)

    def scan_mtr_start(self, spd):
        """Turn scan motor on at speed = spd (int between 0-255)."""
        _ = self._xfer_data((2, 0, 0, 0, 0, spd))

    def scan_mtr_stop(self):
        """Turn scan motor off."""
        _ = self._xfer_data((2, 0, 0, 0, 0, 0))

    def scan(self, spd=150, lev=LEV, hev=HEV):
        """
        Perform one LiDAR scan. Return data as list of dictionaries.

        optional arguments:
            spd -> scan motor speed (100-255)
            lev (low encoder value) -> start of scan
            hev (high encoder value) -> end of scan

        data point dictionary -> keys: values
            'enc_cnt': integer between 0 and 32767
            'dist': distance (cm) read by LiDAR module
            'bytes': nmbr of bytes in input buffer at read time
            'delta_t': time since previous read (sec)
            'theta': angle (radians) in car coordinate system
            'xy': rect coords tuple (x, y) in car coord system
        """
        enc_val = self.get_enc_val()
        # If scan rotor isn't near BDC (back dead cntr), go to BDC
        if enc_val > 3000:
            self.scan_mtr_start(spd)
            while enc_val < 32767:  # continue as values increase to max
                enc_val = self.get_enc_val()
            while enc_val == 32767:  # continue to back dead cntr
                enc_val = self.get_enc_val()
        else:
            self.scan_mtr_start(spd)
            enc_val = self.get_enc_val()
        last_time = time.time()
        data = []
        while enc_val < 32767:  # continue as values increase to max
            dpd = {}  # data point dictionary
            enc_val = self.get_enc_val()
            counter = self.read_dist()
            if lev < enc_val < hev:
                now = time.time()
                delta_t = now - last_time
                last_time = now
                dpd['enc_cnt'] = enc_val
                dpd['bytes'] = counter
                dpd['delta_t'] = delta_t
                dpd['dist'] = self.distance
                theta = encoder_count_to_radians(enc_val)
                dpd['theta'] = theta
                x = self.distance * math.cos(theta)
                y = self.distance * math.sin(theta)
                dpd['xy'] = (x, y)
                data.append(dpd)
        self.scan_mtr_stop()
        self.points = data
        return data

    def open_sectors(self, radius):
        """Return list of sectors containing no points at dist < radius

        Each open sector is a 2-element tuple of bounding angles (deg)
        Sectors and angles are in scan order, so largest angles first.

        The idea is to look for a sector of sufficient width to allow
        the car through, then drive along the center of the sector.
        """
        sectors = []
        in_sector = False
        n = 0
        for pnt in self.points:
            n += 1
            dist = pnt.get("dist")
            if not in_sector:
                if dist < 0 or dist > radius:
                    start_angle = int(pnt.get("theta") * 180 / math.pi)
                    end_angle = start_angle
                    in_sector = True
            elif in_sector:
                if dist < 0 or dist > radius:
                    end_angle = int(pnt.get("theta") * 180 / math.pi)
                elif dist < radius:
                    in_sector = False
                    sector = (start_angle, end_angle)
                    sectors.append(sector)
        sector = (start_angle, end_angle)  # final sector?
        if sector not in set(sectors):
            sectors.append(sector)
        return sectors

    def auto_detect_open_sector(self):
        """ Under development...
        First find average dist value (not == -3).
        Then look for sectors at radius = 1.5 * average.
        Put target near mid-angle a little past radius/2.
        Convert to (x, y) coords and return it.
        """
        # Find average (non-zero) dist value
        rvals = [point.get('dist')
                 for point in self.points
                 if point.get('dist') != 3]
        avgdist = sum(rvals)/len(rvals)

        # make radius somewhat larger
        radius = avgdist * 1.5
        logger.debug(f"Sector detection radius: {int(radius)}")
        sectors = self.open_sectors(radius)
        logger.debug(f"Open sectors: {sectors}")

        # Find first sector of reaonable width
        for sector in sectors:
            angle0, angle1 = sector
            if (angle0 - angle1) > 12:
                target_angle = (angle0 + angle1) * 0.48
                target_pnt = geo.p2r(radius*.8, target_angle)
                break
        return target_pnt

    def close(self):
        ser0.close()
        ser1.close()
        i2cbus.close()
        GPIO.cleanup()


def encoder_count_to_radians(enc_cnt):
    """
    Convert encoder count to angle (radians) in car coordinate system

    encoder_count values start at 0 and increase with CW rotation.
    straight ahead (+X axis): enc_cnt = 20,000; theta = 0
    straight back (-X axis): enc_cnt = 0; theta = pi
    straight left (+Y axis): enc_cnt = 10,000; theta = pi/2
    straight right (-Y axis): enc_cnt = 30,000; theta = -pi/2
    (enc_cnt tops out at 32765, so no info past that)
    """
    theta = math.pi * (20000 - enc_cnt) / (20000)
    return theta

if __name__ == "__main__":
    car = OmniCar()
    time.sleep(0.5)
    from_arduino = car._read_serial_data()
    logger.debug(f"Message from Arduino: {from_arduino}")
    for _ in range(5):
        print("")
        print(f"BNO085 Gyro Heading = {car.heading()}")
        time.sleep(1)
    car.reset_heading()
    for _ in range(5):
        print("")
        print(f"BNO085 Gyro Heading = {car.heading()}")
        time.sleep(1)
    car.close()
