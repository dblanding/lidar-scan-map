"""This program accesses all the motors & sensors of the omni wheel car.

RasPi's USB serial port used to send motor drive cammands to Arduino.
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
from constants import LEV, HEV, VLEG, PIDTRIM
import geom_utils as geo
import pathfwd

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

SECTOR_WIDTH = 18  # Minimum value for open sectors
RADIUS_FACTOR = 1.2  # Used when finding open sectors

# Adafruit BNO085 IMU
imuport = "/dev/serial0"
ser0 = serial.Serial(imuport, 115200, timeout=0.1)
logger.info(f"BNO085 IMU UART port = {imuport}")
rvc = BNO08x_RVC(ser0)

# Communication with Arduino
ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyS0']
for port in ports:
    if os.path.exists(port):
        ser1 = serial.Serial(port, 9600, timeout=0.5)
        break
    else:
        port = None
logger.info(f"Arduino communication port = {port}")

# Lidar distance sensor
lidarport = '/dev/ttyUSB0'
ser2 = serial.Serial(lidarport, 115200)
logger.info(f"Lidar port = {lidarport}")

# 16 bit ADC (for odometer & lidar rotor encoder values)
i2cbus = smbus.SMBus(1)
adc = Adafruit_ADS1x15.ADS1115()

# GPIO pin setup
HDG_RESET_PIN = 18  # Broadcom pin 18 (Pi pin 12)
ODO_RESET_PIN = 24  # Broadcom pin 24 (Pi pin 18)
GPIO.setmode(GPIO.BCM)
GPIO.setup(HDG_RESET_PIN, GPIO.OUT)
GPIO.setup(ODO_RESET_PIN, GPIO.OUT)
GPIO.output(HDG_RESET_PIN, GPIO.HIGH)
GPIO.output(ODO_RESET_PIN, GPIO.HIGH)

def encoder_count_to_radians(enc_cnt):
    """
    Convert encoder count to angle (radians) in car coordinate system

    Lidar rotor angle encoder 0-5 V -> 0-2*pi radians
    encoder_count values start at 0 and increase with CW rotation.
    straight ahead (+X axis): enc_cnt = 20,000; theta = 0
    straight back (-X axis): enc_cnt = 0; theta = pi
    straight left (+Y axis): enc_cnt = 10,000; theta = pi/2
    straight right (-Y axis): enc_cnt = 30,000; theta = -pi/2
    (enc_cnt tops out at 32765, so no info past that)
    """
    theta = math.pi * (20000 - enc_cnt) / (20000)
    return theta


class OmniCar():
    """Control motors & access sensors of omni-wheel car.

    Also produce rotating scan with paired distance, angle values.
    """

    def __init__(self):
        self.distance = 0  # last measured distance (cm) from lidar
        self.points = None  # scan data (list of dictionaries)
        self.ODOMETER_OFFSET = 136
        

    @property
    def odometer(self):
        """Return odometer value in cm."""
        counts = adc.read_adc(1, gain=1, data_rate=250)
        # Formula below found empirically
        return int((counts - self.ODOMETER_OFFSET) * 0.0106)

    def reset_odometer(self):
        """Reset odometer to 0 cm."""
        # Ground the reset pin.
        GPIO.output(ODO_RESET_PIN, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(ODO_RESET_PIN, GPIO.HIGH)
        time.sleep(0.5)  # Wait before first reading
        test_val = adc.read_adc(1, gain=1, data_rate=250)
        readings = []
        for n in range(10):
            readings.append(adc.read_adc(1, gain=1, data_rate=250))
        avg_count = int(sum(readings) / len(readings))
        self.ODOMETER_OFFSET = avg_count
        logger.debug("Odometer reset to 0 cm")

    @property
    def heading(self):
        """Return instantaneous heading (gyro yaw) in degrees.

        BNO085 sends data @ 100 reading/sec on ser0 in RVC mode.
        yaw has a range of +/- 180˚ in 0.01˚ increments.
        """
        ser0.reset_input_buffer()  # purge stale data
        yaw, *_ = rvc.heading
        return yaw

    def reset_heading(self):
        """Reset heading to 0 degrees."""
        GPIO.output(HDG_RESET_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(HDG_RESET_PIN, GPIO.HIGH)
        time.sleep(0.5)  # to avoid read errors
        logger.debug("Heading reset to 0 degrees")
    
    def _read_serial_data(self):
        """Read and return one line from serial port"""
        in_string = None
        while not in_string:
            if ser1.in_waiting:
                in_bytes = ser1.readline()
                in_string = in_bytes.decode("utf-8").rstrip()
        return in_string

    def _xfer_data(self, send_data):
        """Xfer data to Arduino: Send motor spd values, get acknowledge
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
        ser1.write(out_string.encode())
        # Receive acknowledgement
        snsr_str = self._read_serial_data()
        if not snsr_str:
            snsr_str = 'No sensor data'
        logger.debug(f"serial data read: {snsr_str}")
        return snsr_str

    def go(self, speed, angle, spin=0):
        """Drive at speed (int) in relative direction angle (degrees)
        while simultaneoulsy spinning CCW at rate = spin (int)."""

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
            bytes_serial = ser2.read(1)
            if bytes_serial[0] == 0x59 and not first:
                first = True
            elif bytes_serial[0] == 0x59 and first:
                _ = ser2.read(7)
                n += 7
                break
        logger.debug(f"Number of bytes read to resync = {n}")

    def read_dist(self):
        """Read lidar module distance (cm), update self.distance
        Return number of bytes waiting on serial port when read.
        """
        # Prior to first read, purge buildup of 'stale' data
        if ser2.in_waiting > 100:
            ser2.reset_input_buffer()

        # Wait for serial port to accumulate 9 bytes of 'fresh' data
        while ser2.in_waiting < 9:
            time.sleep(.0005)

        # Now read 9 bytes of data on serial port
        counter = ser2.in_waiting
        bytes_serial = ser2.read(9)
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
        """Return encoder value from LiDAR rotor angle encoder."""
        return adc.read_adc(0, gain=1, data_rate=250)

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

    def next_target_point(self):
        """Find next target point by analyzing scan data for openings."""
        # Find average (non-zero) dist value
        rvals = [point.get('dist')
                 for point in self.points
                 if point.get('dist') != -VLEG]
        avgdist = int(sum(rvals)/len(rvals))
        dist = avgdist
        points = [point.get('xy') for point in self.points
                  if point.get('dist') != -VLEG]
        mid_angles = pathfwd.best_paths(points, dist)
        # convert last angle in list to xy coords in car coord sys
        theta = mid_angles[-1] - 90
        target_pnt = geo.p2r(dist, theta)
        return target_pnt

    def close(self):
        ser0.close()
        ser1.close()
        ser2.close()
        i2cbus.close()
        GPIO.cleanup()

def test_hdg(nbr_loops):
    """Test heading data for nbr_loops times through loop
    at full speed through loop."""
    n = 0
    while n < nbr_loops:
        print(car.heading)
        n+= 1

def test_sensors(drive_dist):
    """Test sensors while driving FWD drive_dist (cm)
    at full speed through loop."""
    car.go(150, 0, spin=PIDTRIM)
    print()
    print("ODO\tLIDAR\tHDG")
    odo = car.odometer
    n = 0
    while odo < drive_dist:
        n += 1
        _ = car.read_dist()  # read lidar
        dist = car.distance  # lidar value just read
        hdg = car.heading
        odo = car.odometer
        print(f"{odo}\t{dist}\t{hdg}")
        
    car.stop_wheels()
    time.sleep(1)
    _ = car.read_dist()  # read lidar
    dist = car.distance  # lidar value just read
    hdg = car.heading
    odo = car.odometer
    print(f"Final Odoometer: {odo}")
    print(f"Final Lidar dist: {dist}")
    print(f"Final Heading: {hdg}")
    print(f"{n} times through loop")
    
if __name__ == "__main__":
    car = OmniCar()
    time.sleep(0.5)
    from_arduino = car._read_serial_data()
    logger.info(f"Message from Arduino: {from_arduino}")
    car.reset_odometer()
    time.sleep(0.5)
    #test_hdg(100)
    test_sensors(10)
    car.close()
