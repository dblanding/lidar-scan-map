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
from pprint import pprint
import serial
import smbus
import sys
import time
import board
import busio
import adafruit_mpu6050
import Adafruit_ADS1x15
from adafruit_bno08x_rvc import BNO08x_RVC
from constants import *
import geom_utils as geo

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

# 16 bit analog to digital converter (for encoder values)
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1  #ADC gain

# MPU6050 Inertial Measurement Unit (for z-axis gyro data)
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c, 0x69)

# Adafruit BNO085 IMU
uart = serial.Serial("/dev/serial0", 115200)
rvc = BNO08x_RVC(uart)

#  For bi-directional communication with Arduino
ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyS0']
for port in ports:
    if os.path.exists(port):
        ser0 = serial.Serial(port, 9600, timeout=0.5)
        #ser1 = serial.Serial(port, 115200)
        break
    else:
        port = None
logger.info(f"Serial port = {port}")

# For access to lidar distance sensor
usbports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
for usbport in usbports:
    if os.path.exists(usbport):
        #ser0 = serial.Serial(usbport, 9600, timeout=0.5)
        ser1 = serial.Serial(usbport, 115200)
        break
    else:
        usbport = None
logger.info(f"USB_Serial port = {usbport}")

i2cbus = smbus.SMBus(1)
HMC5883_ADDRESS = 0x1e   # 0x3c >> 1
# HMC5883L Registers and their Address
REGISTER_A = 0x00  # Address of Configuration register A
REGISTER_B = 0x01  # Address of configuration register B
REGISTER_MODE = 0x02  # Address of mode register
X_AXIS_H = 0x03  # Address of X-axis MSB data register
Z_AXIS_H = 0x05  # Address of Z-axis MSB data register
Y_AXIS_H = 0x07  # Address of Y-axis MSB data register


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
        self.points = None  # scan data (list of dictionaries)
        self.target = None  # x, y coords of target point to drive to
        
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

    def mag_heading(self):
        """Return magnetic compass heading of car (degrees)."""

        # Read raw value
        x = self._read_raw_data(X_AXIS_H)
        z = self._read_raw_data(Z_AXIS_H)
        y = self._read_raw_data(Y_AXIS_H)
        # print(x, y, z)
        # working in radians...
        heading = math.atan2(y, x)

        # convert angle into degrees
        hdg = int(heading * 180 / math.pi)

        # calibration worked out experimentally (in degrees)
        C1 = 30
        C2 = 164
        C3 = 93
        cos_term = C1 * math.cos((hdg*math.pi/180) - C2*math.pi/180)
        hdg = int(hdg - cos_term - C3)

        # check for sign
        if hdg < 0:
            hdg += 360

        return hdg

    def heading(self):
        """Return heading (gyro yaw) measured by BNO085 IMU (degrees).

        BNO085 sends data @ 100 reading/sec on uart in RVC mode.
        yaw has a range of +/- 180˚ and is provided in 0.01˚ increments.
        """

        uart.reset_input_buffer()  # purge stale data
        yaw, *_ = rvc.heading
        return yaw

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

    def go(self, speed, angle, spin=0):
        """Drive at speed (int) in relative direction angle (degrees)
        while simultaneoulsy spinning CCW at rate = spin (int).
        Return distances: [front_dist, left_dist, right_dist]"""

        # convert from polar coordinates to omni_car's 'natural' coords
        # where one coaxial pair of wheels drives u and the other drives v
        u, v = geo.p2r(speed, angle - 45)

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

    def get_gyro_data(self):
        """Return gyro data in radians/sec."""
        return mpu.gyro

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
        Put target at mid-angle at radius/2.
        Convert to (x, y) coords and save as self.target
        so that map can access it.
        """
        # Find average (non-zero) dist value
        rvals = [point.get('dist')
                 for point in self.points
                 if point.get('dist') != 3]
        avgdist = sum(rvals)/len(rvals)

        # make radius somewhat larger
        radius = avgdist * 1.5
        print(f"radius: {int(radius)}")
        sectors = self.open_sectors(radius)
        print(sectors)

        # Find first sector of reaonable width
        for sector in sectors:
            angle0, angle1 = sector
            if (angle0 - angle1) > 12:
                target_angle = (angle0 + angle1)/2
                target_coords = geo.p2r(radius*.7, target_angle)
                self.target = target_coords
                break

    def close(self):
        ser0.close()
        ser1.close()
        i2cbus.close()


class PID():
    """
    Closed loop compass feedback used to adjust steering trim underway.
    """

    def __init__(self, target):
        """ Instantiate PID object before entering loop.

        target is the target magnetic compass heading in degrees."""
        """Turn (while stopped) toward target course (degrees)."""
        target = normalize_angle(target)
        self.target = target
        self.prev_error = 0
        self.trimval = PIDTRIM
        # create a rolling list of previous error values
        self.initial = 1
        self.rwl = [self.initial] * PIDWIN  # rolling window list

    def _get_integral_error(self, hdg_err):
        """Return rolling average error."""
        self.rwl.insert(0, hdg_err)
        self.rwl.pop()
        return sum(self.rwl) / len(self.rwl)

    def trim(self):
        """Return trim value to steer toward target."""
        # To avoid the complication of the 360 / 0 transition,
        # convert problem to one of aiming for a target at 180 degrees.
        heading_error = relative_bearing(self.target) - 180
        p_term = heading_error * KP
        i_term = self._get_integral_error(heading_error) * KI
        d_term = (heading_error - self.prev_error) * KD
        self.prev_error = heading_error
        adjustment = int((p_term + i_term + d_term))
        self.trimval += adjustment
        pstr = f"P: {p_term:.2f}\t"
        istr = f"I: {i_term:.2f}\t"
        dstr = f"D: {d_term:.2f}\t"
        tstr = f"Trim: {self.trimval:.2f}\t"
        hstr = f"HDG-Error: {heading_error:.0f}"
        logger.debug(pstr + istr + dstr + tstr + hstr)
        return self.trimval


def get_rate(speed):
    """Return rate (cm/sec) for driving FWD @ speed.

    Determined empirically for carspeed = 200, batt_charge >= 94%
    and distances from 50 - 200 cm.
    """
    return speed*0.155 - 6.5

def pid_steer_test(n=50):
    """
    Test PID steering operation over n cycles through feedback loop. 

    Set logging level to DEBUG to see PID values printed out.
    Allow space for car to drive FWD a few feet.
    """
    hdg = car.heading()
    pid = PID(hdg)
    while n:
        car.go(CARSPEED, FWD, spin=pid.trim())
        n -= 1
    car.stop_wheels()

def drive_ahead_w_compass_fb(dist, spd=CARSPEED):
    """Drive dist and stop using compass feedback for steering trim."""
    # instantiate PID steering
    target = int(car.heading())
    pid = PID(target)

    # drive
    rate = get_rate(spd)  # cm/sec
    time_to_travel = dist / rate
    start = time.time()
    delta_t = 0
    trimlist = []  # for debugging
    while delta_t < time_to_travel:
        delta_t = time.time() - start
        trim = pid.trim()
        trimlist.append(trim)  # for debugging
        sonardist, *_ = car.go(spd, FWD, spin=trim)
        if sonardist < SONAR_STOP:
            print("Bumped into an obstacle!")
            car.stop_wheels()
            break
    avg_trim = sum(trimlist) / len(trimlist)  # for debugging
    logger.debug(f"Average trim = {avg_trim}")  # for debugging
    logger.debug("trimlist:")  # for debugging
    logger.debug(trimlist)  #for debugging
    car.stop_wheels()

def drive_ahead(dist, spd=CARSPEED):
    """Drive dist and stop, w/out closed-loop steering feedback."""
    # drive
    rate = get_rate(spd)  # cm/sec
    time_to_travel = dist / rate
    start = time.time()
    delta_t = 0
    while delta_t < time_to_travel:
        delta_t = time.time() - start
        trim = PIDTRIM
        sonardist, *_ = car.go(spd, FWD, spin=trim)
        if sonardist < SONAR_STOP:
            print("Bumped into an obstacle!")
            car.stop_wheels()
            break
    car.stop_wheels()

def normalize_angle(angle):
    """Convert any value of angle to a value between 0-360."""
    while angle < 0:
        angle += 360
    while angle > 360:
        angle -= 360
    return angle

def relative_bearing(target):
    """Return 'relative' bearing of an 'absolute' target."""
    delta = target - 180
    #logger.debug(f"heading: {car.heading()}")
    rel_brng = int(car.heading() - delta)
    return normalize_angle(rel_brng)

def turn_to(target):
    """Turn (while stopped) to absolute target course (degrees)."""
    target = normalize_angle(target)
    # To avoid the complication of the 360 / 0 transition,
    # convert problem to one of aiming for a target at 180 degrees.
    heading_error = relative_bearing(target) - 180
    logger.debug(f"relative heading: {heading_error} deg")
    done = False
    while not done:
        while heading_error > 2:
            spd = heading_error + 40
            car.spin(spd)
            heading_error = relative_bearing(target) - 180
            logger.debug(f"relative heading: {heading_error} deg")
            time.sleep(0.1)
        car.stop_wheels()
        while heading_error < -2:
            spd = heading_error - 40
            car.spin(spd)
            heading_error = relative_bearing(target) - 180
            logger.debug(f"heading error: {heading_error} deg")
            time.sleep(0.1)
        car.stop_wheels()
        if -3 < abs(heading_error) < 3:
            print("done")
            done = True

def encoder_count_to_radians(enc_cnt):
    """
    Convert encoder count to angle (radians) in car coordinate system

    encoder_count values start at 0 and increase with CW rotation.
    straight back (-Y axis): enc_cnt = 0; theta = 3*pi/2
    straight left (-X axis): enc_cnt = 10,000; theta = pi
    straight ahead (+Y axis): enc_cnt = 20,000; theta = pi/2
    straight right (+X axis): enc_cnt = 30,000; theta = 0
    (enc_cnt tops out at 32765, so no info past that)
    """
    theta = (30000 - enc_cnt) * math.pi / (30000 - 10000)
    return theta

def drive_to_spot(spd=None):
    """
    Scan & display interactive map with proposed target spot shown.
    User then closes map and either enters 'y' to agree to proposed
    spot or 'c' to input coordinates of an alternate one.
    Car drives to spot. Repeat.
    """
    if not spd:
        spd = CARSPEED
    nmbr = 0
    while nmbr < 10:
        # scan & display plot
        car.scan()
        car.auto_detect_open_sector()
        coords = car.target
        '''
        car.map(seq_nmbr=nmbr, show=True)
        
        # get coords from user
        msg = "enter Y to go to yellow dot; C to enter coords; Q to quit: "
        char = input(msg)
        if char in 'yY':
            coords = car.target
        elif char in 'cC':
            coordstr = input("Enter x, y coords: ")
            if ',' in coordstr:
                xstr, ystr = coordstr.split(',')
                x = int(xstr)
                y = int(ystr)
                coords = (x, y)
        else:
            break
        '''
        # convert x,y to r, theta then drive
        r, theta = geo.r2p(coords)
        target_angle = int(theta - 90)
        print(f"Turning {target_angle} degrees")
        turn_to(car.heading()-target_angle)
        print(f"Driving {r:.1f} cm")
        drive_ahead(r, spd=spd)
        nmbr += 1


if __name__ == "__main__":
    car = OmniCar()
    time.sleep(0.5)
    from_arduino = car._read_serial_data()
    logger.debug(f"Message from Arduino: {from_arduino}")
    while True:
        print("")
        print(f"Magnetic Heading = {car.mag_heading()}")
        print(f"BNO085 Gyro Heading = {car.heading()}")
        time.sleep(1)

