"""Collection of car operation commands
"""
import logging
import math
import pickle
import sys
import time
import omnicar
import proscan
from pprint import pprint

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

car = omnicar.OmniCar()
time.sleep(0.5)
from_arduino = car._read_serial_data()
logger.debug(f"Message from Arduino: {from_arduino}")

W2W_DIST = 34  # separation between coaxial wheels (cm)
PAUSE = 0.05  # sec PAUSE time needed between wheel motor commands
CLEARANCE = 30  # threshold min clearance to wall (cm)
MIN_LENGTH = 20  # threshold min length to end of wall (cm)
MIN_DIST = 40  # threshold min distance from car to wall (cm)
KP = 0.25  # steering PID proportional coefficient
KD = 0.3  # steering PID derivative coefficient
CARSPEED = 150  # default car speed
EOW = 10  # Threshold End of Wall

class PID():
    """Adjust trim to maintain heading using pid feedback from compass."""

    def __init__(self, target):
        self.target = target
        self.prev_error = 0
        self.trimval = 3

    def trim(self):
        """Return value for spin"""
        hdg = car.heading()
        heading_error = hdg - self.target
        p_term = heading_error
        d_term = heading_error - self.prev_error
        self.prev_error = heading_error
        adjustment = int((p_term * KP + d_term * KD))
        self.trimval += adjustment
        logger.debug(f"P-term: {p_term:.2f}\tD-term: {d_term:.2f}\tTrim: {self.trimval}\tHDG: {int(hdg)}")
        return self.trimval


def normalize_angle(angle):
    """convert any value of angle to a value between 0-360."""
    while angle < 0:
        angle += 360
    while angle > 360:
        angle -= 360
    return angle

def relative_bearing(target):
    """Return 'relative' bearing of an 'absolute' target."""
    delta = target - 180
    logger.debug(f"heading: {car.heading()}")
    rel_brng = int(car.heading() - delta)
    return normalize_angle(rel_brng)

def turn_in_place(target):
    """Turn (while stopped) toward target course (degrees)."""
    target = normalize_angle(target)
    # To avoid the complication of the 360 / 0 transition,
    # convert problem to one of aiming for a target at 180 degrees.
    heading_error = relative_bearing(target) - 180
    logger.debug(f"relative heading: {heading_error} deg")
    while not -2 <= heading_error <= 2:
        while heading_error > 2:
            car.spin(50)
            time.sleep(0.1)
            car.stop_wheels()
            heading_error = relative_bearing(target) - 180
            logger.debug(f"relative heading: {heading_error} deg")
            time.sleep(0.2)
        while heading_error < -2:
            car.spin(-50)
            time.sleep(0.1)
            car.stop_wheels()
            heading_error = relative_bearing(target) - 180
            logger.debug(f"heading error: {heading_error} deg")
            time.sleep(0.2)

def _turn_on_the_go(target, direction, spin_ratio):
    """Turn toward target course (degrees) while going in direction.
    (For example, direction = math.pi/2 for straight ahead.)"""
    target = normalize_angle(target)
    trim = CARSPEED * spin_ratio
    # To avoid the complication of the 360 / 0 transition,
    # convert problem to one of aiming for a target at 180 degrees.
    heading_error = relative_bearing(target) - 180
    if heading_error > 0:
        car.go(CARSPEED, direction, spin=trim)
        while heading_error > 0:
            logger.debug(f"relative heading: {int(heading_error)}")
            heading_error = relative_bearing(target) - 180
            time.sleep(0.1)
    elif heading_error < 0:
        car.go(CARSPEED, direction, spin=trim)
        while heading_error < 0:
            logger.debug(f"heading error: {int(heading_error)}")
            heading_error = relative_bearing(target) - 180
            time.sleep(0.1)
    car.stop_wheels()

def radius_turn_on_the_go(direction, angle, turn_radius):
    """Turn car angle degrees (CCW positive) at turn_radius (cm)
    while underway in direction (eg: 0 or pi/2)."""
    spin_ratio = (W2W_DIST / 2 / turn_radius) / math.sqrt(2)
    trim = CARSPEED * spin_ratio
    target = car.heading() - angle
    if target < 0:
        target += 360
    _turn_on_the_go(target, direction, spin_ratio)

def r2p(xy_coords):
    """Convert rect coords (x, y) to polar (r, theta)
    with theta in degrees."""
    x, y = xy_coords
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x) * 180 / math.pi
    return (r, theta)

def print_line_params(line_params):
    for item in line_params:
        coords, length, angle, distance = item
        p1_xy_coords, p2_xy_coords = coords
        p1_rp_coords = r2p(p1_xy_coords)
        p2_rp_coords = r2p(p2_xy_coords)
        print(f"P1 rect coords: ({int(p1_xy_coords[0]):.2f}, {int(p1_xy_coords[1]):.2f})")
        print(f"P2 rect coords: ({int(p2_xy_coords[0]):.2f}, {int(p2_xy_coords[1]):.2f})")
        print(f"P1 polar coords: ({int(p1_rp_coords[0]):.2f}, {p1_rp_coords[1]:.2f})")
        print(f"P2 polar coords: ({int(p2_rp_coords[0]):.2f}, {p2_rp_coords[1]:.2f})")
        print(f"length: {length:.2f} cm")
        print(f"angle: {angle:.2f} deg")
        print(f"distance: {distance:.2f} cm")
        print()

def square_to_wall(mapping=True):
    """Spin car to square to wall that is roughly in front."""

    # get scan line(s)
    data = car.scan(lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data)
    line_params = pscan.get_line_parameters()
    print(len(data))
    print(f"line parameters: {line_params}")
    # Examine first line found
    coords, length, angle, dist = line_params[0]

    # Examine first line found
    coords, length, angle, dist = line_params[0]
    logger.debug(f"angle = {angle} degrees")
    logger.debug(f"heading = {car.heading()} degrees")
    target = car.heading() - angle
    logger.debug(f"target = {target} degrees")
    
    # display and save initial map
    if mapping:
        pscan.map(nmbr=1, display_all_points=True)

    # Turn in place
    turn_in_place(target)
    logger.debug(f"heading = {car.heading()} deg")
    
def longest_line(line_params):
    max_len = 0
    idx = 0
    for n, line in enumerate(line_params):
        coords, length, angle, dist = line
        if length > max_len:
            idx = n
            max_len = length
    return line_params[idx]

def approach_wall(carspeed, clearance, mapping=False):
    """Approach wall squarely at carspeed until distance to wall < clearance.
    trim course to maintain current compass heading."""

    # get scan line(s)
    data = car.scan(lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data)
    line_params = pscan.get_line_parameters()
    coords, length, angle, dist = longest_line(line_params)

    # display and save initial map
    if mapping:
        pscan.map(nmbr=1, display_all_points=True)

    # OK to proceed?
    if dist > clearance:
        dist, *rest = car.go(carspeed, math.pi/2)
        prev_dist = dist
        sustained_dist = (dist + prev_dist) / 2        

    # instantiate PID steering
    target_heading = car.heading()
    pid = PID(target_heading)
    logger.debug("")
    logger.debug(f"Approach wall to dist < {clearance}, target heading = {int(target_heading)}")

    # continue toward wall
    while sustained_dist > clearance:  # sonar occasionally gives false low values
        dist, *rest = car.go(carspeed, math.pi/2, spin=pid.trim())
        logger.debug(f"Dist: {int(dist)}")
        sustained_dist = (dist + prev_dist) / 2
        prev_dist = dist

    car.stop_wheels()

def drive_along_wall_to_right(carspeed, clearance, mapping=False):
    """Drive to right maintaining clearance to wall in front. Stop at end.
    Return dist value at end."""

    # get scan line(s)
    data = car.scan(lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data)
    line_params = pscan.get_line_parameters()
    coords, length, angle, dist = longest_line(line_params)
    end_of_wall = coords[-1][0]  # x coordinate of right end of wall

    # display and save initial map
    if mapping:
        pscan.map(nmbr=2, display_all_points=True)

    # OK to proceed?
    if end_of_wall > EOW:
        car.go(carspeed, 0)

    # initialize pid steering
    target_heading = car.heading()
    pid = PID(target_heading)
    logger.debug("")
    logger.debug(f"Drive right until end of wall < {EOW}, target heading = {int(target_heading)}")

    # continue to end of wall
    while end_of_wall > EOW:
        # get scan line(s)
        data = car.scan(lev=17500, hev=22500)
        pscan = proscan.ProcessScan(data)
        line_params = pscan.get_line_parameters()
        coords, length, angle, dist = longest_line(line_params)
        end_of_wall = coords[-1][0]  # x coordinate of right end of wall
        car.go(carspeed, 0, spin=pid.trim())
        logger.debug(f"Dist: {int(dist)}\tAngle: {angle:.2f}\tEOW: {end_of_wall:.2f}")

    car.stop_wheels()

    # display and save final map
    if mapping:
        pscan.map(nmbr=3, display_all_points=False)
    return dist

def round_corner(speed, turn_radius):
    logger.debug("")
    logger.debug(f"turning left: corner R = {int(turn_radius)}")
    # To avoid the complication of the 360 / 0 transition,
    # convert problem to one of aiming for a target at 180 degrees.
    rel_heading = relative_bearing(target)

    start_heading = car.heading()
    spin_ratio = (W2W_DIST / 2 / turn_radius) / math.sqrt(2)
    logger.debug("")
    logger.debug(f"spin_ratio = {spin_ratio:.2f}")
    trim = speed * spin_ratio
    car.go(speed, 0, spin=trim)
    car.stop_wheels()

def save_scandata_as_csv(data, filename):
    write_data = []
    for line in data:
        ev, dist, _, _ = line
        str_to_write = ','.join((str(ev), str(dist))) + '\n'
        write_data.append(str_to_write)
    with open(filename, 'w') as f:
        f.writelines(write_data)

def save_scan(nmbr=None):
    if nmbr is None:
        nmbr = ''
    data = car.scan()
    with open(f'scan_data{nmbr}.pkl', 'wb') as f:
        pickle.dump(data, f)
    print(f"total number of points: {len(data)}")
    save_scandata_as_csv(data, f'scan_data{nmbr}.csv')
    pscan = proscan.ProcessScan(data)
    pscan.map(nmbr=nmbr, display_all_points=True)

def find_std_dev(datalist):
    # Standard deviation of list 
    # Using sum() + list comprehension 
    mean = sum(datalist) / len(datalist) 
    variance = sum([((x - mean) ** 2) for x in datalist]) / len(datalist) 
    std_dev = variance ** 0.5
    return mean, std_dev

def find_outliers(datalist):
    mean, std_dev = find_std_dev(datalist)
    upper_threshold = mean + (3 * std_dev)
    outliers = [value for value in datalist if value > upper_threshold]
    return outliers

def find_clear_path(data, thresh=None):
    """Find widest open sector. Return rel heading and distance to go.

    Do this by finding all contiguous regions with dist values
    equal to zero or greater than thresh * max_dist.
    """
    if not thresh:
        thresh = 0.6

    # find max_dist
    max_dist = 0
    distlist = []  # list of dist values
    for record in data:
        enc_val, dist, _, _ = record
        if dist > max_dist:
            max_dist = dist
        distlist.append(dist)
    outliers = find_outliers(distlist)
    print(f"outliers: {outliers}")

    # generate overlay list, with 1 for open, 0 for not open
    overlay = []
    for record in data:
        enc_val, dist, _, _ = record
        if dist >= max_dist * thresh or dist == 0:
            overlay.append(1)
        else:
            overlay.append(0)

    # look for 'clumps' of contiguous open indexes
    open_clumps = []  # (start index, end_index)
    curr_state = False  # True is open
    for n, val in enumerate(overlay):
        if not curr_state and val:  # start of an open region
            curr_state = True
            start_index = n
        elif curr_state and not val:  # end of an open region
            curr_state = False
            open_clumps.append((start_index, n))
    print(f"open_clumps: {open_clumps}")

    # find largest clump
    maxlen = 0
    widest = None
    for n, clump in enumerate(open_clumps):
        length = clump[-1] - clump[0]
        if length > maxlen:
            maxlen = length
            widest = clump

    # find enc_val if center of clump, dist to drive
    start_record = data[widest[0]]
    end_record = data[widest[-1]]
    start_enc_val, start_dist, _, _ = start_record
    end_enc_val, end_dist, _, _ = end_record
    avg_enc_val = (start_enc_val + end_enc_val) / 2
    stop_dist = (start_dist + end_dist) / 2
    print(avg_enc_val, stop_dist)

    # convert enc_val to relative heading (straight ahead = 0 deg)
    rel_heading = (avg_enc_val - omnicar.MEV) * 90 / (omnicar.MEV - omnicar.LEV)
    return (rel_heading, stop_dist)

if __name__ == "__main__":
    
    save_scan(nmbr=1)
    heading = car.heading()
    pid = PID(heading)
    print(f"Driving 10 seconds at {CARSPEED} in direction {heading}")
    start_time = time.time()
    while time.time()-start_time < 10:
        dist, *rest = car.go(CARSPEED, math.pi/2, spin=pid.trim())
        print(f"Front_dist = {dist}")
    car.stop_wheels()
    
    save_scan(nmbr=2)
    '''
    square_to_wall()
    
    dist = approach_wall(CARSPEED, CLEARANCE)
    
    dist = drive_along_wall_to_right(CARSPEED, CLEARANCE, mapping=True)
    
    radius_turn_on_the_go(0, 90, CLEARANCE)
    
    #data = car.go(100, 0)
    data = car.scan()
    print(f"'Sensor data' returned from car.go() command: {data}")
    time.sleep(1)
    data = car.stop_wheels()
    print(f"'Sensor data' returned from car.stop_wheels() command: {data}")
    
    while True:
        print(f"Car heading = {car.heading()}")
        data = car.scan()
        print(f"total number of points: {len(data)}")
        save_scandata_as_csv(data, 'scan_data.csv')
        rel_heading, dist_to_go = find_clear_path(data)
        print(f"relative heading: {rel_heading}, distance to drive: {dist_to_go}")
        pscan = proscan.ProcessScan(data, gap=15)
        pscan.map(nmbr=1, display_all_points=True)
        turn_in_place(car.heading() + rel_heading)
        car.go(100, math.pi/2, spin=5)
        # goes 64 inches (162 cm) in 10 sec @ spd = 150
        time.sleep(dist_to_go / 16)
        car.stop_wheels()
    '''
    car.close()
