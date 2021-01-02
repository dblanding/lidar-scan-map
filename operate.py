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
CLEARANCE = 70  # threshold min clearance to wall (cm)
MIN_LENGTH = 20  # threshold min length to end of wall (cm)
MIN_DIST = 40  # threshold min distance from car to wall (cm)
KP = 0.25  # steering PID proportional coefficient
KD = 0.3  # steering PID derivative coefficient
CARSPEED = 120  # default car speed

def relative_bearing(target):
    """Return 'relative' bearing of an 'absolute' target."""
    delta = target - 180
    rel_brng = car.heading() - delta
    if rel_brng < 0:
        rel_brng += 360
    return rel_brng

def turn_to(target, spin_ratio):
    """Turn to target heading (degrees)."""
    trim = CARSPEED * spin_ratio
    # To avoid the complication of the 360 / 0 transition,
    # convert problem to one of aiming for a target at 180 degrees.
    rel_heading = relative_bearing(target)
    if rel_heading > 180:
        car.go(CARSPEED, 0, spin=trim)
        while rel_heading > 180:
            logger.debug(f"relative heading: {rel_heading}")
            rel_heading = relative_bearing(target)
            time.sleep(0.1)
    elif rel_heading < 180:
        car.go(CARSPEED, 0, spin=trim)
        while rel_heading < 180:
            logger.debug(f"relative heading: {rel_heading}")
            rel_heading = relative_bearing(target)
            time.sleep(0.1)
    car.stop_wheels()

def radius_turn(angle, turn_radius):
    """Turn car angle degrees (CCW positive) at turn_radius (degrees)."""
    spin_ratio = (W2W_DIST / 2 / turn_radius) / math.sqrt(2)
    trim = CARSPEED * spin_ratio
    start_heading = car.heading()
    end_heading = start_heading - angle
    if end_heading < 0:
        end_heading += 360
    turn_to(end_heading, spin_ratio)

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

def get_indx_of_longest_line(line_params):
    indx = 0
    if len(line_params) > 1:
        maxlen = 0
        for n, line in enumerate(line_params):
            coords, length, angle, dist = line
            if length > maxlen:
                indx = n
                maxlen = length
    return indx

def approach_wall(carspeed, clearance, mapping=False):
    """Approach wall squarely at carspeed until distance to wall < clearance."""

    # get scan line(s)
    data = car.scan(spd=180, lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data, gap=6)
    line_params = pscan.get_line_parameters()
    
    # Examine first line found
    coords, length, angle, dist = line_params[0]

    # display and save initial map
    if mapping:
        pscan.map(nmbr=1, display_all_points=True)

    # OK to proceed?
    if dist > clearance:
        car.go(carspeed, math.pi/2)
        logger.debug("")
        logger.debug(f"Approach wall to dist < {clearance}.")

    # initialize steering variables
    trim = 3  # estimate of needed steering trim
    target_angle = 0
    prev_error = angle - target_angle
    logger.debug("")
    logger.debug(f"Dist\tAngle\tPterm\tDterm\ttrim")

    # continue toward wall
    while dist > clearance:
        # get scan line(s)
        data = car.scan(spd=180, lev=17500, hev=22500)
        pscan = proscan.ProcessScan(data, gap=6)
        line_params = pscan.get_line_parameters()

        # Examine first line found
        coords, length, angle, dist = line_params[0]
        
        # Adjust trim to maintain angle=target using pid feedback
        error = angle - target_angle
        p_term = error
        d_term = error - prev_error
        prev_error = error
        adjustment = int((p_term * KP + d_term * KD))
        trim += adjustment
        logger.debug(f"{int(dist)}\t{angle:.2f}\t{p_term:.2f}\t{d_term:.2f}\t{trim}")
        car.go(carspeed, math.pi/2, spin=trim)

    car.stop_wheels()

def drive_along_wall_to_right(carspeed, clearance, mapping=False):
    """Drive to right maintaining clearance to wall in front. Stop at end.
    Return dist value at end."""

    EOW = 10
    # get scan line(s)
    data = car.scan(spd=180, lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data, gap=5)
    line_params = pscan.get_line_parameters()

    # find longest line 
    indx = get_indx_of_longest_line(line_params)
    coords, length, angle, dist = line_params[indx]
    end_of_wall = coords[-1][0]  # x coordinate of right end of wall

    # display and save initial map
    if mapping:
        pscan.map(nmbr=2, display_all_points=True)

    # OK to proceed?
    if end_of_wall > EOW:
        car.go(carspeed, 0)
        logger.debug("")
        logger.debug(f"Drive right until end of wall < {EOW}")

    # initialize steering variables
    trim = 3  # estimate of needed steering trim
    target_angle = 0
    prev_error = angle - target_angle
    logger.debug("")
    logger.debug(f"Dist\tAngle\tEOW\tPterm\tDterm\ttrim")

    # continue to end of wall
    while end_of_wall > EOW:
        # get scan line(s)
        data = car.scan(spd=180, lev=17500, hev=22500)
        pscan = proscan.ProcessScan(data, gap=5)
        line_params = pscan.get_line_parameters()
    
        # find longest line 
        indx = get_indx_of_longest_line(line_params)
        coords, length, angle, dist = line_params[indx]
        end_of_wall = coords[-1][0]  # x coordinate of right end of wall

        # Adjust trim to maintain angle=target using pid feedback
        error = angle - target_angle
        p_term = error
        d_term = error - prev_error
        prev_error = error
        adjustment = int((p_term * KP + d_term * KD))
        trim += adjustment
        logger.debug(f"{int(dist)}\t{angle:.2f}\t{end_of_wall:.2f}\t{p_term:.2f}\t{d_term:.2f}\t{trim}")
        car.go(carspeed, 0, spin=trim)

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


if __name__ == "__main__":

    dist = approach_wall(CARSPEED, CLEARANCE)
    dist = drive_along_wall_to_right(CARSPEED, CLEARANCE)
    radius_turn(90, dist)
    '''
    #data = car.go(100, 0)
    data = car.scan()
    print(f"'Sensor data' returned from car.go() command: {data}")
    time.sleep(1)
    data = car.stop_wheels()
    print(f"'Sensor data' returned from car.stop_wheels() command: {data}")
    '''
    car.close()
