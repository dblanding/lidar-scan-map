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

PAUSE = 0.05  # sec PAUSE time needed between wheel motor commands
CLEARANCE = 60  # threshold min clearance to wall
MIN_LENGTH = 20  # threshold min length to end of wall
MIN_DIST = 40  # threshold min distance from car to wall
KP = 0.5  # steering PID proportional coefficient
KD = 0.6  # steering PID derivative coefficient
CARSPEED = 50  # default car speed max=100, min=25


def jog_cw(spd, t):
    """Spin car CW at spd (int between 100-255) for t seconds
    """
    car.spin_cw(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(PAUSE)

def jog_ccw(spd, t):
    """Spin car CCW at spd (int between 100-255) for t seconds
    """
    car.spin_ccw(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(PAUSE)

def scan_series(nbr, lev=10000, hev=30000):
    """Perform a series of nbr successive scans of angular sector from
    lev (low encoder value) to hev (high encoder value), moving the car
    between scans. Save the raw scan data as a pickle file.
    Generate & save map plots.
    """
    data_list = []  # list of scan data from multiple scans
    while nbr:
        scan_data = car.scan(lev=lev, hev=hev)
        logger.debug(scan_data[0])
        logger.debug("Number of data points: ", len(scan_data))
        scan_data.pop(0)  # may have 'stale' serial data
        logger.debug(scan_data[0])
        logger.debug("Number of data points: ", len(scan_data))
        data_list.append(scan_data)
        nbr -= 1
        if nbr:
            time.sleep(PAUSE)
            # Move the car between successive scans
            # jog_R(100, 0.75)
            jog_F(100, 1.0, trim=5)

    # Save scan data for later examination, if desired.
    with open('scan_data.pkl', 'wb') as f:
        pickle.dump(data_list, f)

    # map the scans
    for n, scan_data in enumerate(data_list):
        logger.debug()
        logger.debug(f"Scan number {n+1} of {len(data_list)}")
        map_scan_data.show_map(scan_data, n+1)


def relative_bearing(target):
    """Return 'relative' bearing of an 'absolute' target."""
    delta = target - 180
    rel_brng = car.heading() - delta
    if rel_brng < 0:
        rel_brng += 360
    return rel_brng

def turn_to(target):
    """Turn to target heading (degrees)."""
    # To avoid the complication of the 360 / 0 transition, convert
    # the problem to one of aiming for a target at 180 degrees.
    rel_heading = relative_bearing(target)
    if rel_heading > 180:
        car.spin_ccw(80)
        while rel_heading > 180:
            logger.debug(f"relative heading: {rel_heading}")
            rel_heading = relative_bearing(target)
            time.sleep(0.1)
    elif rel_heading < 180:
        car.spin_cw(80)
        while rel_heading < 180:
            logger.debug(f"relative heading: {rel_heading}")
            rel_heading = relative_bearing(target)
            time.sleep(0.1)
    car.stop_wheels()

def turn(angle):
    """Spin car angle degrees (CCW positive)."""
    start_heading = car.heading()
    end_heading = start_heading - angle
    if end_heading < 0:
        end_heading += 360
    turn_to(end_heading)

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

def along_wall_to_right(carspeed, clearance):
    """Drive to right maintaining clearance to wall in front. Stop at end."""
    data = car.scan(spd=180, lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data)
    line_params = pscan.get_line_parameters()

    # Examine longest line found
    indx = get_indx_of_longest_line(line_params)
    coords, length, angle, dist = line_params[indx]
    end_of_wall = coords[-1][0]  # x coordinate of right end of wall
    logger.debug("")
    logger.debug(f"Dist = {int(dist)}")
    logger.debug(f"Angle = {angle:.2f}")
    logger.debug(f"end_of_wall = {end_of_wall}")
    pscan.map(nmbr=2, display_all_points=True)
    target_angle = 0
    if end_of_wall > 10:
        car.go(carspeed, 0)
    logger.debug("")
    logger.debug("Driving to right along wall.")

    # initialize steering variables
    trim = 6  # estimate of needed steering trim
    prev_error = angle - target_angle
    while end_of_wall > 10:  # continue to end of wall
        data = car.scan(spd=180, lev=17500, hev=22500)
        pscan = proscan.ProcessScan(data)
        line_params = pscan.get_line_parameters()
        # Examine longest line found
        indx = get_indx_of_longest_line(line_params)
        coords, length, angle, dist = line_params[indx]
        end_of_wall = coords[-1][0]  # x coordinate of right end of wall
        logger.debug("")
        logger.debug(f"Dist = {int(dist)}")
        logger.debug(f"Angle = {angle:.2f}")
        logger.debug(f"end_of_wall = {end_of_wall:.2f}")
        
        # Adjust trim to keep car heading matched to steer_target
        # Use pid feedback loop
        error = angle - target_angle
        p_term = error
        d_term = error - prev_error
        prev_error = error
        adjustment = int((p_term * KP + d_term * KD))
        trim += adjustment
        logger.debug(f"p_term = {p_term:.2f}, d_term = {d_term:.2f}, trim = {trim}")
        car.go(carspeed, 0, spin=trim)

    car.stop_wheels()
    pscan.map(nmbr=3, display_all_points=False)


def approach_wall(carspeed, clearance):
    """Approach wall squarely at carspeed until distance to wall < clearance.
    """
    data = car.scan(spd=180, lev=17500, hev=22500)
    pscan = proscan.ProcessScan(data, gap=6)
    line_params = pscan.get_line_parameters()
    print_line_params(line_params)

    # Examine first line found
    coords, length, angle, dist = line_params[0]
    pscan.map(nmbr=1, display_all_points=True)
    logger.debug("Approaching wall")
    logger.debug(f"Dist = {int(dist)}")
    logger.debug(f"Angle = {angle:.2f}")
    target_angle = 0
    if dist > clearance:
        car.go(carspeed, math.pi/2)

    # initialize steering variables
    trim = 6  # estimate of needed steering trim
    prev_error = angle - target_angle
    while dist > clearance:
        data = car.scan(spd=180, lev=17500, hev=22500)
        pscan = proscan.ProcessScan(data, gap=6)
        line_params = pscan.get_line_parameters()

        # Examine first line found
        coords, length, angle, dist = line_params[0]
        logger.info('')
        logger.info(f"Dist = {int(dist)} cm")
        logger.info(f"Angle = {angle:.2f} deg")
        # Adjust trim to maintain angle=target using pid feedback loop
        error = angle - target_angle
        p_term = error
        d_term = error - prev_error
        prev_error = error
        adjustment = int((p_term * KP + d_term * KD))
        trim += adjustment
        logger.debug(f"p_term = {p_term:.2f}, d_term = {d_term:.2f}, trim = {trim}")
        car.go(carspeed, math.pi/2, spin=trim)

    car.stop_wheels()


if __name__ == "__main__":

    approach_wall(CARSPEED, CLEARANCE)
    along_wall_to_right(CARSPEED, CLEARANCE)
    '''
    degrees = 270
    heading = degrees * math.pi / 180
    car.go(50, heading, spin=-10)
    time.sleep(2)
    car.stop_wheels()
    '''
    car.close()
