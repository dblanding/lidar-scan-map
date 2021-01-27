"""Collection of car operation commands
"""
import logging
import math
import pickle
import sys
import time
import geom_utils as geo
import omnicar as oc
import proscan
from pprint import pprint

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

car = oc.OmniCar()
time.sleep(0.5)
from_arduino = car._read_serial_data()
logger.debug(f"Message from Arduino: {from_arduino}")

W2W_DIST = 34  # separation between coaxial wheels (cm)
SONAR_LIDAR_OFFSET = 15  # offset distance between sonar & lidar
FWD = 95  # forward drive direction (incl +5 'crab' compensation)
LFT = 180  # left drive direction
REV = 270  # reverse drive direction
RGT = 0  # right drive direction
KP = 0.25  # steering PID proportional coefficient
KD = 0.3  # steering PID derivative coefficient
CARSPEED = 150  # default car speed
RATE = 13.5  # cm/sec traveled by car @ CARSPEED = 150
R = 50  # Distance margin for clearing obstructions
EOW = 10  # Threshold End of Wall
CLEARANCE = 40  # nominal wall clearance (cm)

class PID():
    """Adjust trim to maintain heading using pid feedback from compass."""

    def __init__(self, target):
        self.target = target
        self.prev_error = 0
        self.trimval = 6

    def trim(self):
        """Return value for spin"""
        hdg = car.heading()
        heading_error = hdg - self.target
        p_term = heading_error
        d_term = heading_error - self.prev_error
        self.prev_error = heading_error
        adjustment = int((p_term * KP + d_term * KD))
        self.trimval += adjustment
        #logger.debug(f"P-term: {p_term:.2f}\tD-term: {d_term:.2f}\tTrim: {self.trimval}\tHDG: {int(hdg)}")
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

def turn_to(target):
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
    """
    Turn toward target course (degrees) while going in direction.
    (For example, direction = FWD for straight ahead.)
    """
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
    while underway in direction (eg: LFT, FWD or RGT)."""
    spin_ratio = (W2W_DIST / 2 / turn_radius) / math.sqrt(2)
    trim = CARSPEED * spin_ratio
    target = car.heading() - angle
    if target < 0:
        target += 360
    _turn_on_the_go(target, direction, spin_ratio)

def print_line_params(line_params):
    for item in line_params:
        coords, length, angle, distance = item
        p1_xy_coords, p2_xy_coords = coords
        p1_rp_coords = geo.r2p(p1_xy_coords)
        p2_rp_coords = geo.r2p(p2_xy_coords)
        print(f"P1 rect coords: ({int(p1_xy_coords[0]):.2f}, {int(p1_xy_coords[1]):.2f})")
        print(f"P2 rect coords: ({int(p2_xy_coords[0]):.2f}, {int(p2_xy_coords[1]):.2f})")
        print(f"P1 polar coords: ({int(p1_rp_coords[0]):.2f}, {p1_rp_coords[1]:.2f})")
        print(f"P2 polar coords: ({int(p2_rp_coords[0]):.2f}, {p2_rp_coords[1]:.2f})")
        print(f"length: {length:.2f} cm")
        print(f"angle: {angle:.2f} deg")
        print(f"distance: {distance:.2f} cm")
        print()

def square_to_wall(nmbr=0, mapping=True):
    """Spin car to square with wall that is roughly in front."""

    # scan and get parameters of most salient line
    data = save_scan(nmbr=nmbr)
    pscan = proscan.ProcessScan(data, lev=17500, hev=22500)
    longest_region_idx = pscan.regions_by_length()[0]
    longest_segment = pscan.segments_in_region(longest_region_idx)[0]
    line_params = pscan.get_line_parameters(longest_segment)
    coords, length, angle, dist = line_params

    # calculate target heading to null angle
    logger.debug(f"angle = {angle} degrees")
    logger.debug(f"heading = {car.heading()} degrees")
    target = car.heading() - angle
    logger.debug(f"target = {target} degrees")
    
    # display and save initial map
    if mapping:
        pscan.map(seq_nmbr=nmbr, display_all_points=True)

    # Turn in place to target heading
    turn_to(target)
    logger.debug(f"heading = {car.heading()} deg")
    
def approach_wall(carspeed, clearance, nmbr=1, mapping=True):
    """
    Approach wall at carspeed while trimming course to maintain initial
    compass heading. Stop when distance to wall reaches clearance.
    """

    # scan and get parameters of most salient line
    data = save_scan(nmbr=nmbr)
    pscan = proscan.ProcessScan(data, lev=17500, hev=22500)
    longest_region_idx = pscan.regions_by_length()[0]
    longest_segment = pscan.segments_in_region(longest_region_idx)[0]
    line_params = pscan.get_line_parameters(longest_segment)
    coords, length, angle, dist = line_params

    # Calculate distance & time to reach target location
    dist_to_travel = dist - clearance
    time_to_travel = dist_to_travel / RATE

    # display and save initial map
    if mapping:
        pscan.map(seq_nmbr=nmbr, display_all_points=True)

    # OK to proceed?
    if dist > clearance:

        # instantiate PID steering
        target = int(car.heading())
        pid = PID(target)
        logger.debug("")
        msg = f"Approaching wall to dist: {clearance}, target heading = {target}"
        logger.debug(msg)

        # continue toward wall
        start = time.time()
        delta_t = 0
        logger.debug("Dist:\tTime:")
        while delta_t < time_to_travel:
            delta_t = time.time() - start
            sonardist, *rest = car.go(carspeed, FWD, spin=pid.trim())
            logger.debug(f"{sonardist}\t{delta_t}")
        car.stop_wheels()
    else:
        print(f"Already within {int(dist)}cm of wall")

def drive_along_wall_to_right(carspeed, clearance, nmbr=2, mapping=True):
    """
    Drive to right maintaining clearance to wall in front. Stop at end.
    Return dist value at end."""

    # scan and get parameters of most salient line
    data = save_scan(nmbr=nmbr)
    pscan = proscan.ProcessScan(data, lev=17500, hev=22500)
    longest_region_idx = pscan.regions_by_length()[0]
    longest_segment = pscan.segments_in_region(longest_region_idx)[0]
    line_params = pscan.get_line_parameters(longest_segment)
    coords, length, angle, dist = line_params

    # X coordinate of right end of wall
    end_of_wall = coords[-1][0]
    print(end_of_wall)

    # display and save initial map
    if mapping:
        pscan.map(seq_nmbr=nmbr, display_all_points=True, show=True)

    # OK to proceed?
    if end_of_wall > EOW:
        car.go(carspeed, RGT)

    # continue to end of wall
    while end_of_wall > EOW:

        # scan and find 2 most salient regions
        nmbr += 1
        data = save_scan(nmbr=nmbr)
        pscan = proscan.ProcessScan(data, lev=17500, hev=22500)
        longest_regions = pscan.regions_by_length()[:2]

        # choose left-most region
        longest_regions.sort()  # sorts by value of index
        region_idx = longest_regions[0]

        # get parameters of most salient line in region
        segment = pscan.segments_in_region(region_idx)[0]
        line_params = pscan.get_line_parameters(segment)
        coords, length, angle, dist = line_params

        # use dist to wall feedback for cross-track error correction
        x_track_error = dist - clearance
        Kx = 1  # Coefficient for cross track correction of travel direction
        direction = RGT + Kx * x_track_error
        
        # use wall angle feedback to trim heading
        Ka = 1  # Coefficient for heading error correction
        trim = Ka * angle

        # X coordinate of right end of wall
        end_of_wall = coords[-1][0]

        car.go(carspeed, direction, spin=trim)
        msg = f"Dist: {int(dist)}\tAngle: {angle:.2f}\tEOW: {end_of_wall:.2f}\tTrim: {trim}"
        logger.debug(msg)

    car.stop_wheels()

    # display and save final map
    if mapping:
        pscan.map(seq_nmbr=nmbr, display_all_points=False)
    return dist  # needed by caller for radius of next turn

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

def find_std_dev(datalist):
    # Standard deviation of list 
    # Using sum() + list comprehension 
    mean = sum(datalist) / len(datalist) 
    variance = sum([((x - mean) ** 2) for x in datalist]) / len(datalist) 
    std_dev = variance ** 0.5
    return mean, std_dev

def save_scan(nmbr=None, lev=oc.LEV, hev=oc.HEV):
    """
    Scan and save data in numbered file. Return data.
    """
    if nmbr is None:
        nmbr = ''
    data = car.scan(lev=lev, hev=hev)
    with open(f'Data/scan_data{nmbr}.pkl', 'wb') as f:
        pickle.dump(data, f)
    logger.debug(f"Number of scan points: {len(data)}")
    #save_scandata_as_csv(data, f'Data/scan_data{nmbr}.csv')
    return data

def scan_and_plan(nmbr=None):
    """Scan, save data and analyze. Return course & distance to open sector.
    """
    if nmbr is None:
        nmbr = ''
    data = save_scan(nmbr)
    pscan = proscan.ProcessScan(data)
    logger.debug(f"Regions = {pscan.regions}")
    logger.debug(f"Zero Regions = {pscan.zero_regions}")

    # find indexes of non-zero regions sorted by number of points
    long_regs = pscan.regions_by_length()
    for idx in pscan.zero_regions:
        long_regs.remove(idx)

    # find just left region and right region
    long2regs = long_regs[:2]
    long2regs.sort()
    try:
        left_region, right_region = long2regs
    except ValueError:
        pscan.map(seq_nmbr=nmbr, display_all_points=True)
        return 0, 0

    # find 'far' end of L & R regions as it will be the constriction
    left_pnt_indx = pscan.regions[left_region][-1]
    right_pnt_indx = pscan.regions[right_region][0]

    # find coords of each point
    left_pnt = pscan.points[left_pnt_indx].xy
    right_pnt = pscan.points[right_pnt_indx].xy
    logger.debug(f"Left point: {left_pnt}")
    logger.debug(f"Right point: {right_pnt}")

    # construct waypoint halfway between left and right points
    waypnt = geo.midpoint(left_pnt, right_pnt)
    
    # Path to travel from origin to waypoint (in center of gap)
    r = int(geo.p2p_dist((0,0), waypnt))
    theta = int(geo.p2p_angle((0,0), waypnt))  # w/r/t +X direction
    course = theta - 90  # relative to +Y direction

    # print results and display map
    logger.debug(f"Relative course: {course}")
    logger.debug(f"Travel Distance: {r}")
    pscan.map(seq_nmbr=nmbr, display_all_points=True)
    return course, r

if __name__ == "__main__":
    '''
    nmbr = 0
    while True:
        # scan & plan first leg of route
        nmbr += 1
        heading = car.heading()
        print(f"Car initial heading = {heading}")
        course_deg, r = scan_and_plan(nmbr=nmbr)
        proceed = input("Proceed? (y or n)")
        if proceed == 'y':

            # turn car to new heading
            turn_to(heading - course_deg)

            # drive car to waypoint along prescribed path
            drive_time = r / RATE
            pid = PID(car.heading())
            start_time = time.time()
            while time.time()-start_time < drive_time:
                dist, *rest = car.go(CARSPEED, FWD, spin=pid.trim())
            car.stop_wheels()
        else:
            break
    '''
    
    n = 0  # times through loop
    while n < 5:
        nmbr = n * 10  # sequence number on saved data
        n += 1
        square_to_wall(nmbr=nmbr)
        nmbr += 1
        approach_wall(CARSPEED, CLEARANCE, nmbr=nmbr)
        nmbr += 1
        dist = drive_along_wall_to_right(CARSPEED, CLEARANCE, nmbr=nmbr)
        radius_turn_on_the_go(RGT, 90, dist)
    
    dist = car.get_sensor_data()
    print(dist)
    car.close()
