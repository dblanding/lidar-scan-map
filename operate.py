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
FWD = 90   # forward drive direction
LFT = 180  # left drive direction
REV = 270  # reverse drive direction
RGT = 0  # right drive direction
KP = 0.25  # steering PID proportional coefficient
KD = 0.3  # steering PID derivative coefficient
CARSPEED = 150  # default car speed
RATE = 13.5  # cm/sec traveled by car @ CARSPEED = 150
R = 50  # Distance margin for clearing obstructions
EOW = 25  # Threshold End of Wall
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

class ETA():
    """
    Estimate Time of Arrival based on periodically updated values.
    """

    def __init__(self, initial_val, target):
        self.target = target
        self.initial_val = initial_val
        self.prev_val = initial_val
        self.prev_time = time.time()
        self.delta_time = None

    def update(self, val):
        """
        Based on value, predict time to arrive at target.

        Return 2-element tuple: (eta > delta_time (boolean), eta (float))
        """
        now = time.time()
        self.delta_time = now - self.prev_time

        if self.initial_val > self.target:  # values decreasing
            # calculate delta_value (change in value per update)
            delta_val = (self.prev_val - val)

        elif self.initial_val < self.target:  # values increasing
            # calculate delta_value (change in value per update)
            delta_val = (val - self.prev_val)

        # estimate time to arrival at target
        slope = self.delta_time / delta_val
        eta = (val - self.target) * slope
        self.prev_val = val
        self.prev_time = now

        # estimate nmbr of remaining updates
        nmbr_of_updates_remaining = int(eta / self.delta_time)

        return (nmbr_of_updates_remaining, eta)


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
    #logger.debug(f"heading: {car.heading()}")
    rel_brng = int(car.heading() - delta)
    return normalize_angle(rel_brng)

def turn_to(target):
    """Turn (while stopped) toward target course (degrees)."""
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
            #time.sleep(0.1)
            #car.stop_wheels()
            heading_error = relative_bearing(target) - 180
            logger.debug(f"relative heading: {heading_error} deg")
            time.sleep(0.1)
        car.stop_wheels()
        while heading_error < -2:
            spd = heading_error - 40
            car.spin(spd)
            #time.sleep(0.1)
            #car.stop_wheels()
            heading_error = relative_bearing(target) - 180
            logger.debug(f"heading error: {heading_error} deg")
            time.sleep(0.1)
        car.stop_wheels()
        if -3 < abs(heading_error) < 3:
            print("done")
            done = True

def _turn_on_the_go(speed, target, direction, spin_ratio):
    """
    Turn toward target course (degrees) while going in direction.
    (For example, direction = FWD for straight ahead.)
    """
    target = normalize_angle(target)
    trim = speed * spin_ratio
    # To avoid the complication of the 360 / 0 transition,
    # convert problem to one of aiming for a target at 180 degrees.
    heading_error = relative_bearing(target) - 180
    if heading_error > 0:
        car.go(speed, direction, spin=trim)
        while heading_error > 0:
            #logger.debug(f"relative heading: {int(heading_error)}")
            heading_error = relative_bearing(target) - 180
            time.sleep(0.1)
    elif heading_error < 0:
        car.go(speed, direction, spin=trim)
        while heading_error < 0:
            #logger.debug(f"heading error: {int(heading_error)}")
            heading_error = relative_bearing(target) - 180
            time.sleep(0.1)
    car.stop_wheels()

def radius_turn_on_the_go(speed, direction, angle, turn_radius):
    """Turn car angle degrees (CCW positive) at turn_radius (cm)
    while underway in direction (eg: LFT, FWD or RGT)."""
    spin_ratio = (W2W_DIST / 2 / turn_radius) / math.sqrt(2)
    trim = speed * spin_ratio
    target = car.heading() - angle
    if target < 0:
        target += 360
    _turn_on_the_go(speed,target, direction, spin_ratio)

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

def get_closest_line_params(nmbr, lev=5000, mapping=False):
    """
    Scan and return parameters of most salient line in closest region.

    Save scan data in file labeled with nmbr.
    Set mapping=True to save plot.

    return line parameters as tuple (ccords, length, angle, dist)
    """
    data = save_scan(nmbr=nmbr, lev=lev)
    pscan = proscan.ProcessScan(data, lev=lev, gap=10, fit=4)
    closest_region_idx = pscan.closest_region()
    longest_segment = pscan.segments_in_region(closest_region_idx)[0]
    line_params = pscan.get_line_parameters(longest_segment)
    if mapping:
        pscan.map(seq_nmbr=nmbr, display_all_points=True)
    return line_params

def align_to_wall(nmbr=0, mapping=True):
    """Spin car to align Y-axis to wall that is close in front."""

    coords, length, angle, dist = get_closest_line_params(nmbr)

    # calculate target heading to null angle
    logger.debug(f"angle = {angle} degrees")
    logger.debug(f"heading = {car.heading()} degrees")
    target = car.heading() - angle + 90
    logger.debug(f"target = {target} degrees")
    
    # Turn in place to target heading
    turn_to(target)
    logger.debug(f"heading = {car.heading()} deg")
    
def square_to_wall(nmbr=0, mapping=True):
    """Spin car to aim squarely at wall that is close in front."""

    coords, length, angle, dist = get_closest_line_params(nmbr)

    # calculate target heading to null angle
    logger.debug(f"angle = {angle} degrees")
    logger.debug(f"heading = {car.heading()} degrees")
    target = car.heading() - angle
    logger.debug(f"target = {target} degrees")
    
    # Turn in place to target heading
    turn_to(target)
    logger.debug(f"heading = {car.heading()} deg")
    
def approach_wall(ddir, carspeed, clearance, nmbr=1, mapping=True):
    """
    Drive in direction ddir at carspeed while trimming course to maintain
    initial compass heading. Stop when distance to wall reaches clearance.
    """

    coords, length, angle, dist = get_closest_line_params(nmbr)

    # OK to proceed?
    if dist > clearance:
        # Calculate distance & time to reach target location
        dist_to_travel = dist - clearance
        time_to_travel = dist_to_travel / RATE

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
            sonardist, *rest = car.go(carspeed, ddir, spin=pid.trim())
            # Equivalent lidar distance = sonardist + SONAR_LIDAR_OFFSET
            logger.debug(f"{sonardist+SONAR_LIDAR_OFFSET}\t{delta_t}")
        car.stop_wheels()

    elif dist < clearance:
        # Calculate distance & time to reach target location
        dist_to_travel = clearance - dist
        time_to_travel = dist_to_travel / RATE

        # back away from wall
        start = time.time()
        delta_t = 0
        logger.debug("Dist:\tTime:")
        while delta_t < time_to_travel:
            delta_t = time.time() - start
            sonardist, *rest = car.go(carspeed, REV, spin=0)
            # Equivalent lidar distance = sonardist + SONAR_LIDAR_OFFSET
            logger.debug(f"{sonardist+SONAR_LIDAR_OFFSET}\t{delta_t}")
        car.stop_wheels()
    else:
        print(f"Already within {int(dist)}cm of wall")

def drive_along_wall_on_left(carspeed, clearance, nmbr=2, mapping=True):
    """
    Drive FWD maintaining clearance to wall on left. Stop at end.
    Return dist value at end.
    """

    coords, length, angle, dist = get_closest_line_params(nmbr)

    # Y coordinate of far end of wall
    end_of_wall = coords[-1][1]
    print(f"end_of_wall = {end_of_wall}")

    # OK to proceed?
    if end_of_wall > EOW:
        eta = ETA(end_of_wall, 10)
        car.go(carspeed, FWD)

    # continue to end of wall
    while end_of_wall > EOW:

        # scan and get parameters of most salient line
        nmbr += 1
        coords, length, angle, dist = get_closest_line_params(nmbr)
        
        # Y coordinate of right end of wall
        end_of_wall = coords[-1][1]
        
        # use dist to wall feedback for cross-track error correction
        x_track_error = dist - clearance
        Kx = 1  # Coefficient for cross track correction of travel direction
        direction = FWD + Kx * x_track_error
        
        # use wall angle feedback to trim heading
        Ka = 1  # Coefficient for heading error correction
        trim = Ka * (angle - 80)  # This is a fudge. It ought to be 90

        car.go(carspeed, direction, spin=trim)
        msg = f"Dist: {int(dist)}\tAngle: {angle:.2f}\tEOW: {end_of_wall:.2f}\tTrim: {trim}"
        logger.debug(msg)
        eta_data = eta.update(end_of_wall)
        logger.debug(f"ETA dta: {eta_data}")
        ok_to_continue, time_to_continue = eta_data
        if not ok_to_continue:
            if time_to_continue > 0:
                time.sleep(time_to_continue)
            logger.debug("At end of wall. Breaking out of loop")
            break

    car.stop_wheels()
    # final scan
    nmbr += 1
    clad = get_closest_line_params(nmbr)
    print(f"Final Scan: {nmbr}")
    pprint(clad)

    return dist  # needed by caller for radius of next turn

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
    logger.debug(f"Saving scan number {nmbr} with {len(data)} points.")
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
    left_pnt = pscan.points[left_pnt_indx].get("xy")
    right_pnt = pscan.points[right_pnt_indx].get("xy")
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
    n = 0
    while n < 2:  # times through loop
        nmbr = n * 100  # sequence number on saved data

        print()
        print(f"Aligning to wall at left sequence number {nmbr}")
        print()
        align_to_wall(nmbr=nmbr)
        nmbr += 1
        
        print()
        print(f"Approaching wall at left sequence number {nmbr}")
        print()
        approach_wall(LFT, CARSPEED, CLEARANCE, nmbr=nmbr)
        nmbr += 1

        print()
        print(f"Driving along wall at left sequence number {nmbr}")
        print()
        dist = drive_along_wall_on_left(CARSPEED, CLEARANCE, nmbr=nmbr)

        print()
        print(f"Car heading before turn = {car.heading()}")
        print(f"Turning corner")
        
        radius_turn_on_the_go(CARSPEED, FWD, 85, dist)
        print(f"Car heading after turn = {car.heading()}")
        n += 1

    sonar = car.get_sensor_data()
    print(f"Sonar data: {sonar}")
    car.close()
