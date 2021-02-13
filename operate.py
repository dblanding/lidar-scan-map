"""Collection of car operation commands
"""
import logging
import math
from pathlib import Path
import pickle
import sys
import time
import geom_utils as geo
import omnicar as oc
import proscan
from pprint import pprint

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

car = oc.OmniCar()
time.sleep(0.5)
from_arduino = car._read_serial_data()
logger.debug(f"Message from Arduino: {from_arduino}")

W2W_DIST = 34  # separation between coaxial wheels (cm)
SONAR_LIDAR_OFFSET = 15  # offset distance between sonar & lidar
CARSPEED = 150  # default car speed
RATE = 13.5  # cm/sec traveled by car @ CARSPEED = 150
R = 50  # Distance margin for clearing obstructions
EOW = 25  # Threshold End of Wall
CLEARANCE = 40  # nominal wall clearance (cm)
FWD = 94   # forward drive direction (with 4 deg cross-track correction)
LFT = 180  # left drive direction
REV = 270  # reverse drive direction
RGT = 0  # right drive direction
KP = 0.2  # steering PID proportional coefficient
KI = 0.15  # steering PID integral coefficient
KD = 0.4  # steering PID derivative coefficient
PIDTRIM = 8  # default value for spin trim
PIDWIN = 6  # number of values to use in rolling average

class PID():
    """
    Closed loop compass feedback used to adjust steering trim underway.
    """

    def __init__(self, target):
        """ Instantiate PID object before entering loop.

        target is the target magnetic compass heading in degrees."""
        self.target = target
        self.prev_error = 0
        self.trimval = PIDTRIM
        # create a rolling list of previous (rws) error values
        self.initial = 1
        self.rws = PIDWIN  # rolling window size
        self.rwl = [self.initial] * self.rws  # rolling window list

    def _get_integral_error(self, hdg_err):
        """Return rolling average error."""
        self.rwl.insert(0, hdg_err)
        self.rwl.pop()
        return sum(self.rwl) / len(self.rwl)

    def trim(self):
        """Return trim value to steer toward target."""
        hdg = car.heading()
        heading_error = hdg - self.target
        p_term = heading_error * KP
        i_term = self._get_integral_error(heading_error) * KI
        d_term = (heading_error - self.prev_error) * KD
        self.prev_error = heading_error
        adjustment = int((p_term + i_term + d_term))
        self.trimval += adjustment
        pstr = f"P-term: {p_term:.2f}\t"
        istr = f"I-term: {i_term:.2f}\t"
        dstr = f"D-term: {d_term:.2f}\t"
        tstr = f"Trim: {self.trimval:.2f}\t"
        hstr = f"HDG-Error: {int(hdg)-self.target}"
        logger.debug(pstr + istr + dstr + tstr + hstr)
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

        Return 2-element tuple: (reads remaining (int), eta (float))
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


class DeltaT():
    """Home made timeit."""
    def __init__(self):
        self.prev = time.time()

    def delta(self):
        now = time.time()
        delt = now - self.prev
        self.prev = now
        return delt


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

def drive_ahead(dist):
    """drive dist and stop."""

    # instantiate PID steering
    target = int(car.heading())
    pid = PID(target)

    # drive
    time_to_travel = dist / RATE
    start = time.time()
    delta_t = 0
    while delta_t < time_to_travel:
        delta_t = time.time() - start
        _ = car.go(CARSPEED, FWD, spin=pid.trim())
    car.stop_wheels()

def drive_ahead(dist):
    """drive dist and stop. (no pid steering)"""

    # drive
    time_to_travel = dist / RATE
    start = time.time()
    delta_t = 0
    while delta_t < time_to_travel:
        delta_t = time.time() - start
        _ = car.go(CARSPEED, FWD, spin=PIDTRIM)
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

def get_closest_line_params(nmbr, mapping=False):
    """
    Scan and return parameters of most salient line in closest region.

    Save scan data in file labeled with nmbr.
    Set mapping=True to save plot.

    return line parameters as tuple (ccords, length, angle, dist)
    """
    data = save_scan(nmbr=nmbr)
    pscan = proscan.ProcessScan(data)
    closest_region_idx = pscan.closest_region()
    longest_segment = pscan.segments_in_region(closest_region_idx)[0]
    line_params = pscan.get_line_parameters(longest_segment)
    if mapping:
        pscan.map(seq_nmbr=nmbr, display_all_points=True)
    return line_params

def align_to_wall(nmbr=0, mapping=True):
    """Spin car to align Y-axis to wall that is closest."""

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
            sonardist, *rest = car.go(carspeed, ddir-180, spin=PIDTRIM)
            # Equivalent lidar distance = sonardist + SONAR_LIDAR_OFFSET
            logger.debug(f"{sonardist+SONAR_LIDAR_OFFSET}\t{delta_t}")
        car.stop_wheels()
    else:
        print(f"Already within {int(dist)}cm of wall")

def drive_along_wall_on_left(carspeed, clearance, nmbr=2, mapping=True):
    """Drive FWD along wall on left to end_of_wall

    Scan along the way, saving data in mumbered files, starting with nmbr.
    Find most salient line in closest region (at left).
    Use distance to line to adjust cross-track velocity to maintain clearance.
    Maintain heading parallel to line angle.
    Return distance value at end.
    """

    coords, length, angle, dist = get_closest_line_params(nmbr)

    # Y coordinate of far end of wall
    end_of_wall = coords[-1][1]

    # OK to proceed?
    if end_of_wall > EOW:
        # initialize ETA
        eta = ETA(end_of_wall, 10)

    # continue to end of wall
    while end_of_wall > EOW:  # 2.1 seconds / loop

        # Physical scanning, incl 1 rotation of rotor: 1.83 sec
        # scan and get parameters of most salient line
        nmbr += 1
        coords, length, angle, dist = get_closest_line_params(nmbr)
        # Y coordinate of far end of wall
        end_of_wall = coords[-1][1]

        # use dist to wall feedback for cross-track error correction
        x_track_error = dist - clearance
        Kx = 1  # Coefficient for cross track correction of travel direction
        direction = FWD + Kx * x_track_error

        # use wall angle feedback to trim heading
        Ka = 1  # Coefficient for heading error correction
        trim = Ka * (angle - 90)

        # Serial comm w/ Arduino: .25 sec
        car.go(carspeed, direction, spin=trim+PIDTRIM)
        msg = f"Dist: {int(dist)}\tAngle: {angle:.2f}\tEOW: {end_of_wall:.2f}\tTrim: {trim}"
        logger.debug(msg)
        eta_data = eta.update(end_of_wall)
        logger.debug(f"ETA dta: {eta_data}")
        scans_remaining, time_remaining = eta_data

        if end_of_wall < 50 and not scans_remaining:
            if time_remaining > 0:
                time.sleep(time_remaining)
            logger.debug("At end of wall. Breaking out of loop")
            break

    car.stop_wheels()
    # final scan
    nmbr += 1
    clad = get_closest_line_params(nmbr)
    logger.debug(f"Final Scan: {nmbr}")
    pprint(clad)

    return clad  # needed by caller for next turn

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

def follow_walls_left(n_cycles=3):
    """(Align, Approach, Along, Around) x n_cycles

    Align heading to (closest) wall on left,
    Approach (sideways) to CLEARANCE. Then drive FWD
    Along wall to end, then go
    Around corner.
    Repeat for n_cycles.

    Save all scan data. Run remap to generate plots.
    """
    n = 0
    while n < n_cycles:  # number of cycles through loop
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
        clad = drive_along_wall_on_left(CARSPEED, CLEARANCE, nmbr=nmbr)
        *rest, dist = clad

        print()
        print(f"Car heading before turn = {car.heading()}")
        print(f"Turning around corner")
        
        radius_turn_on_the_go(CARSPEED, FWD, 90, dist)
        print(f"Car heading after turn = {car.heading()}")
        n += 1
    
def follow_walls_left_lite(n_cycles=1):
    """(Along, Around) x n_cycles

    Along wall to end, then go Around corner.
    Repeat for n_cycles.

    Save all scan data. Run plot_data to generate plots.
    """
    n = 0
    while n < n_cycles:  # number of cycles through loop
        nmbr = n * 100  # sequence number on saved data

        print()
        print(f"Driving along wall at left sequence number {nmbr}")
        print()
        clad = drive_along_wall_on_left(CARSPEED, CLEARANCE, nmbr=nmbr)
        coords, length, angle, dist = clad

        # Calculate bevel turn
        end_of_wall = coords[-1][1]
        if end_of_wall > 0.5 * CLEARANCE:
            print("Didn't reach end of wall")
            break
        y = CLEARANCE+end_of_wall
        x = -dist
        turn_angle1 = math.atan2(y, x) * 180 / math.pi - 90
        drive_dist = math.sqrt(x**2 + y**2)
        turn_angle2 = 90 - turn_angle1
        print()
        print(f"Car turning left {turn_angle1:.0f} degrees")
        turn_to(car.heading() - turn_angle1)

        print()
        print(f"Car driving {drive_dist} cm")
        drive_ahead(drive_dist)
        
        print()
        print(f"Car turning left {turn_angle2:.0f} degrees")
        turn_to(car.heading() - turn_angle2)
        n += 1

def purge_data_folder():
    """
    Purge all the .pkl files in Data/ folder.
    """
    d = Path('Data')
    datalist = list(d.glob('*.pkl'))
    for file in datalist:
        file.unlink()

def plot_data():
    """
    Load each .pkl data file in Data/ folder, generate plot,
    save as correspondingly numbered .png image in Maps/ folder.
    """
    # remove all old plots from /Maps folder
    m = Path('Maps')
    maplist = list(m.glob('*.png'))
    for file in maplist:
        file.unlink()

    # generate new plots and save in 'Maps' folder
    p = Path('Data')
    pathlist = list(p.glob('*.pkl'))
    for f in pathlist:
        fname = f.stem
        nmbr_str = fname.rpartition('data')[-1]
        plot(nmbr_str, verbose=False, display=False)

def plot(nmbr, verbose=False, display=True):
    """
    Load saved datafile by file 'nmbr'

    if verbose: pretty print data
    if display: display interactive plot
    """
    filename = f'Data/scan_data{nmbr}.pkl'
    with open(filename, 'rb') as file:
        data = pickle.load(file)
    if verbose:
        pprint.pprint(data)
    pscan = proscan.ProcessScan(data)
    if verbose:
        print(f"Regions: {pscan.regions}")
    pscan.map(seq_nmbr=nmbr, display_all_points=True, show=display)

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

def drive_to_open_sector(nmbr):
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
            drive_ahead(r)
        else:
            break
def drive_to_spot():
    """Scan, ask for spot to drive to & display interactive map
    User then inpts coordinates of destination. Repeat."""
    nmbr = 0
    while True:
        # scan & display plot
        data = save_scan(nmbr)
        pscan = proscan.ProcessScan(data)
        pscan.map(seq_nmbr=nmbr, show=True)
        coordstr = input("Enter x, y coords of target, q to quit: ")
        if ',' in coordstr:
            xstr, ystr = coordstr.split(',')
            x = int(xstr)
            y = int(ystr)
            coords = (x, y)
        else:
            break
        r, theta = geo.r2p(coords)
        target_angle = int(theta - 90)
        print(f"Turning {target_angle} degrees")
        turn_to(car.heading()-target_angle)
        print(f"Driving {r} cm")
        drive_ahead(r)
        nmbr += 1
        

if __name__ == "__main__":

    drive_to_spot()
    #drive_to_open_sector(0)
    #follow_walls_left_lite(1)
    '''
    print("Purging data folder")
    purge_data_folder()
    print("Running car")
    follow_walls_left_lite(2)
    print("plotting data")
    plot_data()
    sonar = car.get_sensor_data()
    print(f"Sonar data: {sonar}")
    '''
    car.close()
