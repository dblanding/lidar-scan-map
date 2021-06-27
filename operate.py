"""Top level operating script for omni-wheel vehicle.
With omni-wheel vehicle poised in 'Home' position, click 'run'.
Vehicle will begin a multi-leg trip by computing waypoints in open
sectors, then pausing at each waypoint to scan & map & repeat. 
Will follow a generally CCW loop through its environment. 
Markdown log-file will be written on completion of trip."""

import logging
import math
import matplotlib.pyplot as plt
import os
import pickle
import sys
import time
from constants import CARSPEED, FWD, PIDTRIM, PIDWIN, KP, KI, KD, VLEG
import geom_utils as geo
import mapper
import omnicar as oc
import proscan2
from triplogger import TripLog

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

MAX_TURN_DITHER = 3  # Theshold for limiting dither in turning
MAX_HDG_ERR = 2  # Threshold max heading error for complete turn
POINTFOLDER = 'scanpoints'  # Location of saved scanpoints

car = oc.OmniCar()
time.sleep(0.5)
from_arduino = car._read_serial_data()
logger.info(f"Message from Arduino: {from_arduino}")

def normalize_angle(angle):
    """Convert any value of angle to a value between -180 and +180."""
    while angle < -180:
        angle += 360
    while angle > 180:
        angle -= 360
    return angle

def relative_bearing(target):
    """Return relative bearing of an absolute target.

    If a target is straight ahead of the car, its relative bearing is
    zero. Targets to the left of straight ahead have a negative relative
    bearing; targets to the right have a positive relative bearing.
    This is consistent with the standard conventions of navigation
    by compass angles.
    If, for example, while proceeding on an absolute course of 90 deg,
    a target with an absolute bearing of 135 deg would have a relative
    bearing of +45 deg. A target with an absolute bearing of 45 deg would
    have a relative bearing of -45 deg."""
    hdg = car.heading
    rel_brng = int(target - hdg)
    return normalize_angle(rel_brng)

def turn_to_abs(target_angle):
    """Turn (while stopped) to an absolute target angle (deg) as
    specified by the IMU.

    Pos direction is OPPOSITE to the standard mathematics convention of
    measuring angles in the CCW direction as positive (+) so there is
    potential for confusion.
    """
    target = normalize_angle(target_angle)
    logger.debug(f"Normalized angle: {target} deg")
    # To avoid the complication of the -180 / +180 transition,
    # convert problem to one of aiming for a target at 0 degrees.
    # If the target relative bearing is neg, heading error is pos.
    heading_error = -relative_bearing(target)
    logger.debug(f"relative heading: {heading_error} deg")
    turn_complete = False
    times_thru_loop = 0
    spd = 50
    while not turn_complete:
        times_thru_loop += 1
        if times_thru_loop > MAX_TURN_DITHER:
            spd = 40
        if heading_error > MAX_HDG_ERR:
            _ = car.spin(spd)
            print("Turning CCW")
            while heading_error > MAX_HDG_ERR:
                heading_error = -relative_bearing(target)
                #print(f"heading error: {heading_error} deg")
            car.stop_wheels()
        if heading_error < -MAX_HDG_ERR:
            _ = car.spin(-spd)
            print("Turning CW")
            while heading_error < -MAX_HDG_ERR:
                heading_error = -relative_bearing(target)
                #print(f"heading error: {heading_error} deg")
            car.stop_wheels()
        heading_error = -relative_bearing(target)
        print(f"Heading Error = {heading_error}")
        if -MAX_HDG_ERR <= abs(heading_error) <= MAX_HDG_ERR:
            turn_complete = True

def drive_ahead(dist, spd=None):
    """Drive dist and stop."""
    if not spd:
        spd = CARSPEED
    # instantiate PID steering
    target = int(car.heading)
    pid = PID(target)

    # drive
    odo = 0
    while odo < dist:
        print(f"ODO = {odo}")
        car.go(spd, FWD, spin=pid.trim())
        odo = car.odometer

    car.stop_wheels()
    time.sleep(0.5)
    odo = car.odometer
    print(f"Final odometer: {odo}")
    

def R_xform(pnt, angle):
    """Transform point by rotation angle (degrees) CW about the origin."""
    r, theta = geo.r2p(pnt)
    newpnt = geo.p2r(r, theta + angle)
    return newpnt

def T_xform(pnt, tx, ty):
    """Transform point by translation tx in x and ty in y."""
    x, y = pnt
    return (x+tx, y+ty)

def xform_pnt(pnt, ang, tx, ty):
    """Return 2D transformed point, rotated & translated.

    When transforming in both rotation and translation, rotation
    must be applied first, while the car's center is located at the
    origin. This results in a pure rotation.
    If a point were first transformed in translation, rotation
    would then produce an unwanted motion through an arc."""
    rotated = R_xform(pnt, ang)
    translated = T_xform(rotated, tx, ty)
    return translated

def integerize(coords):
    x, y = coords
    return (int(x), int(y))


class PID():
    """
    Closed loop compass feedback used to adjust steering trim underway.
    """

    def __init__(self, target):
        """ Instantiate PID object before entering loop.

        target is the target heading in degrees."""
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
        target = normalize_angle(self.target)
        logger.debug(f"Normalized angle: {target} deg")
        # To avoid the complication of the -180 / +180 transition,
        # convert problem to one of aiming for a target at 0 degrees.
        # If the target relative bearing is neg, heading error is pos.
        heading_error = -relative_bearing(target)
        logger.debug(f"relative heading: {heading_error} deg")
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


class Trip():
    """Scan, plan, map & drive multi-leg trip.

    Starting from 'home' position at WCS (0, 0), heading angle = 0,
    car navigates automously, by analyzing scan data and finding its
    next waypoint in the left-most open sector, then drives toward it.
    As it moves along each leg of its trip, it tracks its progress to
    each successive waypoint onto a map of its world."""

    def __init__(self, y_min=None, y_max=None):
        car.reset_heading()
        if y_min:
            self.y_min = y_min
        else:
            self.y_min = -500
        if y_max:
            self.y_max = y_max
        else:
            self.y_max = 500
        self.nmbr = 0  # Leg number
        self.data = None  # Scan data
        self.posn = (0, 0)  # Position of car (in WCS)
        self.theta = 0  # Angle to next target (CCW from car X axis)
        self.drive_dist = 0  # Distance to target point
        self.rel_trgt_pnt = (0, 0)  # target point (CCS)
        self.waypoints = []  # list of waypoints visited
        self.COAST_DIST = 25  # dist_to_go value to stop wheels
        self.log = TripLog()

    def complete_one_leg(self):
        """Auto sequence multiple legs of trip."""
        self.nmbr += 1
        self.log.addplot(self.nmbr)
        self.log.addline(f"Leg {self.nmbr}")
        self.log.addline(f"Starting Coords: {integerize(self.posn)}")
        self.waypoints.append(self.posn)
        self.scan_plan()
        # self.plot_histogram()
        self.map(show=True)
        decision = self.decide()
        if decision == 'quit':
            self.log.write()
            return True
        self.turn()
        self.drive_to_target()
        if self.y_min:
            x, y = self.posn
            if y <= self.y_min:
                self.log.write()
                return True
        if self.y_max:
            x, y = self.posn
            if y >= self.y_max:
                self.log.write()
                return True
        return False

    def decide(self):
        """Decide based on user input"""
        while True:
            char = input("Enter p to proceed, t to turn, q to quit: ")
            if char in "tT":  # turn
                theta = float(input("enter CCW angle(deg): "))
                self.turn(theta)
                self.scan_plan()
                self.map(show=True)
            elif char in "pP":  # proceed
                return 'proceed'
            elif char in 'qQ':  # quit
                return 'quit'
            else:
                print(f"You entered {char}. Try again.")

    def scan_plan(self):
        """Scan and compute target in left-most open sector."""
        self.data = car.scan(spd=120)
        try:
            self.rel_trgt_pnt = car.next_target_point()
        except:
            self.log.write()
            print("Unable to find next target point")
        # convert target_pnt to dist, theta for turn & drive
        dist, theta = geo.r2p(self.rel_trgt_pnt)
        self.theta = normalize_angle(theta)
        if dist > 240:  # odometer only goes to 255
            dist = 240
        self.drive_dist = dist

    def plot_histogram(self):
        """Plot histogram of lidar distance values."""
        dist_list = [pnt['dist']
                     for pnt in self.data
                     if pnt['dist'] > 0]
        plt.hist(dist_list, bins=10)
        plt.show()
        plt.clf()

    def map(self, show=False):
        """Show (most salient) scan points overlayed on a map.
        Also show proposed next target point (yellow),
        current car position (red),
        and previously visited waypoints (green)."""
        '''
        pscan = proscan2.ProcessScan(self.data)
        longest_regions = pscan.regions_by_length()
        scanpoints = []  # xy coords of points in longest regions
        for regn_indx in longest_regions:
            points_in_region = pscan.get_points_in_region(regn_indx)
            if len(points_in_region) > 15:
                scanpoints.extend(points_in_region)
        # Transform point coordinates from Car CS to Map CS
        # +/- convention for polar angle is opposite to heading
        '''
        scanpoints = [point.get('xy')
                      for point in self.data]
        angle = -normalize_angle(car.heading)
        tx, ty = self.posn
        transformed_pnts = [xform_pnt(point, angle, tx, ty)
                            for point in scanpoints]
        transformed_target = xform_pnt(self.rel_trgt_pnt, angle, tx, ty)
        # Make overlay plot of points on map, then show & save map
        mapper.plot(transformed_pnts, mapper.load_base_map(),
                    waypoints=self.waypoints,
                    target=transformed_target,
                    carspot=self.posn,
                    seq_nmbr=self.nmbr, show=show)

    def turn(self, theta=None):
        """Turn to (relative) target heading"""
        if not theta:
            theta = self.theta
        target_hdg = car.heading - theta
        if theta < 0:
            msg = f"Turning Right {-theta} deg "
            msg += f"to heading {target_hdg}"
            self.log.addline(msg)
        else:
            msg = f"Turning Left {theta} deg "
            msg += f"to heading {target_hdg}"
            self.log.addline(msg)
        turn_to_abs(target_hdg)
        self.log.addline(f"Heading after turn: {car.heading} deg.")

    def drive_to_target(self, spd=None):
        """Drive forward self.drive_dist toward target,
        updating self.posn incrementally along the way."""
        if not spd:
            spd = CARSPEED
        # instantiate PID steering
        target = int(car.heading)
        pid = PID(target)
        
        car.reset_odometer()
        print(f"Odometer offset = {car.ODOMETER_OFFSET}")
        print(f"First odo reading after reset = {car.odometer}")
        prev_dist = 0
        waypoints = []
        self.log.addline(f"Distance to target: {self.drive_dist:.1f} cm.")
        dist_to_go = self.drive_dist
        print(f"Distance to go: {dist_to_go}")
        wx, wy = self.posn
        while dist_to_go > self.COAST_DIST and self.y_min < wy < self.y_max:
            car.go(spd, FWD, spin=pid.trim())
            # Update self.posn incrementally
            curr_dist = car.odometer
            incr_dist = curr_dist - prev_dist
            prev_dist = curr_dist
            hdg = -car.heading
            dx, dy = geo.p2r(incr_dist, hdg)
            wx, wy = self.posn
            self.posn = (wx+dx, wy+dy)
            waypoints.append(self.posn)
            odo = car.odometer
            dist_to_go = self.drive_dist - odo
            print(f"Odometer: {odo}")
        car.stop_wheels()
        time.sleep(0.5)
        curr_dist = car.odometer
        incr_dist = curr_dist - prev_dist
        dx, dy = geo.p2r(incr_dist, hdg)
        wx, wy = self.posn
        self.posn = (wx+dx, wy+dy)
        waypoints.append(self.posn)
        self.log.addline(f"Distance driven: {car.odometer} cm.")
        # Just plot every third waypoint
        waypoints = [waypoint
                     for idx, waypoint in enumerate(waypoints)
                     if not idx % 3]
        self.waypoints.extend(waypoints)
        self.log.addline(f"Heading after drive: {car.heading} deg.")
        self.log.addline(f"Ending Coords: {integerize(self.posn)}")


if __name__ == "__main__":
    '''
    car.reset_odometer()
    time.sleep(0.1)
    drive_ahead(100)
    '''
    trip = Trip(y_max=250)
    done = False
    while not done:
        done = trip.complete_one_leg()
    turn_to_abs(-90)
    car.close()
