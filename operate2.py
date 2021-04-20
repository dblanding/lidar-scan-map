"""Top level operating script for omni-wheel vehicle.
With omni-wheel vehicle poised in 'Home' position, click 'run'.
Vehicle will begin a multi-leg trip by computing waypoints in open
sectors, then pausing at each waypoint to scan & map & repeat. 
Will follow a generally CCW loop through its environment. 
Markdown file will be written on completion of trip."""

import logging
import math
import sys
import time
from constants import CARSPEED, FWD, SONAR_STOP, PIDTRIM
import geom_utils as geo
import mapper
import omnicar as oc
import proscan2
from triplogger import TripLog

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

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
    zero. Targets to the left of straight ahead have a (+) relative
    bearing; targets to the right have a (-) relative bearing.
    If, for example, the car is on a heading of 0 degrees and intends
    to turn left to a target heading of -90 degrees, the relative
    bearing of the target is +90 degrees.
    This is consistent with the standard mathematics convention of
    measuring angles in the CCW direction as positive (+)."""
    hdg = car.heading
    rel_brng = int(hdg - target)
    return normalize_angle(rel_brng)

def turn_to_abs(target_angle):
    """Turn (while stopped) to absolute target angle (degrees)."""
    target = normalize_angle(target_angle)
    logger.debug(f"Normalized angle: {target} deg")
    # To avoid the complication of the -180 / +180 transition,
    # convert problem to one of aiming for a target at 0 degrees.
    heading_error = relative_bearing(target)
    logger.debug(f"relative heading: {heading_error} deg")
    turn_complete = False
    while not turn_complete:
        if heading_error > 2:
            spd = 60
            foo = car.spin(spd)
            print(foo)
            while heading_error > 2:
                heading_error = relative_bearing(target)
                print(f"relative heading: {heading_error} deg")
            car.stop_wheels()
        if heading_error < -2:
            spd = -60
            foo = car.spin(spd)
            print(foo)
            while heading_error < -2:
                heading_error = relative_bearing(target)
                print(f"heading error: {heading_error} deg")
            car.stop_wheels()
        heading_error = relative_bearing(target)
        print(f"Heading Error = {heading_error}")
        if -2 <= abs(heading_error) <= 2:
            turn_complete = True

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


class Trip():
    """Scan, plan, map & drive multi-leg trip.

    Starting from 'home' position at WCS (0, 0), heading angle = 0,
    car navigates automously, by analyzing scan data and finding its
    next waypoint in the left-most open sector, then drives toward it.
    As it moves along each leg of its trip, it tracks its progress to
    each successive waypoint onto a map of its world."""

    def __init__(self):
        car.reset_heading()
        self.nmbr = 0  # Leg number
        self.data = None  # Scan data
        self.posn = (0, 0)  # Position of car (in WCS)
        self.heading = -car.heading  # Current heading of car
        self.theta = 0  # Angle to next target (CCW from car X axis)
        self.drive_dist = 0  # Distance to target point
        self.rel_trgt_pnt = (0, 0)  # target point (CCS)
        self.waypoints = []  # list of waypoints visited
        self.COAST_DIST = 15  # coast dist (cm) after wheels stopped
        self.log = TripLog()

    def complete_one_leg(self):
        """Auto sequence multiple legs of trip."""
        self.nmbr += 1
        self.log.addplot(self.nmbr)
        self.log.addline(f"Leg {self.nmbr}")
        self.log.addline(f"Coords: {self.posn}")
        self.waypoints.append(self.posn)
        self.scan_plan()
        self.show_map()
        char = input("Enter y to continue: ")
        if char not in "yY":
            self.log.write()
            return True
        self.turn()
        self.drive_to_target()
        return False

    def scan_plan(self):
        """Scan and compute target in open sector."""
        self.data = car.scan(spd=120)
        self.rel_trgt_pnt = car.auto_detect_open_sector()
        # convert target_pnt to dist, theta for turn & drive
        dist, theta = geo.r2p(self.rel_trgt_pnt)
        self.theta = normalize_angle(theta)
        self.drive_dist = dist

    def show_map(self):
        """Show (most salient) scan points overlayed on a map.
        Also show proposed next target point (yellow),
        current car position (red),
        and previously visited waypoints (green)."""
        pscan = proscan2.ProcessScan(self.data)
        longest_regions = pscan.regions_by_length()
        scanpoints = []  # xy coords of points in longest regions
        for regn_indx in longest_regions:
            points_in_region = pscan.get_points_in_region(regn_indx)
            if len(points_in_region) > 15:
                scanpoints.extend(points_in_region)
        # Transform point coordinates from Car CS to Map CS
        hdg = self.heading
        tx, ty = self.posn
        transformed_pnts = [xform_pnt(point, hdg, tx, ty)
                            for point in scanpoints]
        transformed_target = xform_pnt(self.rel_trgt_pnt, hdg, tx, ty)
        # Make overlay plot of points on map, then show & save map
        mapper.plot(transformed_pnts, mapper.load_base_map(),
                    waypoints=self.waypoints,
                    target=transformed_target,
                    carspot=self.posn,
                    seq_nmbr=self.nmbr)

    def turn(self):
        """Turn to target heading, update self.heading"""
        if self.theta < 0:
            self.log.addline(f"Turning Right {-self.theta} deg.")
        else:
            self.log.addline(f"Turning Left {self.theta} deg.")
        turn_to_abs(car.heading - self.theta)
        self.heading = -car.heading
        self.log.addline(f"Heading after turn: {self.heading} deg.")

    def drive_to_target(self, spd=CARSPEED):
        """Drive forward self.drive_dist toward target,
        updating self.posn incrementally along the way."""
        car.reset_odometer()
        time.sleep(1)
        prev_dist = 0
        waypoints = []
        self.log.addline(f"Driving {self.drive_dist:.1f} cm.")
        trim = PIDTRIM
        dist_to_go = self.drive_dist - car.odometer
        print(f"Distance to go: {dist_to_go}")
        while dist_to_go > self.COAST_DIST:
            car.go(spd, FWD, spin=trim)
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
        self.waypoints.extend(waypoints)
        self.log.addline(f"Final heading = {self.heading} deg.")
        self.log.addline()


if __name__ == "__main__":

    trip = Trip()
    done = False
    while not done:
        done = trip.complete_one_leg()

    car.close()
