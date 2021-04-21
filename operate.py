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

MAX_HDG_ERR = 3  # Threshold max heading error for complete turn

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
    while not turn_complete:
        if heading_error > MAX_HDG_ERR:
            spd = 50
            foo = car.spin(spd)
            print(foo)
            while heading_error > MAX_HDG_ERR:
                heading_error = -relative_bearing(target)
                print(f"relative heading: {heading_error} deg")
            car.stop_wheels()
        if heading_error < -MAX_HDG_ERR:
            spd = -50
            foo = car.spin(spd)
            print(foo)
            while heading_error < -MAX_HDG_ERR:
                heading_error = -relative_bearing(target)
                print(f"heading error: {heading_error} deg")
            car.stop_wheels()
        heading_error = -relative_bearing(target)
        print(f"Heading Error = {heading_error}")
        if -MAX_HDG_ERR <= abs(heading_error) <= MAX_HDG_ERR:
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

def integerize(coords):
    x, y = coords
    return (int(x), int(y))


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
        self.log.addline(f"Starting Coords: {integerize(self.posn)}")
        self.waypoints.append(self.posn)
        self.scan_plan()
        self.show_map()
        char = input("Enter y to continue: ")
        if char not in "yY":  # end of trip
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
        if dist > 240:  # odometer only goes to 255
            dist = 240
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
        # +/- convention for polar angle is opposite to heading
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
                    seq_nmbr=self.nmbr)

    def turn(self):
        """Turn to (absolute) target heading"""
        target_hdg = car.heading - self.theta
        if self.theta < 0:
            msg = f"Turning Right {-self.theta} deg "
            msg += f"to heading {target_hdg}"
            self.log.addline(msg)
        else:
            msg = f"Turning Left {self.theta} deg "
            msg += f"to heading {target_hdg}"
            self.log.addline(msg)
        turn_to_abs(target_hdg)
        self.log.addline(f"Heading after turn: {car.heading} deg.")

    def drive_to_target(self, spd=CARSPEED):
        """Drive forward self.drive_dist toward target,
        updating self.posn incrementally along the way."""
        car.reset_odometer()
        time.sleep(1)
        prev_dist = 0
        waypoints = []
        self.log.addline(f"Distance to target: {self.drive_dist:.1f} cm.")
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
        self.log.addline(f"Distance driven: {car.odometer} cm.")
        self.waypoints.extend(waypoints)
        self.log.addline(f"Heading after drive: {car.heading} deg.")
        self.log.addline(f"Ending Coords: {integerize(self.posn)}")


if __name__ == "__main__":

    trip = Trip()
    done = False
    while not done:
        done = trip.complete_one_leg()
    car.close()
