"""Top level operating script for omni-wheel vehicle."""

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
logger.debug(f"Message from Arduino: {from_arduino}")

def get_rate(speed):
    """Return rate (cm/sec) for driving FWD @ speed.
    Determined empirically for carspeed = 200, batt_charge >= 94%
    and distances from 50 - 200 cm.
    """
    return speed*0.155 - 6.5

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
    rel_brng = int(car.heading() - target)
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
        while heading_error > 2:
            spd = heading_error + 40
            car.spin(spd)
            heading_error = relative_bearing(target)
            logger.debug(f"relative heading: {heading_error} deg")
        car.stop_wheels()
        while heading_error < -2:
            spd = heading_error - 40
            car.spin(spd)
            heading_error = relative_bearing(target)
            logger.debug(f"heading error: {heading_error} deg")
        car.stop_wheels()
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
    """Scan, plan, map & drive multi-leg trip, starting from 'home'.

    Home position is WCS (0, 0) at a heading angle = 0
    As car moves through its 'world', we need to transform 2D coords
    between car coord sys (CCS) & world coord sys (WCS)."""

    def __init__(self):
        car.reset_heading()
        self.nmbr = 0  # Leg number
        self.data = None  # Scan data
        self.posn = (0, 0)  # Position of car (in WCS)
        self.heading = -car.heading()  # Current heading of car
        self.theta = 0  # Angle to next target (CCW from car X axis)
        self.drive_angle = 0  # Angle to target CCW from car Y axis
        self.drive_dist = 0  # Distance to target point
        self.rel_trgt_pnt = (0, 0)  # target point (CCS)
        self.waypoints = []  # list of waypoints visited
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
        drive_angle = normalize_angle(theta - 90)
        self.drive_angle = int(drive_angle)
        self.drive_dist = dist
        self.theta = theta

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
        if self.drive_angle < 0:
            self.log.addline(f"Turning Right {-self.drive_angle} deg.")
        else:
            self.log.addline(f"Turning Left {self.drive_angle} deg.")
        turn_to_abs(car.heading() - self.drive_angle)
        self.heading = -car.heading()
        self.log.addline(f"Heading after turn: {self.heading} deg.")

    def drive_to_target(self, spd=CARSPEED):
        """Drive forward self.drive_dist toward target
        updating self.posn incrementally along the way."""
        self.log.addline(f"Driving {self.drive_dist:.1f} cm.")
        trim = PIDTRIM
        rate = get_rate(spd)  # cm/sec
        time_to_travel = self.drive_dist / rate
        start_time = time.time()
        elapsed_time = 0
        prev_time = start_time
        delta_t = 0
        while elapsed_time < time_to_travel:
            self.heading = -car.heading()
            curr_time = time.time()
            delta_t = curr_time - prev_time
            elapsed_time = curr_time - start_time
            prev_time = curr_time
            sonardist, *_ = car.go(spd, FWD, spin=trim)
            if sonardist < SONAR_STOP:
                self.log.addline(f"Bumped into obstacle at {sonardist} cm")
                car.stop_wheels()
                break
            dx = rate * delta_t * math.cos((self.heading+90)*math.pi/180)
            dy = rate * delta_t * math.sin((self.heading+90)*math.pi/180)
            x, y = self.posn
            self.posn = (x + dx, y + dy)
        car.stop_wheels()
        self.log.addline(f"Final heading = {self.heading} deg.")
        self.log.addline()


if __name__ == "__main__":

    trip = Trip()
    done = False
    while not done:
        done = trip.complete_one_leg()
    car.close()
