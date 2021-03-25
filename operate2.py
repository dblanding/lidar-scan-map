"""Top level operating script for omniwheel vehicle."""

import logging
import math
from pathlib import Path
import pickle
from pprint import pprint
import sys
import time
from constants import CARSPEED, FWD, SONAR_STOP, PIDTRIM
import geom_utils as geo
import mapper
import omnicar as oc
import proscan2

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

def drive_ahead(dist, spd=CARSPEED):
    """Drive dist and stop, w/out closed-loop steering feedback."""
    # drive
    rate = get_rate(spd)  # cm/sec
    time_to_travel = dist / rate
    start = time.time()
    delta_t = 0
    while delta_t < time_to_travel:
        delta_t = time.time() - start
        trim = PIDTRIM
        sonardist, *_ = car.go(spd, FWD, spin=trim)
        if sonardist < SONAR_STOP:
            print(f"Bumped into an obstacle at {sonardist}cm")
            car.stop_wheels()
            break
    car.stop_wheels()

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
    done = False
    while not done:
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
            done = True

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
    As car moves through its 'world', we need to transform 2D coords
    between car coord sys (CCS) & world coord sys (WCS)."""

    def __init__(self):
        car.reset_heading()
        self.nmbr = 0  # leg number
        self.data = None  # Scan data
        self.posn = (0, 0)  # Position of car (in WCS)
        self.heading = -car.heading()  # Current heading of car
        self.theta = 0  # Angle to next target (CCW from X axis)
        self.drive_angle = 0  # Angle to target CCW from Y axis
        self.drive_dist = 0  # Distance to target point
        self.rel_trgt_pnt = (0, 0)  # target point (CCS)
        self.transformed_target = (0, 0)  # target point (WCS)

    def complete_one_leg(self):
        """Auto sequence multiple legs of trip."""
        self.nmbr += 1
        print(f"Leg {self.nmbr}  ")
        print(f"Coords: {self.posn}  ")
        self.scan_plan()
        self.show_map()
        char = input("Enter y to continue: ")
        if char not in "yY":
            return True
        self.drive()
        return False

    def scan_plan(self):
        """Scan and compute target in open sector."""
        self.data = car.scan(spd=120)
        self.rel_trgt_pnt = car.auto_detect_open_sector()
        # convert target_pnt to dist, theta for turn & drive
        dist, theta = geo.r2p(self.rel_trgt_pnt)
        self.drive_angle = int(theta - 90)
        self.drive_dist = dist
        self.theta = theta

    def show_map(self):
        """Show (most salient) scan points overlayed on a map.
        Also show proposed target point (yellow)
        and current car position (red)."""
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
                    target=transformed_target,
                    carspot=self.posn,
                    seq_nmbr=self.nmbr)
        self.transformed_target = transformed_target

    def drive(self):
        """Drive to target, then update current position & heading."""
        if self.drive_angle < 0:
            print(f"Turning Right {-self.drive_angle} degrees.  ")
        else:
            print(f"Turning Left {self.drive_angle} degrees.  ")
        turn_to_abs(car.heading() - self.drive_angle)
        # Calculate next postion based on heading after turn
        hdng_strt = -car.heading()
        self.heading = hdng_strt
        print(f"Driving {self.drive_dist:.1f}cm on heading {hdng_strt}deg  ")
        drive_ahead(self.drive_dist, spd=CARSPEED)
        # Update values of self.posn and self.heading
        hdng_end = -car.heading()
        self.posn = self.next_posn()
        self.heading = hdng_end
        print(f"New heading = {self.heading}deg  ")
        print(f"Next position = {self.posn}  ")
        print()

    def next_posn(self):
        """Calculate next waypoint based on the car's actual heading.

        When driving to the next target pos'n, the car first turns in
        place, then drives the prescribed distance. However, when the
        car turns, it doesn't go exactly as commanded. It may be off
        a degree or two. Because we are dead reckoning the car's
        postion, the estimate of each successive waypoint is improved
        by using the actual heading rather than the target value."""
        # Revise calculation of next target based on actual heading
        theta = 90 - car.heading()
        theta = normalize_angle(theta)
        rel_trgt_pnt = geo.p2r(self.drive_dist, theta)
        dx, dy = rel_trgt_pnt  # relative coords of target point
        px, py = self.posn  # coords of current position
        return (dx + px, dy + py)


if __name__ == "__main__":
    car.reset_heading()
    trip = Trip()
    done = False
    while not done:
        done = trip.complete_one_leg()
    car.close()
