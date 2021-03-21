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
logger.setLevel(logging.DEBUG)  # set to DEBUG | INFO | WARNING | ERROR
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
    bearing; targets to the right have (-) relative bearings.
    If, for example, the car is on a heading of 0 degrees and intends
    to turn left to a target heading of -90 degrees, the relative
    bearing of the target is +90 degrees.
    This is consistent with the standard mathematics convention of
    measuring angles in the CCW direction as positive (+).
    """
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
            print("done")
            done = True

def R_xform(pntlist, angle):
    """Transform each point in pntlist by rotation angle (degrees)
    w/r/t the coordinate system origin.

    When applying both a rotation and translation transform, rotation
    must be applied first, while the car's center is located at the
    origin. This results in a pure rotation.
    If the points were first transformed in translation, rotation
    transformation would produce an unwanted motion through an arc.
    """

    result = []
    for pnt in pntlist:
        r, theta = geo.r2p(pnt)
        newpnt = geo.p2r(r, theta+angle)
        result.append(newpnt)
    return result

def T_xform(pntlist, tx, ty):
    """Transform each point in pntlist by translation in x and y."""

    return [(x+tx, y+ty)
            for x, y in pntlist]

def xform_pnts(pntlist, ang, tx, ty):
    """Return list of 2D transformed points, rotated & translated."""
    rotated = R_xform(pntlist, ang)
    translated = T_xform(rotated, tx, ty)
    return translated

def show_scan_overlay(curr_posn, angle, nmbr):
    print(f"Current position: {curr_posn}")
    print(f"angle for transform = {angle} deg")
    data = car.scan(spd=120)
    target_pnt = car.auto_detect_open_sector()
    pscan = proscan2.ProcessScan(data)
    longest_regions = pscan.regions_by_length()
    scanpoints = []  # xy coords of points in longest regions
    for regn_indx in longest_regions:
        points_in_region = pscan.get_points_in_region(regn_indx)
        if len(points_in_region) > 15:
            scanpoints.extend(points_in_region)
    transformed_pnts = xform_pnts(scanpoints, angle,
                                  curr_posn[0],
                                  curr_posn[1])
    transformed_target = xform_pnts([target_pnt], angle,
                                    curr_posn[0],
                                    curr_posn[1])[0]
    # Make overlay plot of most salient scan points on map
    mapper.plot(transformed_pnts, mapper.load_base_map(),
                target=transformed_target, carspot=curr_posn,
                seq_nmbr=nmbr)
    return target_pnt

class Trip():
    """multi-leg trip."""

    def __init__(self):
        car.reset_heading()
        self.initial_heading = 0
        self.nmbr = 0  # leg number
        self.data = None  # Scan data
        self.curr_posn = (0, 0)  # Current position of car
        self.curr_heading = 0  # Current heading of car
        self.theta = 0  # Angle to next target CCW from X axis (+)
        self.drive_angle = 0  # Angle to target CCW from Y axis (+)
        self.drive_dist = 0  # Distance to target point
        self.target_pnt = (0, 0)  # x, y coords of target point

    def complete_one_leg(self):
        self.nmbr += 1
        self.scan_plan()
        self.show_map()
        self.drive()

    def scan_plan(self):
        self.data = car.scan(spd=120)
        self.target_pnt = car.auto_detect_open_sector()
        # target_pnt to dist, theta for turn & drive
        dist, theta = geo.r2p(self.target_pnt)
        self.drive_angle = int(theta - 90)
        self.drive_dist = dist
        self.theta = theta

    def drive(self):
        print(f"Turning {self.drive_angle} degrees (+) CCW")
        turn_to_abs(car.heading() - self.drive_angle)
        # Update curr_heading after turning
        self.curr_heading = self.initial_heading - car.heading()
        print(f"Driving {self.drive_dist:.1f} cm")
        drive_ahead(self.drive_dist, spd=CARSPEED)
        # Update curr_posn after driving to target_pnt
        self.curr_posn = (self.curr_posn[0] + self.target_pnt[0],
                          self.curr_posn[1] + self.target_pnt[1])

    def show_map(self):
        pscan = proscan2.ProcessScan(self.data)
        longest_regions = pscan.regions_by_length()
        scanpoints = []  # xy coords of points in longest regions
        for regn_indx in longest_regions:
            points_in_region = pscan.get_points_in_region(regn_indx)
            if len(points_in_region) > 15:
                scanpoints.extend(points_in_region)
        transformed_pnts = xform_pnts(scanpoints,
                                      self.curr_heading,
                                      self.curr_posn[0],
                                      self.curr_posn[1])
        transformed_target = xform_pnts([self.target_pnt],
                                        self.curr_heading,
                                        self.curr_posn[0],
                                        self.curr_posn[1])[0]
        # Make overlay plot of most salient scan points on map
        mapper.plot(transformed_pnts, mapper.load_base_map(),
                    target=transformed_target,
                    carspot=self.curr_posn,
                    seq_nmbr=self.nmbr)


def drive_to_spot(spd=None):
    """
    Scan & display interactive map with proposed target spot shown.
    User then closes map and either enters 'y' to agree to proposed
    spot or 'c' to input coordinates of an alternate one.
    Car drives to spot. Repeat.
    """

    if not spd:
        spd = CARSPEED
    n = 0
    car.reset_heading()
    time.sleep(0.01)
    home_angle = car.heading()
    curr_posn = (0, 0)
    angle = 0
    while n < 2:
        # scan & display plot
        target = show_scan_overlay(curr_posn, angle, nmbr=n)
        # convert x,y to dist, theta for turn & drive
        dist, theta = geo.r2p(target)
        target_angle = int(theta - 90)
        print(f"Turning {target_angle} degrees")
        turn_to_abs(car.heading()-target_angle)
        print(f"Driving {r:.1f} cm")
        drive_ahead(dist, spd=spd)
        # Update curr_posn, angle
        
        n += 1

if __name__ == "__main__":
    car.reset_heading()
    trip = Trip()
    for _ in range(2):
        trip.complete_one_leg()

    car.close()
