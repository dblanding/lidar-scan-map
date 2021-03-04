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
            print("Bumped into an obstacle!")
            car.stop_wheels()
            break
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
    """Turn (while stopped) to absolute target course (degrees)."""
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
    # Make overlay plot of most salient scan points on map
    mapper.plot(transformed_pnts,
                mapper.load_base_map(),
                carspot=curr_posn,
                seq_nmbr=nmbr)


if __name__ == "__main__":
    local_deviation = -4  # degrees
    n = 1
    heading = car.heading()
    print(f"Initial Heading = {heading} deg")
    home_angle = heading
    curr_posn = (0, 0)
    angle = home_angle - heading
    show_scan_overlay(curr_posn, angle, nmbr=n)

    # Drive ahead
    dist = 100  # dist to drive
    print()
    print(f"Driving ahead {dist} cm")
    drive_ahead(dist)
    time.sleep(1)

    # update current estimated position by dead reckoning
    r, theta = dist, (home_angle - heading + 90)
    dx, dy = geo.p2r(r, theta)
    x, y = curr_posn
    curr_posn = x+dx, y+dy
    heading = car.heading()
    print(f"Heading = {heading} deg")
    angle = home_angle - heading
    n += 1
    show_scan_overlay(curr_posn, angle, nmbr=n)

    # turn left 90 deg
    turn_target = home_angle - 90
    print()
    print(f"Command: Turn left to {turn_target}")
    turn_to(turn_target)
    time.sleep(0.5)
    turn_to(turn_target)
    heading = car.heading()
    print(f"Heading = {heading} deg after turn")
    angle = home_angle - heading
    n += 1
    show_scan_overlay(curr_posn, angle, nmbr=n)

    # Drive ahead
    dist = 140  # dist to drive
    print()
    print(f"Driving ahead {dist} cm")
    drive_ahead(dist)
    time.sleep(1)

    # update current estimated position by dead reckoning
    r, theta = dist, (home_angle - heading + 90)
    dx, dy = geo.p2r(r, theta)
    x, y = curr_posn
    curr_posn = x+dx, y+dy
    heading = car.heading()
    print(f"Heading = {heading} deg")
    angle = home_angle - heading + local_deviation
    print(f"Computed angle for scan = {angle} deg")
    angle = int(input("Enter measured angle of car: "))
    n += 1
    show_scan_overlay(curr_posn, angle, nmbr=n)
    
    # turn right 90 deg
    turn_target = home_angle + local_deviation
    print()
    print(f"Command: Turn right to {turn_target}")
    turn_to(turn_target)
    time.sleep(0.5)
    turn_to(turn_target)
    heading = car.heading()
    print(f"Heading = {heading} deg after turn")
    angle = home_angle - heading + local_deviation
    print(f"Computed angle for scan = {angle} deg")
    angle = int(input("Enter measured angle of car: "))
    n += 1
    show_scan_overlay(curr_posn, angle, nmbr=n)

    car.close()

