"""Collection of car operation commands
"""
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
            print("Bumped into an obstacle!")
            car.stop_wheels()
            break
    car.stop_wheels()

def T_xform(pntlist, tx, ty):
    """Transform each point in pntlist by translation in x and y."""

    return [(x+tx, y+ty)
            for x, y in pntlist]

def R_xform(pntlist, angle):
    """Transform each point in pntlist by rotation angle (degrees)."""

    result = []
    for pnt in pntlist:
        r, theta = geo.r2p(pnt)
        newpnt = geo.p2r(r, theta+angle)
        result.append(newpnt)
    return result
    
if __name__ == "__main__":
    n = 1
    while n:
        print(car.heading())
        data = car.scan(spd=120)
        pscan = proscan2.ProcessScan(data)
        longest_regions = pscan.regions_by_length()
        scanpoints = []  # xy coords of points in longest regions
        for regn_indx in longest_regions:
            points_in_region = pscan.get_points_in_region(regn_indx)
            if len(points_in_region) > 15:
                scanpoints.extend(points_in_region)
        scanpoints_R = R_xform(scanpoints, 90)
        scanpoints_T = T_xform(scanpoints_R, 0, 100)
        '''
        print(f"Length of segment = {pnts_in_seg} pnts")
        coords1 = pscan.points[segment[0]].get("xy")
        coords2 = pscan.points[segment[1]].get("xy")
        print(f"Coords1: {coords1}")
        print(f"Coords2: {coords2}")
        line = geo.cnvrt_2pts_to_coef(coords1, coords2)
        loc_xy = geo.proj_pt_on_line(line, (0,0))
        loc_vctr = geo.r2p(loc_xy)
        print(f"Location vector: {loc_vctr}")
        '''
        mapper.plot(scanpoints_T, mapper.load_base_map())
        #drive_ahead(100)
        n -= 1
    car.close()

