"""Collection of car operation commands
"""
import logging
import math
from pathlib import Path
import pickle
from pprint import pprint
import sys
import time
from constants import *
import geom_utils as geo
import omnicar as oc
import proscan2

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

car = oc.OmniCar()
time.sleep(0.5)
from_arduino = car._read_serial_data()
logger.debug(f"Message from Arduino: {from_arduino}")

if __name__ == "__main__":
    print(car.heading())
    data = car.scan(spd=120)
    pscan = proscan2.ProcessScan(data)
    print("Regions")
    pprint(pscan.regions)
    print("Regions by length")
    longest_regions = pscan.regions_by_length()
    pprint(longest_regions)
    print("Segements in longest region")
    pprint(pscan.segments_in_region(longest_regions[0]))
    print("Segements in 2nd longest region")
    pprint(pscan.segments_in_region(longest_regions[1]))
    
    pscan.map(show=True)
    car.close()

