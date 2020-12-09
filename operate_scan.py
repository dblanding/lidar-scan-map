"""
Collect several sequential scans as car advances.
"""

import map_scan_data
import omnicar
import time
import pickle

car = omnicar.OmniCar()
data_list = []
pause = 0.05  # sec pause time needed for wheel motor commands

def fwd(spd, t):
    """Drive car forward at spd (int between 100-255) for t seconds
    """
    car.go_fwd(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def back(spd, t):
    """Drive car backward at spd (int between 100-255) for t seconds
    """
    car.go_back(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def left(spd, t):
    """Drive car left at spd (int between 100-255) for t seconds
    """
    car.go_left(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def right(spd, t):
    """Drive car right at spd (int between 100-255) for t seconds
    """
    car.go_right(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

n = 3  # number of scans to collect
while n:
    scan_data = car.scan()
    print(scan_data[0])
    print("Number of data points: ", len(scan_data))
    scan_data.pop(0)
    print(scan_data[0])
    print("Number of data points: ", len(scan_data))
    data_list.append(scan_data)
    n -= 1
    if n:
        time.sleep(pause)
        fwd(100, 0.5)

# scan data can be reloaded by operate_map.py, if needed.
with open('scan_data.pkl', 'wb') as f:
    pickle.dump(data_list, f)

# map the scans
for n, scan_data in enumerate(data_list):
    map_scan_data.show_map(scan_data, n+1)
