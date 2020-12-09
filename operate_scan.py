"""
Collect several sequential scans as car advances (stop / start).
"""

import map_scan_data
import omnicar
import time
import pickle

car = omnicar.OmniCar()
data_list = []
pause = 0.05  # sec pause time needed for wheel motor commands

def obliq1(spd, t):
    """Jog oblique quadrant 1 at spd (int 0-255) for t seconds
    """
    car.go_oblique1(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def obliq2(spd, t):
    """Jog oblique quadrant 2 at spd (int 0-255) for t seconds
    """
    car.go_oblique2(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def obliq3(spd, t):
    """Jog oblique quadrant 3 at spd (int 0-255) for t seconds
    """
    car.go_oblique3(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def obliq4(spd, t):
    """Jog oblique quadrant 4 at spd (int 0-255) for t seconds
    """
    car.go_oblique4(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

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

def cw(spd, t):
    """Spin car CW at spd (int between 100-255) for t seconds
    """
    car.spin_cw(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def ccw(spd, t):
    """Spin car CCW at spd (int between 100-255) for t seconds
    """
    car.spin_ccw(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

n = 6  # number of scans to collect
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
        #ccw(100, 0.85)  # roughly 45 degrees
        right(100, .5)

# scan data can be reloaded by operate_map.py, if needed.
with open('scan_data.pkl', 'wb') as f:
    pickle.dump(data_list, f)

# map the scans
for n, scan_data in enumerate(data_list):
    print()
    print(f"Scan number {n+1} of {len(data_list)}")
    map_scan_data.show_map(scan_data, n+1)
