"""
Collect scans while the car is moving.
"""

import map_scan_data
import omnicar
import time
import pickle

car = omnicar.OmniCar()

pause = 0.05  # sec pause time needed for wheel motor commands
spd = 80  # car speed
min_length = 15  # threshold minimum length of wall
min_dist = 30  # threshold min distance to wall

def drive_and_scan():
    """
    Drive parallel to wall on the left keeping track of the viewed
    length, angle, and location of the wall. Stop the car when
    the viewed length of the wall drops below min_length.
    """
    scan_data = car.scan()
    length, angle, x_value = map_scan_data.analyze_data(scan_data)
    print(f"length: {length}")
    print(f"angle: {angle}")
    print(f"x_value: {x_value}")
    print()
    if abs(x_value) < min_dist:
        car.go_right(spd)
        time.sleep(pause)
        print("Driving Right")
        print()
    while abs(x_value) < min_dist:
        scan_data = car.scan()
        length, angle, x_value = map_scan_data.analyze_data(scan_data)
        print(f"length: {length}")
        print(f"angle: {angle}")
        print(f"x_value: {x_value}")
        print()
    car.stop_wheels()
    print("Wheels Stopped")
    print()
    time.sleep(pause)
    if length > min_length:
        car.go_fwd(spd)
        time.sleep(pause)
        print("Driving Forward")
        print()
    while length > min_length:
        scan_data = car.scan()
        length, angle, x_value = map_scan_data.analyze_data(scan_data)
        print(f"length: {length}")
        print(f"angle: {angle}")
        print(f"x_value: {x_value}")
        print()
    car.stop_wheels()
    time.sleep(pause)
    print("Wheels Stopped")
    print()
    
drive_and_scan()
time.sleep(1)
car.close()
