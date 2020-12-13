"""Collection of car operation commands
"""

import map_scan_data
import omnicar
import time
import pickle

car = omnicar.OmniCar()

pause = 0.05  # sec pause time needed between wheel motor commands
spd = 100  # car speed
min_length = 20  # threshold min length to end of wall
min_dist = 30  # threshold min distance from car to wall
kp = 0.5  # steering proportional PID coefficient
kd = 0.4  # steering derivative PID coefficient

def jog_FR(spd, t, trim=5):
    """Jog diagonally forward + right at spd (int 0-255) for t seconds
    """
    car.go_FR(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_FL(spd, t, trim=5):
    """Jog diagonally forward + left at spd (int 0-255) for t seconds
    """
    car.go_FL(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_BL(spd, t, trim=5):
    """Jog diagonally back + left at spd (int 0-255) for t seconds
    """
    car.go_BL(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_BR(spd, t, trim=5):
    """Jog diagonally back + right at spd (int 0-255) for t seconds
    """
    car.go_BR(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_F(spd, t, trim=5):
    """Jog forward at spd (int: 100-255) for t seconds
    trim (int) is used to null out any unwanted spin.
    """
    car.go_F(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_B(spd, t, trim=5):
    """Jog backward at spd (int: 100-255) for t seconds
    trim (int) is used to null out any unwanted spin.
    """
    car.go_B(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_L(spd, t, trim=5):
    """Jog left at spd (int: 100-255) for t seconds
    trim (int) is used to null out any unwanted spin.
    """
    car.go_L(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_R(spd, t, trim=7):
    """Jog right at spd (int: 100-255) for t seconds
    trim (int) is used to null out any unwanted spin.
    """
    car.go_R(spd, trim)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_cw(spd, t):
    """Spin car CW at spd (int between 100-255) for t seconds
    """
    car.spin_cw(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_ccw(spd, t):
    """Spin car CCW at spd (int between 100-255) for t seconds
    """
    car.spin_ccw(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def scan_series(nbr, lev=10000, hev=30000):
    """Perform a series of nbr successive scans of angular sector from
    lev (low encoder value) to hev (high encoder value), moving the car
    between scans. Save the raw scan data as a pickle file.
    Generate & save map plots.
    """
    data_list = []  # list of scan data from multiple scans
    while nbr:
        scan_data = car.scan(low_enc_val=lev, hi_enc_val=hev)
        print(scan_data[0])
        print("Number of data points: ", len(scan_data))
        scan_data.pop(0)  # may have 'stale' serial data
        print(scan_data[0])
        print("Number of data points: ", len(scan_data))
        data_list.append(scan_data)
        nbr -= 1
        if nbr:
            time.sleep(pause)
            # Move the car between successive scans
            # jog_R(100, 0.75)
            jog_F(100, 1.0, trim=5)

    # Save scan data for later examination, if desired.
    with open('scan_data.pkl', 'wb') as f:
        pickle.dump(data_list, f)

    # map the scans
    for n, scan_data in enumerate(data_list):
        print()
        print(f"Scan number {n+1} of {len(data_list)}")
        map_scan_data.show_map(scan_data, n+1)

def remap_scans():
    """Load saved scan data and generate maps.
    """
    modes = ('straight', 'corner')
    for mode in modes:
        if mode == 'straight':
            data_file = 'scan_data.pkl'
            map_folder = 'sideMaps'
        elif mode == 'corner':
            data_file = 'corner_data.pkl'
            map_folder = 'cornerMaps'
        with open(data_file, 'rb') as f:
            data_list = pickle.load(f)

        for n, scan_data in enumerate(data_list):
            print()
            print(f"Scan number {n+1} of {len(data_list)}")
            map_scan_data.show_map(scan_data, map_folder, n+1)

def drive_and_scan():
    """
    Drive parallel to wall on left keeping track of its parameters:
        length: distance from projection of car to the end of wall.
        angle: angle of the wall w/r/t car's X axis. target = 90 degrees
        x_dist: lateral distance to the wall (in x direction)
    Start out initially so that x_dist >= min_dist
    Stop the car when the value of length drops below min_length.
    While driving, adjust steering trim to minimize angle error.
    """
    data_list = []  # list of data from multiple scans
    target = 90  # angle (degrees)

    # Before driving along wall, move the car sideways until it is
    # at least the minimum desired distance from the wall at left.
    scan_data = car.scan(low_enc_val=10000, hi_enc_val=20000)
    data_list.append(scan_data)
    start_idx, end_idx, length, angle, x_dist = map_scan_data.analyze_data(scan_data)
    threshold_idx = (start_idx + end_idx) / 2
    print(f"length: {length}")
    print(f"angle: {angle}")
    print(f"x_dist: {x_dist}")
    print()
    if abs(x_dist) < min_dist:  # too close to wall
        car.go_R(spd)
        time.sleep(pause)
        print("Driving away from wall")
        print()
        while abs(x_dist) < min_dist:  # move farther from wall
            scan_data = car.scan(low_enc_val=10000, hi_enc_val=20000)
            data_list.append(scan_data)
            start_idx, end_idx, length, angle, x_dist = map_scan_data.analyze_data(scan_data)
            threshold_idx = (start_idx + end_idx) / 2
            print(f"length: {length}")
            print(f"angle: {angle}")
            print(f"x_dist: {x_dist}")
            print()

    car.stop_wheels()
    print("Wheels Stopped")
    print()

    prev_angle = angle
    trim = 3  # estimate of correct steering trim
    time.sleep(pause)
    if length > min_length:  # drive along wall
        car.go_F(spd, trim=trim)
        time.sleep(pause)
        print("Driving along wall")
        print()
    while length > min_length:  # continue while steering
        scan_data = car.scan(low_enc_val=10000, hi_enc_val=20000)
        data_list.append(scan_data)
        start_idx, end_idx, length, angle, x_dist = map_scan_data.analyze_data(scan_data)
        if start_idx > threshold_idx:
            print("Left wall gone from view: Exiting loop")
            break
        else:
            threshold_idx = (start_idx + end_idx) / 2
        print(f"length: {length}")
        print(f"angle: {angle}")
        print(f"x_dist: {x_dist}")
        print()
        # Adjust trim as needed to keep car driving along left wall
        # This means keep angle equal to target using pid feedback
        p_term = angle - target
        d_term = angle - prev_angle
        prev_angle = angle
        adjustment = int((p_term * kp + d_term * kd))
        trim += adjustment
        print(f"p_term = {p_term}, d_term = {d_term}, trim = {trim}")
        car.go_F(spd, trim=trim)
    car.stop_wheels()
    time.sleep(pause)
    print("Wheels Stopped")
    print()
    # Save collected scan data
    # Use remap_scans() to map it.
    with open('scan_data.pkl', 'wb') as f:
        pickle.dump(data_list, f)

def cross_corner_scan():
    """
    Traverse corner from the end of one wall to the start of the next:
        drive forward while scanning
        watch for angle to flip from 45 (old wall) to 135 (new wall)
        then keep going a little farther
    """
    data_list = []  # list of data from multiple scans
    
    scan_data = car.scan(low_enc_val=10000, hi_enc_val=20000)
    data_list.append(scan_data)
    strt_idx, end_idx, length, angle, x_dist = map_scan_data.analyze_data(scan_data)
    print(f"length: {length}")
    print(f"angle: {angle}")
    print(f"x_dist: {x_dist}")
    print()
    car.go_F(spd, trim=7)
    time.sleep(pause)
    print("Driving across corner")
    print()
    xtra_loops = 2  # number of extra times to go through loop below
    more = 0  # flag to go through loop below more times
    while angle < 90 or more < xtra_loops:
        scan_data = car.scan(low_enc_val=10000, hi_enc_val=20000)
        data_list.append(scan_data)
        strt_idx, end_idx, length, angle, x_dist = map_scan_data.analyze_data(scan_data)
        print(f"length: {length}")
        print(f"angle: {angle}")
        print(f"x_dist: {x_dist}")
        print()
        if angle > 90:
            more += 1
    car.stop_wheels()
    time.sleep(pause)
    print("Wheels Stopped")
    print()
    # Save collected scan data
    # Use remap_scans() to map it.
    with open('corner_data.pkl', 'wb') as f:
        pickle.dump(data_list, f)


if __name__ == "__main__":

    nmbr_of_loops = 1
    while nmbr_of_loops:
        drive_and_scan()
        print("Turning 45 degrees CCW")
        jog_ccw(100, 1.1)  # roughly 45 degrees
        cross_corner_scan()
        print("Turning 45 degrees CCW")
        jog_ccw(100, 1.1)  # roughly 45 degrees
        nmbr_of_loops -= 1

    remap_scans()
    car.close()
