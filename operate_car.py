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

def jog_FL(spd, t):
    """Jog diagonally forward + left at spd (int 0-255) for t seconds
    """
    car.go_FL(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_BL(spd, t):
    """Jog diagonally back + left at spd (int 0-255) for t seconds
    """
    car.go_BL(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_BR(spd, t):
    """Jog diagonally back + right at spd (int 0-255) for t seconds
    """
    car.go_BR(spd)
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

def jog_B(spd, t):
    """Jog backward at spd (int: 100-255) for t seconds
    """
    car.go_B(spd)
    time.sleep(t)
    car.stop_wheels()
    time.sleep(pause)

def jog_L(spd, t):
    """Jog left at spd (int: 100-255) for t seconds
    """
    car.go_L(spd)
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

def scan_series(nbr):
    """Perform a series of nbr successive scans, moving car between scans.
    Save scan data. Generate and save maps.
    """
    data_list = []  # list of scan data from multiple scans
    while nbr:
        scan_data = car.scan(low_enc_val=20000, hi_enc_val=30000)
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
            jog_R(100, 0.75)  # ccw(100, 0.85) is roughly 45 degrees
            # jog_F(100, 1.5, trim=5)

    # scan data can be reloaded by remap_scans(), if needed.
    with open('scan_data.pkl', 'wb') as f:
        pickle.dump(data_list, f)

    # map the scans
    for n, scan_data in enumerate(data_list):
        print()
        print(f"Scan number {n+1} of {len(data_list)}")
        map_scan_data.show_map(scan_data, n+1)

def remap_scans():
    """Load saved scan data and remap (with different map parameters).
    """
    with open('scan_data.pkl', 'rb') as f:
        data_list = pickle.load(f)

    for n, scan_data in enumerate(data_list):
        print()
        print(f"Scan number {n+1} of {len(data_list)}")
        map_scan_data.show_map(scan_data, n+1)

def drive_and_scan():
    """
    Drive parallel to wall on left keeping track of
    its parameters: length, angle, and x_dist
    length: distance from projection of car to the end of wall.
    angle: angle of the wall w/r/t car's X axis. taget = 90 degrees
    x_dist: lateral distance to the wall (in x direction)
    Start out initially so that x_dist >= min_dist
    Stop the car when the value of length drops below min_length.
    While driving, adjust steering trim to minimaize error.
    """
    data_list = []  # list of data from multiple scans
    target = 90  # angle (degrees)
    prev_angle = target
    # Before driving along wall, move the car sideways until it is at
    # least the minimum desired distance from the wall at left.
    scan_data = car.scan()
    data_list.append(scan_data)
    length, angle, x_dist = map_scan_data.analyze_data(scan_data)
    print(f"length: {length}")
    print(f"angle: {angle}")
    print(f"x_dist: {x_dist}")
    print()
    if abs(x_dist) < min_dist:  # too close to wall
        car.go_R(spd)
        time.sleep(pause)
        print("Driving Right")
        print()
    while abs(x_dist) < min_dist:  # get farther from wall
        scan_data = car.scan()
        data_list.append(scan_data)
        length, angle, x_dist = map_scan_data.analyze_data(scan_data)
        print(f"length: {length}")
        print(f"angle: {angle}")
        print(f"x_dist: {x_dist}")
        print()
        prev_angle = angle
    car.stop_wheels()
    print("Wheels Stopped")
    print()
    prev_angle = angle
    trim = 3  # estimate of correct steering trim
    time.sleep(pause)
    if length > min_length:  # drive toward end of wall
        car.go_F(spd, trim=trim)
        time.sleep(pause)
        print("Driving Forward")
        print()
    while length > min_length:  # continue while steering
        scan_data = car.scan()
        data_list.append(scan_data)
        length, angle, x_dist = map_scan_data.analyze_data(scan_data)
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


if __name__ == "__main__":
    #remap_scans()
    '''
    drive_and_scan()
    time.sleep(1)
    '''
    scan_series(6)
    car.close()
