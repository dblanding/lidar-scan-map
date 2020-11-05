"""Communicate with ZT car onboard arduino through serial port,
able to send commands to specify wheel motor speed & initiate scan.
Do a 180 deg scan, collect scan data points and save to file.
Analyze data pts and find set of best fit lines.
Plot points and lines. Show plot and save plot image.
"""

import math
import matplotlib.pyplot as plt
from matplotlib import style
import os
import serial
import time

style.use('fivethirtyeight')
filename = "scandata"
imagefile = filename + ".png"
    
# detection level thesholds
GAP = 10
CORNER = 8

# global values
xs = []  # list of x values of data points
ys = []  # list of y values of data points
nr_of_rows = 0 # Number of rows of data
direction = None  # +1 for 'CW'; -1 for 'CCW'

ports = ['/dev/ttyACM0', '/dev/ttyACM1']
for port in ports:
    if os.path.exists(port):
        ser = serial.Serial(port, 115200, timeout=None)
        ser.flush()
        break

# utility functions for working with 2D points and lines
# a point (x, y) is represented by its x, y coordinates
# a line (a, b, c) is represented by the eqn: ax + by + c = 0

def cnvrt_2pts_to_coef(pt1, pt2):
    """Return (a,b,c) coef of line defined by 2 (x,y) pts."""
    x1, y1 = pt1
    x2, y2 = pt2
    a = y2 - y1
    b = x1 - x2
    c = x2*y1-x1*y2
    return (a, b, c)

def proj_pt_on_line(line, pt):
    """Return point which is the projection of pt on line."""
    a, b, c = line
    x, y = pt
    denom = a**2 + b**2
    if not denom:
        return pt
    xp = (b**2*x - a*b*y -a*c)/denom
    yp = (a**2*y - a*b*x -b*c)/denom
    return (xp, yp)

def p2p_dist(p1, p2):
    """Return the distance between two points"""
    x, y = p1
    u, v = p2
    return math.sqrt((x-u)**2 + (y-v)**2)

def p2p_angle(p0, p1):
    """Return angle (degrees) from p0 to p1."""
    return math.atan2(p1[1]-p0[1], p1[0]-p0[0])*180/math.pi

def p2line_dist(pt, line):
    """Return perpendicular distance between pt & line"""
    p0 = proj_pt_on_line(line, pt)
    return p2p_dist(pt, p0)


class point():

    def __init__(self, dist, enc_val, lev=0, theta=0, x=0, y=0):
        self.dist = dist
        self.enc_val = enc_val
        sefl.lev = lev
        self.theta = theta
        self.x = x
        self.y = y


def read_data_line():
    if ser.in_waiting:
        try:
            read_line = ser.read_until()   # byte type
            line = read_line.decode("utf-8").strip()
        except UnicodeDecodeError:  
            line = None
        return line


def scan():
    # collect incoming scan data, return list
    datalen = 0
    data = []
    data_line = read_data_line()  # 'A' from previous command
    data_line = read_data_line()  # None
    while data_line != 'A':  #  Wait for trailing 'A'
        data_line = read_data_line()
        if data_line:
            if data_line != 'A':
                data.append(data_line)
                if len(data) != datalen:
                    print(data_line)
                    datalen = len(data)
    return data

def find_corners(regions):
    """Within each continuous region, find index and value of data point
    located farthest from the end_to_end line segment if value > CORNERS.
    If index(es) found, split region(s) at index & return True."""
    corners = []
    found = False
    for region in regions:
        max_dist = 0
        max_idx = 0
        idx1 = region[0]
        idx2 = region[-1]
        p1 = (xs[idx1], ys[idx1])
        p2 = (xs[idx2], ys[idx2])
        line = cnvrt_2pts_to_coef(p1, p2)
        for n in range(idx1, idx2):
            pnt = (xs[n], ys[n])
            dist = p2line_dist(pnt, line)
            if dist > max_dist:
                max_dist = dist
                max_idx = n
        if max_dist > CORNER:
            corners.append(max_idx)
    # Insert found corners into regions. Start with high values of index first
    # to avoid problems when popping and inserting into regions.
    if corners:
        found = True
        corners.reverse()
        for corner_index in corners:
            for n, region in enumerate(regions):
                if region[0] < corner_index < region[-1]:
                    start, end = regions.pop(n)
                    regions.insert(n, (start, corner_index))
                    regions.insert(n+1, (corner_index+1, end))
    return found

def find_segments():
    """
    Read scan data (list) in which each line contains 3
    comma separated values: direction, distance & encoder_count.
    Return regions, a list of pairs of index values representing
    the end points of straight line segments which fit the data.
    """
    global xs, ys, nr_of_rows, direction
    print("gap threshold = ", GAP)
    print("corner threshold = ", CORNER)
    distances = []  # List of distance values
    enc_cnts = []  # List of actual encoder count values
    for line in scan():
        str_dir, str_dist, str_enc_cnt = line.strip().split(', ')
        # map only data from center 180 degrees of rotation
        if 256 <= int(str_enc_cnt) <= 768:
            distances.append(int(str_dist))
            enc_cnts.append(int(str_enc_cnt))
            if int(str_dir) > 0:
                direction = 'CW'
            else:
                direction = 'CCW'
    nr_of_rows = len(distances)
    print("Number of data points = ", nr_of_rows)

    # linearize the encoder count values to remove jitter
    ec_start = enc_cnts[0]
    ec_end = enc_cnts[-1]
    ec_incr = float(enc_cnts[-1]-enc_cnts[0]) / (nr_of_rows - 1)
    lin_ec_vals = []  # List of linearized encoder count values
    start_cnt = float(enc_cnts[0])
    for n in range(nr_of_rows):
        lin_ec_vals.append(start_cnt + n * ec_incr)
    #print("Minimum encoder value = ", lin_ec_vals[0])
    #print("Maximum encoder value = ", lin_ec_vals[-1])

    # Calculate theta (increasing from 0 @ enc=768 to pi @ enc=256)
    thetas = []  # list of theta values
    for n in range(nr_of_rows):
        enc_val = lin_ec_vals[n]
        theta = 1.5 * math.pi * (1 - (enc_val / 768))
        thetas.append(theta)
    #print("Minimum theta value = ", thetas[0])
    #print("Maximum theta value = ", thetas[-1])

    # convert polar (dist, theta) coords to rect (x, y) coords
    for n in range(nr_of_rows):
        x = distances[n] * math.cos(thetas[n])
        y = distances[n] * math.sin(thetas[n])
        xs.append(x)
        ys.append(y)

    # Find continuous regions of closely spaced points (delta dist < GAP).
    # 'large' gaps (delta dist > GAP) represent edges of "continuous regions".
    # Record the index of the start & end points of these continuous regions.
    regions = []  # list of regions
    start_index = 0  # index of start of first region
    for n in range(1, nr_of_rows):
        gap = abs(distances[n] - distances[n-1])
        if gap > GAP:
            if n > (start_index + 1):
                regions.append((start_index, n-1))
            start_index = n
    regions.append((start_index, n))

    # Discard regions having only 2 or 3 points
    to_discard = []
    for n, region in enumerate(regions):
        if region[-1] - region[0] < 4:
            to_discard.append(n)
    to_discard.reverse()
    for n in to_discard:
        _ = regions.pop(n)
            
    print("Continuous regions: ", regions)

    # If the points in a continuous region are substantially straight & linear,
    # they can be well represented by a straight line segment between the start
    # and end points of the region. However, if the points trace an 'L' shape,
    # as they would where two walls meet at a corner, two straight line segments
    # would be needed, with the two segments meeting at the corner.
    # To find a corner in a region, look for the point at the greatest distance
    # from the straight line joining the region end points.
    # Only one corner is found at a time. To find multiple corners in a region
    # (of a zig-zag shaped wall, for example) make multiple passes.

    p = 0
    while find_corners(regions):
        p += 1
        print("Look for corners, pass %s:" % p)
        print("Regions: ", regions)
    return regions

if __name__ == "__main__":
    
    motorsOff = '0, 0, 0\n'.encode('utf-8')
    startscan = '0, 0, 2\n'.encode('utf-8')
    ser.write(motorsOff)

    # Start to accelerate wheel motors and back to zero
    m = 0
    up = True # Accelerating
    while (m>=0):
        if(ser.in_waiting):
            #read_serial = ser.readline()
            print(m, read_data_line())
            outString = "%i, %i, 0\n" % (m,m)
            ser.write(outString.encode('utf-8'))
            if up:
                m+=10
                if (m>20):
                    up = False
            else:
                m-=10
    ser.write(motorsOff)
    print(read_data_line())
    ser.write(startscan)
    print(read_data_line())
    segments = find_segments()
    
    # plot data points & line segments
    plt.scatter(xs, ys, color='#003F72')
    title = "(%s pts) " % nr_of_rows
    title += "GAP: %s, CORNER: %s" % (GAP, CORNER)                                                           
    plt.title(title)
    for segment in segments:
        start, end = segment
        x_vals = [xs[start], xs[end]]
        y_vals = [ys[start], ys[end]]
        plt.plot(x_vals, y_vals)
    plt.axis('equal')
    plt.savefig(imagefile)
    plt.show()
