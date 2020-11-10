"""Communicate with ZT car onboard arduino through serial port,
Send commands to specify wheel motor speed & initiate scan.
Do a 180 deg scan and collect scan data points.
Analyze scan data points and find set of best fit lines.
Plot points and lines. Display and save plot image.
"""

import math
import matplotlib.pyplot as plt
from matplotlib import style
import os
from pprint import pprint
import serial

style.use('fivethirtyeight')
filename = "scandata"
imagefile = filename + ".png"

# detection level thesholds
GAP = 10
CORNER = 8

# global values
points = []  # list of point type items
nr_of_rows = 0  # Number of rows of scan data

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


class Point():
    """Convenience structure encapsulating data point measured values

    (int type) and calculated values (float type)"""

    def __init__(self, dist, enc_val, lev=0, theta=0, xy=(0, 0)):
        self.dist = dist  # integer measured distance (cm)
        self.enc_val = enc_val  # integer angle encoder value
        self.lev = lev  # float linearized encoder value
        self.theta = theta  # (derived) float angle wrt car (radians)
        self.xy = xy  # (x, y) coordinates


def read_data_line():
    if ser.in_waiting:
        try:
            read_line = ser.read_until()   # byte type
            line = read_line.decode("utf-8").strip()
        except UnicodeDecodeError:
            line = None
        return line


def scan():
    """collect incoming scan data, return list of string types"""
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
    If index found, split region(s) at index & return True."""
    corners = []
    found = False
    for region in regions:
        max_dist = 0
        max_idx = 0
        idx1 = region[0]
        idx2 = region[-1]
        p1 = (points[idx1].xy)
        p2 = (points[idx2].xy)
        line = cnvrt_2pts_to_coef(p1, p2)
        for n in range(idx1, idx2):
            pnt = (points[n].xy)
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
    Populate global points (list of points) and return
    regions, a list of pairs of index values representing
    the end points of straight line segments which fit the data.
    """
    global points, nr_of_rows
    print("gap threshold = ", GAP)
    print("corner threshold = ", CORNER)
    for line in scan():
        str_dir, str_dist, str_enc_cnt = line.strip().split(', ')
        # map only data from center 180 degrees of rotation
        if 256 <= int(str_enc_cnt) <= 768:
            pnt = Point(int(str_dist), int(str_enc_cnt))
            points.append(pnt)
    nr_of_rows = len(points)
    print("Number of data points = ", nr_of_rows)

    # Linearize the encoder count values to remove ADC jitter
    # and use linearized values to calculate theta values.
    # encoder count values go from 0/1023 (straight behind)
    # and increase with CW rotation.
    # straight left: enc_val = 256; theta = pi
    # straight ahead: enc_val = 512; theta = pi/2
    # straight right: enc_val = 768; theta = 0
    ec_start = points[0].enc_val
    ec_end = points[-1].enc_val
    ec_incr = float(ec_end - ec_start) / (nr_of_rows - 1)
    for n, pnt in enumerate(points):
        lev = float(ec_start + (n * ec_incr))
        pnt.lev = lev
        theta = 1.5 * math.pi * (1 - (lev / 768))
        pnt.theta = theta

    # Remove points whose dist value == 1200 (max value)
    cull_list = []
    for n, pnt in enumerate(points):
        if pnt.dist == 1200:
            cull_list.append(n)
    cull_list.reverse()
    for n in cull_list:
        _ = points.pop(n)

    # convert polar coords (dist, theta) to (x, y) coords
    for p in points:
        x = p.dist * math.cos(p.theta)
        y = p.dist * math.sin(p.theta)
        p.xy = (x, y)

    # Find continuous regions of closely spaced points (delta dist < GAP).
    # 'large' gaps (delta dist > GAP) represent edges of "continuous regions".
    # Record the index of the start & end points of these continuous regions.
    regions = []  # list of regions
    start_index = 0  # index of start of first region
    gap = 0
    for n, pnt in enumerate(points):
        if n:  # skip first point because there is no 'previous'
            gap = abs(pnt.dist - points[n-1].dist)
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

    motors_off = '0, 0, 0\n'.encode('utf-8')
    start_scan = '0, 0, 2\n'.encode('utf-8')
    ser.write(motors_off)

    # Start to accelerate wheel motors and back to zero
    m = 0
    up = True # Accelerating
    while m >= 0:
        if ser.in_waiting:
            #read_serial = ser.readline()
            print(m, read_data_line())
            out_string = "%i, %i, 0\n" % (m, m)
            ser.write(out_string.encode('utf-8'))
            if up:
                m += 10
                if m > 30:
                    up = False
            else:
                m -= 10
    ser.write(motors_off)
    print(read_data_line())
    ser.write(start_scan)
    print(read_data_line())
    segments = find_segments()

    # plot data points & line segments
    xs = []
    ys = []
    for pnt in points:
        x, y = pnt.xy
        xs.append(x)
        ys.append(y)
    plt.scatter(xs, ys, color='#003F72')
    title = "(%s pts) " % nr_of_rows
    title += "GAP: %s, CORNER: %s" % (GAP, CORNER)
    plt.title(title)
    line_coords = []
    for segment in segments:
        start, end = segment
        line_coords.append
        x_vals = [xs[start], xs[end]]
        y_vals = [ys[start], ys[end]]
        line_coords.append(((xs[start], ys[start]), (xs[end], ys[end])))
        plt.plot(x_vals, y_vals)
    pprint(line_coords)
    plt.axis('equal')
    plt.savefig(imagefile)
    plt.show()
