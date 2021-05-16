"""
Find the angles of the most likely paths forward based on the
absence of obstructions in the scan points data.
Algorithm plots a black background image in which 1 pixel represents
1 cm with the car positioned at the center of the image, and the top
of image coresponding with the car's forward direction.
Scan points are plotted with X axis pointing to the right and Y axis
vertically up (forward). Angles are measured CCW from the X axis.
"""
import cv2
import numpy as np
import pickle
from pprint import pprint

ANG_STEP = 6 # size of angular step at which open-ness is tested
S = 500  # Image size (square) S x S
DISKRAD = 20  # Disk radius (based on robot size) for clearance
color = 255
thickness = -1  # line thickness (-1 means filled in)

def image_from_points(points):
    """Generate a square black image (size=SxS) w/ overlapping white
    circles (radius = DISKRAD) located at each point in points."""
    img = np.zeros((S, S), dtype = "uint8")
    # Points are in car coord sys (X straight ahead, Y to left)
    # But we want to define X to the right and Y straight ahead
    # with the origin at the center of the image.
    # Also, point coordinates must be integers, not floats.
    # cv2 origin is at the upper left corner of image with Y down.
    # All the above happens in code below:
    for point in points:
        x, y = point
        pnt = (-(int(y) - S//2), -(int(x) - S//2))
        img = cv2.circle(img, pnt, DISKRAD, color, thickness)
    return img
    

def make_mask(angle, dist):
    """Make a 1 pixel wide slit from image center to radial dist,
    oriented at angle (degrees) CCW from the X axis (on right).
    Return square black image (size S) with white slit."""
    img = np.zeros((S, S), dtype = "uint8")
    cv2.rectangle(img, (S//2, S//2), (S//2 + dist, 250), 255, -1)
    rotation_matrix = cv2.getRotationMatrix2D((S//2, S//2), angle, 1)
    img_rotation = cv2.warpAffine(img, rotation_matrix, (S, S))
    return img_rotation

def find_open_angles(scan_img, dist):
    """Test (at a range of angles) for obstruction between
    image center and point located at radial dist.
    Return list of angles where no obstruction is found."""
    open_angles = []
    # Test for obstruction-free path at many angles
    for angle in range(0, 226, ANG_STEP):
        mask_img = make_mask(angle, dist)
        scan_and_mask_img = scan_img & mask_img
        scan_or_mask_img = scan_img | mask_img
        non_zero = np.nonzero(scan_and_mask_img)
        a, *rest = non_zero
        if len(a) == 0:
            open_angles.append(angle)
    return open_angles

def find_open_paths(open_angles):
    """Find mid-line angles of contiguously 'clumped' open angles.
    Return mid-line angles (degrees) CCW from X-axis (on right)."""
    mid_angles = None
    clump = None
    clumps = []
    for ind, angle in enumerate(open_angles):
        if not ind:
            clump = [angle, ]
        elif angle == clump[-1] + ANG_STEP:
            clump.append(angle)
        else:
            clumps.append(clump)
            clump = [angle, ]
    if clump:
        clumps.append(clump) 
        mid_angles = [(clump[0]+clump[-1])/2 for clump in clumps]
    return mid_angles

def best_paths(points, dist):
    """Find angles of mid lines through open sectors."""
    image = image_from_points(points)
    open_angs = find_open_angles(image, dist)
    # find center angles of contiguous open sectors
    mid_angs = find_open_paths(open_angs)
    return mid_angs

if __name__ == "__main__":
    # There are points from 13 legs stored in 'scanpoints/'
    # The average distances are:
    avg_dists = [0, 185, 115, 93, 122, 117, 143, 151, 125,
                 130, 138, 160, 154, 171]
    leg_open_path_dict = {}
    for leg in range(1, 14):
        with open(f'scanpoints/leg{leg}.pkl', 'rb') as file:
            points = pickle.load(file)
        dist = avg_dists[leg]
        image = image_from_points(points)
        open_angs = find_open_angles(image, dist)
        # find center angles of contiguous open sectors
        mid_angs = find_open_paths(open_angs)
        leg_open_path_dict[leg] = mid_angs
    pprint(leg_open_path_dict)
    
