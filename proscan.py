import logging
import math
import matplotlib.pyplot as plt
from matplotlib import style
import sys
import omnicar as oc

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

style.use('fivethirtyeight')

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

def group(data):
    """lidar has the habit of producing repeated distance values for
    adjacent points, 'triplets' being common, occasionally 'twins'.
    This function groups adjacent points with the same dist value
    into groups of triplets or twins."""
    group = []
    grouped_data = []
    for record in data:
        if not group:  # Initial pass through for loop
            dist = record[1]
            group.append(record[:2])
        elif len(group) == 3:  # cap group size at 3
            grouped_data.append(group)
            dist = record[1]
            group = [record[:2]]
        elif dist == record[1]:
            group.append(record[:2])
        else:
            grouped_data.append(group)
            dist = record[1]
            group = [record[:2]]
    grouped_data.append(group)  # append final group
    return grouped_data

def cull_repeats(grouped_data):
    """Filters all but one from each group of triplets or twins."""
    thindata = []
    for group in grouped_data:
        idx = len(group) // 2  # index of record to keep
        thindata.append(group[idx])
    return thindata


class Point():
    """Convenience structure encapsulating data point measured values
    (int type) and calculated values (float type)"""

    def __init__(self, dist, enc_val, theta=0, xy=(0, 0)):
        self.dist = dist  # integer measured distance (cm)
        self.enc_val = enc_val  # angle encoder value (int)
        self.theta = theta  # (derived) float angle wrt car (radians)
        self.xy = xy  # (x, y) coordinates


# Default values used in ProcessScan
GAP = 12  # Threshold distance between adjacent points for continuity
FIT = 3  # Threshold distance (point to line) for good fit

class ProcessScan():
    """
    Generate points and find set of best fit lines from scan data.
    Plot points and/or lines. Display and/or save plot image.
    """

    def __init__(self, data, lev=None, hev=None, gap=None, fit=None):
        """Generate list of Point objects from (scan) data.
        Optionally specify: sector of interest
        from lev (low encoder value) to hev (high encoder value),
        gap (threshold distance between adjacent points for continuity),
        fit (threshold point to line distance to qualify as 'good' fit).
        """
        if lev:
            self.LEV = lev
        else:
            self.LEV = oc.LEV
        if hev:
            self.HEV = hev
        else:
            self.HEV = oc.HEV
        if gap:
            self.GAP = gap
        else:
            self.GAP = GAP
        if fit:
            self.FIT = fit
        else:
            self.FIT = FIT
        self.points = []
        self.regions = []
        self.segments = []
        self._generate_points(data)
        self._generate_regions()

    def _cull_regions(self):
        """Kind of like linting for regions..."""

        # Remove regions having fewer than 4 points
        to_remove = []
        for region in self.regions:
            if (region[-1] - region[0]) < 4:
                to_remove.append(region)
        logger.debug(f"Removed {len(to_remove)} tiny regions")
        for region in to_remove:
            self.regions.remove(region)

        # Find and remove regions with only 1 point
        for region in self.regions:
            if region[0] == region[-1]:
                self.regions.remove(region)
                logger.debug("Removed a region with only 1 point")

    def _find_corners(self, region):
        """
        If the points in a continuous region are substantially straight &
        linear, they can be well represented by a straight line segment
        between the start and end points of the region.
        However, if the points in a region trace an 'L' or 'U' shape, as
        they would where walls meet at corners, we would expect to find
        multiple straight lines, with those lines intersecting at corners.

        Two obvious places to initiate a search for a straight line are
        the end points of the region.
        Return list of indexes at found corners.
        """
        idx0, idx1 = region  # indexes of region end points
        corners = []
        # find local minimum in region
        #local_min_list = self._find_local_min(idx0, idx1)
        corner_idx = self._find_line_segment(idx0, idx1)
        corners.append(corner_idx)
        corner_idx = self._find_line_segment(idx1, idx0)
        corners.append(corner_idx)
        return corners

    def _find_line_segment(self, begin_idx, end_idx):
        """
        Starting with the point at begin_idx and moving toward end_idx,
        return the index of the last point in a series of adjacent points
        (up to end_idx) which 'fit' the straight line segment drawn from
        begin to end. The method is:
        Starting with two adjacent points, test the fit of the points with
        the line joining the points. Of course the fit wil be perfect with
        only two points. Now, gradually extend the line by adding the next
        point and again test the fit of all points with the line. Continue
        to do this until the line cannot be made any longer and still have
        a good fit.
        Return the index of the last point that fits the straight line.
        """
        if begin_idx < end_idx:  # start at start_idx and go up
            istrt = begin_idx
            istop = begin_idx
            avg_dist = 0
            while avg_dist < self.FIT:
                istop += 1
                if istop > end_idx:
                    break
                line = cnvrt_2pts_to_coef(self.points[istrt].xy,
                                          self.points[istop].xy)
                avg_dist, cum_dsqr = self._find_sum_of_sq_dist_to_line(line,
                                                                       istrt,
                                                                       istop)
            istop -= 1
        elif begin_idx > end_idx:  # start at end_idx and go down
            istrt = begin_idx
            istop = begin_idx
            avg_dist = 0
            while avg_dist < self.FIT:
                istop -= 1
                if istop < end_idx:
                    break
                line = cnvrt_2pts_to_coef(self.points[istrt].xy,
                                          self.points[istop].xy)
                avg_dist, cum_dsqr = self._find_sum_of_sq_dist_to_line(line,
                                                                      istrt,
                                                                      istop)
            istop += 1
        return istop

    def _find_local_min(self, indx0, indx1):
        """
        For points between indx0 & indx1, if there is a series of adjacent
        points sharing a local min dist value, return a tuple of the
        indexes of the first and last indexes of those adjacent points.
        If not, return an empty tuple.

        If a series of adjacent points happens to be at a local minimum
        distance to the lidar module, it can be a good place to look for
        and find a straight line. There are several reasons:
        1. The points are spaced most closely together.
        2. This section of wall is closest to the lidar module.
        3. This area is being scanned from a direction which is nearly
        parallel to the surface normal so the distance values will tend
        to be more reliable than for areas being scanned more obliquely.
        """
        mindist = ()
        for pnt in self.points:
            dist = pnt.dist
            if not mindist or dist < mindist:
                mindist = dist
        min_indx_list = [indx for indx, pnt in enumerate(self.points)
                         if pnt.dist <= mindist+1]
        return min_indx_list

    def _find_p2p_angles_of_pnts(self, indx0, indx1):
        """ """
        indexlist = [indx for indx in range(indx0, indx1)]
        slopelist = []
        for indx in indexlist:
            slope = proscan.p2p_angle(points[indx].xy, points[indx + 1].xy)
            slopelist.append(slope)
        return zip(indexlist, slopelist)

    def _find_sum_of_sq_dist_to_line(self, line, indx0, indx1):
        """
        Return sum of squares of distances between line (a, b, c)
        and a series of adjacent points from indx0 to indx1.
        """
        cum_dist = 0  # cumulative distance
        cum_dsqr = 0  # cumulative distance squared
        n = 0
        if indx1 < indx0:
            indx1, indx0 = indx0, indx1
        for idx in range(indx0, indx1):
            pnt = self.points[idx].xy
            dist = p2line_dist(pnt, line)
            dsqr = dist * dist
            cum_dist += dist
            cum_dsqr += dsqr
            n += 1
        avg_dist = cum_dist / n
        return (avg_dist, cum_dsqr)

    def _generate_points(self, data):
        """
        Thin raw data by culling adjacent records with same distance value.
        Each item of data contains 4 values:
        encoder_count, distance, byte_count, delta_time.
        Populate self.points list with Point objects,
        removing any whose distance value is 0 or 1200.

        enc_cnt (encoder count) values start at 0 
        and increase with CW rotation.
        straight left: enc_val = self.LEV; theta = pi
        straight ahead: enc_val = self.MEV; theta = pi/2
        straight right: enc_val = self.HEV; theta = 0
        (enc_cnt tops out at 32765, so no info past that)
        """
        grouped_data = group(data)
        thindata = cull_repeats(grouped_data)
        for item in thindata:
            enc_cnt = item[0]
            dist = item[1]
            if dist and dist <= 1200:
                pnt = Point(dist, enc_cnt)
                self.points.append(pnt)

        # calculate 'theta' value of each point (for polar coords)
        for pnt in self.points:
            #theta = math.pi * 1.5 * (1 - (pnt.enc_val / 30000))
            theta = (self.HEV - pnt.enc_val) * math.pi / (self.HEV - self.LEV)
            pnt.theta = theta

        # convert polar coords (dist, theta) to (x, y) coords
        for pnt in self.points:
            x = pnt.dist * math.cos(pnt.theta)
            y = pnt.dist * math.sin(pnt.theta)
            pnt.xy = (x, y)

    def _generate_regions(self):
        """Find continuous regions of closely spaced points (clumps)
        Large gaps (dist to neighbor > gap) represent 'edges' of regions.
        Record index of the start & end points of each region.
        Save as self.regions.
        """
        regions = []  # list of regions of closely spaced points
        start_index = 0
        dist = 0
        for n, pnt in enumerate(self.points):
            if pnt.enc_val < self.LEV:
                start_index = n
            elif self.LEV <= pnt.enc_val <= self.HEV:
                dist = p2p_dist(pnt.xy, self.points[n-1].xy)
            elif pnt.enc_val > self.HEV:
                break
            if dist > self.GAP * pnt.dist / 100:
                if n > (start_index + 1):
                    regions.append((start_index, n-1))
                start_index = n
        if n != start_index:
            regions.append((start_index, n))  # add last region
        self.regions = regions
        logger.debug(f"Regions: {self.regions}")

        # Cull tiny regions
        self._cull_regions()

        # In each region, find corners
        allcorners = set()
        for region in self.regions:
            corners = self._find_corners(region)
            for corner in corners:
                allcorners.add(corner)

        # split existing regions at corners
        new_regions = []
        for region in self.regions:
            new_indices = [region[0]]
            for corner_idx in allcorners:
                if corner_idx in range(region[0], region[-1]):
                    new_indices.append(corner_idx)
            new_indices.append(region[-1])
            for n in range(len(new_indices)-1):
                new_regions.append((new_indices[n], new_indices[n+1]))
        self.regions = new_regions

    def get_line_parameters(self):
        """Return a list of tuples, each tuple containing the parameters
        of the line which best fits each region in self.regions.
        acronym: 'clad' for (coords, length, angle, distance)."""
        linelist = []
        for region in self.regions:
            start_idx = region[0]
            end_idx = region[1]
            start_coords = self.points[start_idx].xy
            end_coords = self.points[end_idx].xy
            line = cnvrt_2pts_to_coef(start_coords, end_coords)
            coords = (start_coords, end_coords)
            length = p2p_dist(start_coords, end_coords)
            angle = p2p_angle(start_coords, end_coords)
            dist = p2line_dist((0, 0), line)  # perp distance to line
            linelist.append((coords, length, angle, dist))
        return linelist

    def _indexes_in_regions(self):
        """Return list of indexes contained in regions.
        """
        indexes = []
        for region in self.regions:
            indexes.extend(range(region[0], region[-1]+1))
        return indexes

    def map(self, map_folder="Maps", nmbr=None, show=True,
            display_all_points=False):

        filename = f"{map_folder}/scanMap"
        if nmbr:
            imagefile = filename + str(nmbr) + ".png"
        else:
            imagefile = filename + ".png"

        # build data lists to plot data points
        xs = []
        ys = []
        if display_all_points:
            pnts_to_plot = self.points
        else:
            pnts_to_plot = [pnt for idx, pnt in enumerate(self.points)
                            if idx in self._indexes_in_regions()]
        for pnt in pnts_to_plot:
            x, y = pnt.xy
            xs.append(x)
            ys.append(y)
        plt.scatter(xs, ys, color='#003F72')
        title = "(%s pts) " % len(self.points)
        title += "GAP = %s, FIT = %s" % (self.GAP, self.FIT)
        plt.title(title)

        # plot line segments
        line_coords = []  # x, y coordinates
        for region in self.regions:
            idx1, idx2 = region
            pnt1 = self.points[idx1].xy
            pnt2 = self.points[idx2].xy
            x_vals = [pnt1[0], pnt2[0]]
            y_vals = [pnt1[1], pnt2[1]]
            line_coords.append((pnt1, pnt2))
            plt.plot(x_vals, y_vals)
        #logger.debug(line_coords)

        plt.axis('equal')
        plt.savefig(imagefile)
        if show:
            plt.show()  # shows interactive plot
        plt.clf()  # clears previous points & lines
