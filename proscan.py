
import math
import matplotlib.pyplot as plt
from matplotlib import style

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


class Point():
    """Convenience structure encapsulating data point measured values
    (int type) and calculated values (float type)"""

    def __init__(self, dist, enc_val, theta=0, xy=(0, 0)):
        self.dist = dist  # integer measured distance (cm)
        self.enc_val = enc_val  # angle encoder value (int)
        self.theta = theta  # (derived) float angle wrt car (radians)
        self.xy = xy  # (x, y) coordinates

# Default values used in ProcessScan
LEV = 10000  # Start of sector (Low Encoder Value)
HEV = 30000  # End of sector (High Encoder Value)
GAP = 7
CORNER = 7
HOOK = 2


class ProcessScan():
    """
    Generate points and find set of best fit lines from scan data.
    Plot points and/or lines. Display and/or save plot image.
    """

    def __init__(self, data, lev=None, hev=None, gap=None, crnr=None, hook=None):
        """Generate list of Point objects from (scan) data.
        Optionally specify: sector of interest
        from lev (low encoder value) to hev (high encoder value),
        gap (threshold of discontinuity of adjacent points),
        crnr (threshold to detect a corner in a region of continuous points),
        hook (to trigger rejection of end points hooked off main line).
        """
        if lev:
            self.LEV = lev
        else:
            self.LEV = LEV
        if hev:
            self.HEV = hev
        else:
            self.HEV = HEV
        if gap:
            self.GAP = gap
        else:
            self.GAP = GAP
        if crnr:
            self.CORNER = crnr
        else:
            self.CORNER = CORNER
        if hook:
            self.END_HOOK_LIMIT = hook
        else:
            self.END_HOOK_LIMIT = HOOK
        self.points = []
        self.regions = []
        self.segments = []
        self._generate_points(data)
        self._generate_regions()

    def _generate_points(self, data):
        """
        Convert raw data (list) to points.

        Each item of data contains 4 values:
        encoder_count, distance, byte_count, delta_time.
        Populate self.points list with Point objects,
        removing any whose distance value is 0 or 1200.

        enc_cnt (encoder count) values start at 0 (straight behind)
        and increase with CW rotation.
        straight left: enc_val = 10,000; theta = pi
        straight ahead: enc_val = 20,000; theta = pi/2
        straight right: enc_val = 30,000; theta = 0
        (enc_cnt tops out at 32765, so no info past that)
        """
        for item in data:
            enc_cnt = item[0]
            dist = item[1]
            if dist and dist != 1200:
                pnt = Point(dist, enc_cnt)
                self.points.append(pnt)

        # calculate 'theta' value of each point (for polar coords)
        for pnt in self.points:
            theta = 1.5 * math.pi * (1 - (pnt.enc_val / 30000))
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
            if dist > self.GAP:
                if n > (start_index + 1):
                    regions.append((start_index, n-1))
                start_index = n
        if n != start_index:
            regions.append((start_index, n))  # add last region
        self.regions = regions

        # Cull tiny regions
        self._cull_regions()

        # Find corners
        pass_nr = 0
        while self._find_corners():
            pass_nr += 1
            print(f"Looking for corners, pass {pass_nr}")
            print(f"Regions: {self.regions}")

        # Remove hooked ends from segments Todo: get this working better
        # self._remove_hooks()

    def _cull_regions(self):
        """Kind of like linting for regions..."""

        # Remove regions having fewer than 4 points
        to_remove = []
        for region in self.regions:
            if (region[-1] - region[0]) < 4:
                to_remove.append(region)
        print(f"Removed {len(to_remove)} tiny regions")
        for region in to_remove:
            self.regions.remove(region)

        # Find and remove regions with only 1 point
        for region in self.regions:
            if region[0] == region[-1]:
                self.regions.remove(region)
                print("Removed a region with only 1 point")

    def _find_corners(self):
        """
        If the points in a continuous region are substantially straight &
        linear, they can be well represented by a straight line segment
        between the start and end points of the region.
        However, if the points trace an 'L' shape, as they would where two
        walls meet at a corner, two straight line segments would be needed,
        with the two segments meeting at the corner.
        To find a corner in a region, look for the point at the greatest
        distance from the straight line joining the region end points.
        Only one corner is found at a time. To find multiple corners in a
        region (a zig-zag shaped wall, for example) make multiple passes.

        Within each continuous region, find index and value of data point
        located farthest from the end_to_end line segment if value > CORNERS.
        If index found, split region(s) at index.
        """
        corners = []
        found = False
        for region in self.regions:
            max_dist = 0
            max_idx = 0
            idx1 = region[0]
            idx2 = region[-1]
            p1 = (self.points[idx1].xy)
            p2 = (self.points[idx2].xy)
            line = cnvrt_2pts_to_coef(p1, p2)
            for n in range(idx1, idx2):
                pnt = (self.points[n].xy)
                dist = p2line_dist(pnt, line)
                if dist > max_dist:
                    max_dist = dist
                    max_idx = n
            if max_dist > self.CORNER:
                corners.append(max_idx)
        # Insert found corners into regions. Start with high values of index
        # first to avoid problems when popping and inserting into regions.
        if corners:
            found = True
            corners.reverse()
            for corner_index in corners:
                for n, region in enumerate(self.regions):
                    if region[0] < corner_index < region[-1]:
                        start, end = self.regions.pop(n)
                        self.regions.insert(n, (start, corner_index))
                        self.regions.insert(n+1, (corner_index+1, end))

    def _remove_hooks(self):
        """Remove the end point(s) of a region if they 'hook' off base line.
        """
        for n, region in enumerate(self.regions):
            start_idx = region[0]
            end_idx = region[-1]
            first_point = self.points[start_idx].xy
            last_point = self.points[end_idx].xy
            # find 'inner' base line points with first & last points removed
            pt1, pt2 = (self.points[start_idx + 1].xy, self.points[end_idx -1].xy)
            line = cnvrt_2pts_to_coef(pt1, pt2)
            if p2line_dist(first_point, line) > self.END_HOOK_LIMIT:
                start_idx += 2
                print("Removed a point at start of line")
            if p2line_dist(last_point, line) > self.END_HOOK_LIMIT:
                end_idx -= 2
                print("Removed a point at end of line")
            self.regions[n] = (start_idx, end_idx)

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

    def indexes_in_regions(self):
        """Return list of indexes contained in regions.
        """
        indexes = []
        for region in self.regions:
            indexes.extend(range(region[0], region[-1]+1))
        return indexes

    def map(self, map_folder="Maps", nmbr=None, show=True):

        filename = f"{map_folder}/scanMap"
        if nmbr:
            imagefile = filename + str(nmbr) + ".png"
        else:
            imagefile = filename + ".png"

        # build data lists to plot data points
        xs = []
        ys = []
        pnts_to_plot = [pnt for idx, pnt in enumerate(self.points)
                        if idx in self.indexes_in_regions()]
        for pnt in pnts_to_plot:
            x, y = pnt.xy
            xs.append(x)
            ys.append(y)
        plt.scatter(xs, ys, color='#003F72')
        title = "(%s pts) " % len(self.points)
        title += "GAP: %s, CORNER: %s" % (self.GAP, self.CORNER)
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
        # print(line_coords)

        plt.axis('equal')
        plt.savefig(imagefile)
        if show:
            plt.show()  # shows interactive plot
        plt.clf()  # clears previous points & lines
