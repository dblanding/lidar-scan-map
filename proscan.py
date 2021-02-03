import logging
import math
import matplotlib.pyplot as plt
from matplotlib import style
import operator
import sys
import geom_utils as geo
import omnicar as oc

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

style.use('fivethirtyeight')

# Default values used in ProcessScan
GAP = 12  # Threshold distance between adjacent points for continuity
FIT = 3  # Threshold distance (point to line) for good fit

def encoder_count_to_radians(enc_val):
    """
    Convert encoder count to angle (radians) in car coordinate system

    X axis to the right, Y axis straight ahead
    theta = 0 along positive X axis, increaasing CCW
    """
    theta = (oc.HEV - enc_val) * math.pi / (oc.HEV - oc.LEV)
    return theta


class ProcessScan():
    """
    Generate points and find set of best fit lines from scan data.
    Plot points and/or lines. Display and/or save plot image.
    """

    def __init__(self, data, lev=None, hev=None, gap=None, fit=None):
        """Generate list of point dicts from (scan) data.
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
        self.zero_regions = []
        self._generate_points(data)
        self._generate_regions()
        self._generate_segments()
        self._find_zero_regions()

    def _find_corners(self, region):
        """
        If the points in a continuous region are substantially straight &
        linear, they can be well represented by a straight line segment
        between the start and end points of the region.
        However, if the points in a region trace an 'L' or 'U' shape, as
        they would where walls meet at corners, we would expect to find
        multiple straight lines, with those lines intersecting at corners.

        This method initiates the search at the beginning of the region
        and returns a list of indexes of the found corners.
        """
        idx0, idx1 = region  # indexes of region end points
        # Find corners with index ascending
        corners_fwd = []
        start_idx = idx0
        while start_idx != idx1:
            corner_idx = self._find_line_segment(start_idx, idx1)
            corners_fwd.append(corner_idx)
            start_idx = corner_idx
            

        # Alternatively, we could look in the reverse direction
        corners_rev = []
        corner_idx = idx1
        while corner_idx != idx0:
            corner_idx = self._find_line_segment(corner_idx, idx0)
            corners_rev.append(corner_idx)

        logger.debug(f"region: {region}")
        logger.debug(f"corners forward: {corners_fwd}")
        logger.debug(f"corners reverse: {corners_rev}")

        return corners_fwd

    def _find_line_segment(self, begin_idx, end_idx):
        """
        Starting with the point at begin_idx and moving toward end_idx,
        return the index of the last point in a series of adjacent points
        (up to end_idx) which 'fits' the straight line segment drawn from
        begin to end. The method is:
        Starting with two adjacent points, test the fit of the points with
        the line joining the points. Of course the fit wil be perfect with
        only two points. Now, gradually extend the line by adding the next
        point and again test the fit of all points with the line. Continue
        to do this until the line cannot be made any longer and still have
        a good fit.
        Return the index of the last point that fits the straight line.
        """
        if begin_idx < end_idx:  # indexes ascending
            ascending = True
        else:  # indexes descending
            ascending = False
        istrt = begin_idx
        istop = begin_idx
        avg_dist = 0
        while avg_dist < self.FIT:
            if ascending:
                istop += 1
                if istop > end_idx:
                    break
            else:
                istop -= 1
                if istop < end_idx:
                    break
            line = geo.cnvrt_2pts_to_coef(self.points[istrt].get("xy"),
                                          self.points[istop].get("xy"))
            avg_dist, cum_dsqr = self._find_sum_of_sq_dist_to_line(line,
                                                                   istrt,
                                                                   istop)
        if ascending:
            istop -= 1
        else:
            istop += 1
        return istop

    def _find_local_min(self, region):
        """
        For points between indx0 & indx1, return a tuple (dist, index)
        of the last point foune having the minimum dist value.
        """
        indx0, indx1 = region
        mindist = ()
        for pnt in self.points[indx0 : indx1]:
            dist = pnt.get("dist")
            if not mindist or dist < mindist:
                mindist = dist
        min_indx_list = [indx for indx, pnt in enumerate(self.points)
                         if pnt.get("dist") <= mindist+1]
        return min_indx_list

    def _find_p2p_angles_of_pnts(self, indx0, indx1):
        """Tabulate point to point angle of a serries of adjacent points."""
        indexlist = [indx for indx in range(indx0, indx1)]
        slopelist = []
        for indx in indexlist:
            slope = geo.p2p_angle(points[indx].get("xy"), points[indx + 1].get("xy"))
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
            pnt = self.points[idx].get("xy")
            dist = geo.p2line_dist(pnt, line)
            dsqr = dist * dist
            cum_dist += dist
            cum_dsqr += dsqr
            n += 1
        avg_dist = cum_dist / n
        return (avg_dist, cum_dsqr)

    def _generate_points(self, data):
        """
        populate self.points list with point dicts

        data: (encoder_count, distance, byte_count, delta_time)

        encoder_count values start at 0 and increase with CW rotation.
        straight left: enc_val = self.LEV; theta = pi
        straight ahead: enc_val = self.MEV; theta = pi/2
        straight right: enc_val = self.HEV; theta = 0
        (enc_cnt tops out at 32765, so no info past that)
        """
        points = []
        for record in data:
            encoder_count, dist, *rest = record
            point = {"encdr": encoder_count, "dist": dist}
            points.append(point)

        # calculate 'theta' value of each point
        for pnt in points:
            theta = encoder_count_to_radians(pnt.get("encdr"))
            pnt["theta"] = theta

        # calculate (x, y) coords from polar coords (dist, theta)
        for pnt in points:
            theta = pnt.get("theta")
            dist = pnt.get("dist")
            x = pnt.get("dist") * math.cos(pnt.get("theta"))
            y = pnt.get("dist") * math.sin(pnt.get("theta"))
            pnt["xy"] = (x, y)
        self.points = points

    def _generate_regions(self):
        """Find continuous regions of closely spaced points (clumps)
        Large gaps (dist to neighbor > gap) represent 'edges' of regions.
        Record index of the start & end points of each region.
        Save as self.regions."""

        regions = []  # list of regions of closely spaced points
        start_index = 0
        dist = 0
        for n, pnt in enumerate(self.points):
            if pnt.get("encdr") < self.LEV:
                start_index = n
            elif self.LEV <= pnt.get("encdr") <= self.HEV:
                dist = abs(pnt.get("dist") - self.points[n-1].get("dist"))
            elif pnt.get("encdr") > self.HEV:
                break
            if dist > self.GAP * pnt.get("dist") / 100:
                if n > (start_index + 1):
                    regions.append((start_index, n-1))
                start_index = n
        if n != start_index:
            regions.append((start_index, n))  # add last region
        self.regions = regions
        logger.debug(f"Regions: {self.regions}")

    def _generate_segments(self):
        """
        Within regions, detect linear sections which can be represented
        by straight line segments. Save as self.segments.
        """

        # build a list of index pairs representing segment end points
        all_segments = []
        for region in self.regions:
            start_idx, end_idx = region
            corners = self._find_corners(region)
            for index in region:
                if index not in corners:
                    corners.append(index)
            corners.sort()
            segments = zip(corners, corners[1:])
            all_segments.extend(segments)
        self.segments = all_segments

    def _find_zero_regions(self):
        """
        Find regions comprised of points whose distance values = 0

        Return list of indices of self.regions
        """
        zero_regions = []
        for n, region in enumerate(self.regions):
            if self.points[region[0]].get("dist") == -oc.VLEG:
                zero_regions.append(n)
        self.zero_regions = zero_regions

    def _indexes_in_regions(self):
        """Return list of indexes contained in regions.
        """
        indexes = []
        for region in self.regions:
            indexes.extend(range(region[0], region[-1]+1))
        return indexes

    def closest_point(self, region):
        """
        Search for non-zero points in region where
        region is a 2 element tuple (low_index, hi_index)
        Return (dist, index) tuple of closest point.
        dist is the lowest distance value found and
        index is the highest index having this value.
        """
        low_idx, hi_idx = region
        lowest_dist_val = 1200
        idx_of_lowest_val = None
        for idx in range(low_idx, hi_idx+1):
            dist = self.points[idx].get("dist")
            if 0 < dist < lowest_dist_val:
                lowest_dist_val = dist
                idx_of_lowest_val = idx
        return (lowest_dist_val, idx_of_lowest_val)

    def closest_region(self):
        """Return index of closest non-zero region."""
        minlist = []
        for idx, region in enumerate(self.regions):
            if idx not in self.zero_regions:
                closest, _ = self.closest_point(region)
                minlist.append((closest, idx))
        minlist.sort()
        index_of_closest_region = minlist[0][-1]
        return index_of_closest_region

    def regions_by_length(self):
        """
        Return list of region indexes sorted by length (longest first)
        """
        idx_len_pairs = [(n, region[-1]-region[0])
                         for n, region in enumerate(self.regions)]
        idx_len_pairs.sort(key=operator.itemgetter(1))
        idx_len_pairs.reverse()
        indexes = [pair[0] for pair in idx_len_pairs]
        return indexes

    def segments_in_region(self, indx):
        """
        Return segments in self.regions[indx] sorted longest first
        """
        idx_first, idx_last = self.regions[indx]
        indexes_in_region = [n for n in range(idx_first, idx_last+1)]
        seg_len_list = [(segment, segment[-1]-segment[0])
                        for segment in self.segments
                        if segment[0] in indexes_in_region]
        seg_len_list.sort(key=operator.itemgetter(1))
        seg_len_list.reverse()
        seglist = [pair[0] for pair in seg_len_list]
        return seglist

    def get_line_parameters(self, segment):
        """Return a tuple of the parameters of line segment.

        segment is a tuple of the inexes of the segment end points

        parameter acronym: 'clad' for (coords, length, angle, distance)
        """
        start_idx = segment[0]
        end_idx = segment[1]
        start_coords = self.points[start_idx].get("xy")
        end_coords = self.points[end_idx].get("xy")
        line = geo.cnvrt_2pts_to_coef(start_coords, end_coords)
        coords = (start_coords, end_coords)
        length = geo.p2p_dist(start_coords, end_coords)
        angle = geo.p2p_angle(start_coords, end_coords)
        dist = geo.p2line_dist((0, 0), line)  # perp distance to line
        return (coords, length, angle, dist)

    def map(self, map_folder="Maps", seq_nmbr=None, show=False,
            display_all_points=True):
        """Plot all points and line segments and save in map_folder.

        Optional args:
        display_all_points=False to plot only points in regions
        seq_nmbr to append to save_file_name
        show=True to display an interactive plot (which blocks program).
        """

        filename = f"{map_folder}/scanMap"
        if seq_nmbr:
            str_seq_nmbr = str(seq_nmbr)
            # prepend a '0' to single digit values (helps viewer sort) 
            if len(str_seq_nmbr) == 1:
                str_seq_nmbr = '0' + str_seq_nmbr
            imagefile = filename + str_seq_nmbr + ".png"
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
            x, y = pnt.get("xy")
            xs.append(x)
            ys.append(y)
        plt.scatter(xs, ys, color='#003F72')
        title = f"({len(self.points)} pts) GAP={self.GAP}, FIT={self.FIT}"
        plt.title(title)

        # plot line segments
        line_coords = []  # x, y coordinates
        for segment in self.segments:
            idx1, idx2 = segment
            pnt1 = self.points[idx1].get("xy")
            pnt2 = self.points[idx2].get("xy")
            x_vals = [pnt1[0], pnt2[0]]
            y_vals = [pnt1[1], pnt2[1]]
            line_coords.append((pnt1, pnt2))
            plt.plot(x_vals, y_vals)

        plt.axis('equal')
        plt.savefig(imagefile)
        if show:
            plt.show()  # shows interactive plot
        plt.clf()  # clears previous points & lines
