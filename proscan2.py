import logging
import math
import operator
import sys
from constants import LEV, HEV, VLEG
import geom_utils as geo

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # set to DEBUG | INFO | WARNING | ERROR
logger.addHandler(logging.StreamHandler(sys.stdout))

# Default values used in ProcessScan
GAP = 10  # Threshold distance between adjacent points for continuity
FIT = 4  # Threshold distance (point to line) for good fit


class ProcessScan():
    """Process scan data.

    Find uninterrupted regions of closely spaced adjacent points
    likely to represent continuous surfaces.
    Find best fit straight lines within regions, likely to
    represent straight sections of walls.
    """

    def __init__(self, pointlist, lev=LEV, hev=HEV, gap=GAP, fit=FIT):
        """Accept scan data as list of point dictionaries.
        Optionally specify: sector of interest from
        lev (low encoder value) to hev (high encoder value),
        gap (threshold distance between adjacent points for continuity),
        fit (threshold point to line distance to qualify as 'good' fit).
        """
        self.target = None
        self.LEV = lev
        self.HEV = hev
        self.GAP = gap
        self.FIT = fit
        self.points = pointlist  # list of dictionaries in scan order
        self.regions = []  # list of 2-elem tuples (start_idx, end_idx)
        self.segments = []  # list of 2-elem tuples (start_idx, end_idx)
        self.zero_regions = []  # list of 2-elem tuples (start_idx, end_idx)
        self._generate_regions()
        self._generate_segments()
        self._find_zero_regions()

    def _find_corners(self, region):
        """
        Return list of indexes of points representing the end points
        of linked straight line segments spanning the region.
        The returned list includes the region end points as well as the
        corners where the straight line segnments join.

        How it is done:
        This method looks for corners by testing for fit, point by point
        with linked straight line segments within region. Test is done
        twice, first from beginning to end as index increases, then in
        reverse from end to beginning. The two results are compared.
        The result chosen is the one which finds the segment containing
        the greatest number of points, thus discouraaging the detection
        of 'false' corners in sections of wall that are really straight.
        The list of indexes returned is sorted lowest index first.

        Background info:
        If the points in a continuous region are substantially straight &
        linear, they can be well represented by a straight line segment
        between the start and end points of the region. However, if the
        points in a region trace an 'L' or zig-zag shape, as they would
        where walls meet at corners, we would find multiple straight
        lines segments linked at the corners.
        """
        idx0, idx1 = region  # indexes of region end points
        # Find corners startng with lowest index
        corners_fwd = [idx0]
        start_idx = idx0
        while start_idx != idx1:
            corner_idx = self._find_line_segment(start_idx, idx1)
            corners_fwd.append(corner_idx)
            start_idx = corner_idx

        # now find corners starting with highest index
        corners_rev = [idx1]
        corner_idx = idx1
        while corner_idx != idx0:
            corner_idx = self._find_line_segment(corner_idx, idx0)
            corners_rev.append(corner_idx)
        corners_rev.sort()

        logger.debug(f"region: {region}")
        logger.debug(f"corners forward: {corners_fwd}")
        logger.debug(f"corners reverse: {corners_rev}")

        # find the solution with the most points in a single segment
        largest_diff_fwd = self._largest_diff(corners_fwd)
        largest_diff_rev = self._largest_diff(corners_rev)
        if largest_diff_rev and largest_diff_rev > largest_diff_fwd:
            logger.debug(f"longest line reverse: {largest_diff_rev}")
            return corners_rev
        else:
            logger.debug(f"longest line forward: {largest_diff_fwd}")
            return corners_fwd

    def _largest_diff(self, intgrlst):
        """Given a list (length n) of integers, generate a list
        (length n-1) of the differences between adjacent integers. 
        Return the value of the largest difference.
        """
        diffs = [abs(intgrlst[n] - intgrlst[n-1])
                 for n in range(len(intgrlst))
                 if n]
        diffs.sort()  # largest last
        if diffs:
            return diffs.pop()
        else:
            return 0

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
        For points in a continuoous region, return a tuple (dist, index)
        of the last point found having the minimum dist value.
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

    def _generate_regions(self):
        """Find continuous regions of closely spaced points (clumps)
        Large gaps (dist to neighbor > gap) represent 'edges' of regions.
        Record index of the start & end points of each region.
        Save as self.regions."""

        regions = []  # list of regions of closely spaced points
        start_index = 0
        dist = 0
        for n, pnt in enumerate(self.points):
            if pnt.get("enc_cnt") < self.LEV:
                start_index = n
            elif self.LEV <= pnt.get("enc_cnt") <= self.HEV:
                dist = abs(pnt.get("dist") - self.points[n-1].get("dist"))
            elif pnt.get("enc_cnt") > self.HEV:
                break
            if dist > self.GAP * pnt.get("dist") / 100:
                if n > (start_index + 3):  # exclude 'tiny' regions
                    regions.append((start_index, n-1))
                start_index = n
        if n > start_index + 3:
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
            corners = self._find_corners(region)
            segments = zip(corners, corners[1:])
            all_segments.extend(segments)
        self.segments = all_segments

    def _find_zero_regions(self):
        """
        Find regions comprised of points whose distance values = -VLEG.

        Return list of indices of self.regions
        """
        zero_regions = []
        for n, region in enumerate(self.regions):
            if self.points[region[0]].get("dist") == -VLEG:
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
        Return (dist, index) tuple of closest point (to car).
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
        """Return index of non-zero region closest to car."""
        minlist = []
        for idx, region in enumerate(self.regions):
            if idx not in self.zero_regions:
                closest, _ = self.closest_point(region)
                minlist.append((closest, idx))
        minlist.sort()
        index_of_closest_region = minlist[0][-1]
        return index_of_closest_region

    def get_points_in_region(self, regn_indx):
        """Return list of xy coords of points in self.regions[regn_indx]
        """
        region = self.regions[regn_indx]
        point_indexes = range(region[0], region[-1]+1)
        return [self.points[indx].get('xy')
                for indx in point_indexes]

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
