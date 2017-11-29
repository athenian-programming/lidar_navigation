import random

import sys
from scipy import stats

random.seed(0)

def random_pair(points):
    cnt = len(points)
    if cnt < 2:
        return None, None
    while True:
        index0 = int(random.uniform(0, cnt))
        index1 = int(random.uniform(0, cnt))
        if index0 != index1:
            return points[index0], points[index1]


def slopeYInt(p0, p1):
    xdiff = p1.x - p0.x
    # Avoid div by zero problems by adding a little noise
    if xdiff == 0:
        xdiff = sys.float_info.epsilon
    m = (p1.y - p0.y) / xdiff
    y = p0.y - (p0.x * m)
    return m, y


def straight_line(x, m, b):
    return (m * x) + b


class WallFinder(object):
    def __init__(self, iterations, threshold, min_points, points):
        self.__iterations = iterations
        self.__threshold = threshold
        self.__min_points = min_points
        self.__points = points

    def walls(self):
        # Return all walls with size of min_points
        wall_points = [p for p in self.__points]
        while True:
            if len(wall_points) < self.__min_points:
                return

            wall_inliers = []
            wall_outliers = []
            wall_p0 = None
            wall_p1 = None
            for i in range(self.__iterations):
                p0, p1 = random_pair(wall_points)

                iter_inliners = []
                iter_outliers = []
                for p in wall_points:
                    dist = p.distance_to_line(p0, p1)
                    if dist <= self.__threshold:
                        iter_inliners.append(p)
                    else:
                        iter_outliers.append(p)

                if len(iter_inliners) > len(wall_inliers):
                    wall_inliers = iter_inliners
                    wall_outliers = iter_outliers
                    wall_p0 = p0
                    wall_p1 = p1

            if len(wall_inliers) >= self.__min_points:
                # Use the outliers in the next iteration to find next wall in remaining points
                wall_points = wall_outliers
                yield Wall(wall_p0, wall_p1, wall_inliers)
            else:
                return


class Wall(object):
    def __init__(self, p0, p1, points):
        self.__p0 = p0
        self.__p1 = p1
        self.__points = points
        self.__m = None
        self.__b = None

    @property
    def p0(self):
        return self.__p0

    @property
    def p1(self):
        return self.__p1

    @property
    def points(self):
        return self.__points

    def slopeYInt(self):
        if self.__m is None:
            self.__m, self.__b = stats.linregress([p.x for p in self.points], [p.y for p in self.points])[0: 2]
        return self.__m, self.__b

    def yfit(self, x):
        m, b = self.slopeYInt()
        return (m * x) + b

    def xfit(self, y):
        m, b = self.slopeYInt()
        return (y - b) / m

