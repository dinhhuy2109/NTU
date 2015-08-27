import numpy as np
import scipy as sp

from scipy.spatial import ConvexHull

def rotatePolygon(polygon,theta):
    """Rotates the given polygon which consists of corners represented as (x,y),
    around the ORIGIN, clock-wise, theta degrees"""
    theta = math.radians(theta)
    rotatedPolygon = []
    for corner in polygon :
        rotatedPolygon.append(( corner[0]*math.cos(theta)-corner[1]*math.sin(theta) , corner[0]*math.sin(theta)+corner[1]*math.cos(theta)) )
    return rotatedPolygon

def rotatePoint(centerPoint,point,angle):
    """Rotates a point around another centerPoint. Angle is in degrees.
    Rotation is counter-clockwise"""
    angle = math.radians(angle)
    temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
    temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
    temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
    return temp_point


class Vector2D(object):
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = x, y

    def rotate(self, angle):
        angle = math.radians(angle)
        sin = math.sin(angle)
        cos = math.cos(angle)
        x = self.x
        y = self.y
        self.x = x * cos - y * sin
        self.y = x * sin + y * cos

    def __repr__(self):
        return '<Vector2D x={0}, y={1}>'.format(self.x, self.y)

class Polygon(object):
    def __init__(self, points):
        self.points = [Vector2D(*point) for point in points]

    def rotate(self, angle):
        for point in self.points:
            point.rotate(angle)

    def center(self):
        totalX = totalY = 0.0
        for i in self.points:
            totalX += i.x
            totalY += i.y

        len_points = len(self.points)

        return Vector2D(totalX / len_points, totalY / len_points)

    def rotate(self, angle):
        center = self.center()
        for point in self.points:
            point.x -= center.x
            point.y -= center.y
            point.rotate(angle)
            point.x += center.x
            point.y += center.y

def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
    from scipy.spatial import Delaunay
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0
