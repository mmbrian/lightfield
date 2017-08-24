# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 14:42:57 2017

@author: Mohsen
"""
from __future__ import division

from geom import PinholeCamera, rayPlaneIntersection
import numpy as np
from math import ceil, floor

tl_cam_offset = np.array((0, 0, 0)) # top left offset position of C
# Camera surface C is parallel to xy plane and this offset locates the
# location of top left camera in the grid. therefore it is also a point
# on this plane

def intersectWithC(ray):
    '''
    Intersects a ray with camera plane C and returns intersection point
    NOTE: it is assumed that C is parallel to xy plane
    '''
    p0 = tl_cam_offset
    n = np.array((0, 0, 1))
    return rayPlaneIntersection(ray, p0, n)

def getST(p, xs, ys):
    '''
    Given a point on C, returns s,t position of the corresponding camera
    (can be float since cameras are located at discrete positions)
    '''
    x, y = p[0], p[1]
    dx, dy = x - tl_cam_offset[0], y - tl_cam_offset[1]
    s = dx / xs
    t = dy / ys
    return (s, t)

def getCamerasWithinSTrange(s, t, delta, n_horizontal_views, n_vertical_views):
    min_s, max_s = int(ceil(s-delta/2)), int(floor(s+delta/2))
    min_t, max_t = int(ceil(t-delta/2)), int(floor(t+delta/2))
    out_st = []
    for s in xrange(min_s, max_s+1):
        for t in xrange(min_t, max_t+1):
            # Check if s,t is a valid data camera in the camera array
            if s>=0 and t>=0 and s<n_horizontal_views and t<n_vertical_views:
               out_st.append((s, t))
    return out_st

def createCameraSurfaceC(n_horizontal_views, n_vertical_views, f, xs, ys):
    '''
    f  is focal length on each camera on the grid
    xs and ys are camera separating distances in x-axis and y-axis respectively
    This method assumes the top left camera in the camera array is located
    at tl_cam_offset in world coordinate system
    '''
    C = [[False for i in xrange(n_horizontal_views)] \
          for i in xrange(n_vertical_views)]
    for t in xrange(n_vertical_views): # each row
        for s in xrange(n_horizontal_views): # each column
            _t = tl_cam_offset + np.array((s*xs, t*ys, 0)) # translation from (0, 0, 0)
            # initializing with default parameters (sensor size and camera resolution)
            C[t][s] = PinholeCamera('view%d-%d' % (t, s), f = f, t = _t)
    return C

def createCameraK(c, f, sw, sh, xres, yres):
    '''
    c is the camera center in world coordinates
    f is focal length of the desired view (camera K)
    sw and sh are sensor width and height respectively
    xres and yres are horizontal and vertical resolution of image in pixels
    '''
    return PinholeCamera('F', f=f, t=c, sw=sw, sh=sh, xres=xres, yres=yres)
    