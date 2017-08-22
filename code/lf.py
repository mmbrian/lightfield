# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 14:42:57 2017

@author: Mohsen
"""

from geom import PinholeCamera



def createCameraSurfaceC(n_horizontal_views, n_vertical_views, f, xs, ys):
    '''
    f  is focal length on each camera on the grid
    xs and ys are camera separating distances in x-axis and y-axis respectively
    '''
    C = [[False for i in xrange(n_horizontal_views)] \
          for i in xrange(n_vertical_views)]
    for i in xrange(n_vertical_views): # each row
        for j in xrange(n_horizontal_views): # each column
            t = (j*xs, i*ys, 0) # translation from (0, 0, 0)
            # initializing with default parameters (sensor size and camera resolution)
            C[i][j] = PinholeCamera('view%d-%d' % (i, j), f = f, t = t)
    return C

def createFocalSurfaceF(c, f, sw, sh, xres, yres):
    '''
    c is the camera center in world coordinates
    f is focal length of the focal surface
    sw and sh are sensor width and height respectively
    xres and yres are horizontal and vertical resolution of image in pixels
    '''
    return PinholeCamera('F', f=f, t=c, sw=sw, sh=sh, xres=xres, yres=yres)
    