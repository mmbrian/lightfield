# -*- coding: utf-8 -*-
"""
Created on Sun Aug 13 20:39:59 2017

@author: Mohsen
"""
from __future__ import division

import numpy as np
from math import atan2, sqrt

def getRotationMatrixFromAngles(r):
    '''
    Returns a rotation matrix by combining elemental rotations 
    around x, y', and z''
    It also appends a zero row, so the end result looks like:
    [R_11 R_12 R_13]
    [R_11 R_12 R_13]
    [R_11 R_12 R_13]
    [0    0    0   ] 
    This is convenient since we can insert a translation column
    and get a full transformation matrix for vectors in homogeneous
    coordinates
    '''
    cos = map(np.cos, r)
    sin = map(np.sin, r)
    Rx = np.array([
        [1,      0,       0],
        [0, cos[0], -sin[0]],
        [0, sin[0], cos[0]]])
    Ry = np.array([
        [ cos[1], 0, sin[1]],
        [ 0     , 1,      0],
        [-sin[1], 0, cos[1]]])
    Rz = np.array([
        [cos[2], -sin[2], 0],
        [sin[2],  cos[2], 0],
        [0     ,       0, 1]])
    R = Rz.dot(Ry.dot(Rx))
    return np.concatenate((R, [[0, 0, 0]]))
    
    
class PinholeCamera:
    '''
    Models a Pinhole Camera with 9 degrees of freedom
    '''
    ################################################################################################
    ## Intrinsic parameters
    f = 50 # focal length in mm
    p = (0, 0) # position of principal point in the image plane
    
    # Extrinsic parameters
    # this rotation corresponds to a camera setting pointing towards (0, 0, -1)
    r = (0, 0, 0) # rotations in x, y', and z'' planes respectively 
    t = (0, 0, 0) # camera center translation w.r.t world coordinate system
    #
    # Using the above parameters we can construct the camera matrix
    #
    #                      [f 0 p.x 0]
    # P = K[R|t] where K = [0 f p.y 0] is the camera calibration matrix (full projection matrix)
    #                      [0 0  1  0]
    #
    # and thus we have x = PX for every point X in the word coordinate system 
    # and its corresponding projection in the camera image plane x
    # NOTE: points are assumed to be represented in homogeneous coordinates

    # Other parameters
    label = ''
    direction = (0, 0, 1) # camera direction vector (viewpoint direction)
    hfov = 37.556 # horizontal field of view
    vfov = 26.991 # vertical field of view
    sw = 35 # sensor width in mm
    sh = 24 # sensor height in mm
    # image resolution information
    xres = 4020
    yres = 2679 
    ################################################################################################
    
    def __init__(self, label, f = 50, r = (0, 0, 0), t = (0, 0, 0), direction = (0, 0, 1), sw = 35, sh = 24, xres = 4020, yres = 2679):
        self.label = label
        self.f = f
        self.r = r
        self.direction = direction
        self.t = t
        self.setXYRes(xres, yres)
        self.computeFOV(sw, sh) # computes fov from sensor size
        self.recomputeCameraMatrix(True, True)

    def recomputeCameraMatrix(self, changedRotation, changedIntrinsic):
        '''
        Updates camera matrix
        '''
        if changedRotation:
            # Computing rotation matrix using elemental rotations
            self.R = getRotationMatrixFromAngles(self.r)
            # by default if rotation is 0 then camera optical axis points to positive Z
            self.direction = np.array([0, 0, 1, 0]).dot(self.R) 

        # Computing the extrinsic matrix
        _t = -self.R.dot(np.array(self.t)) # t = -RC
        self.Rt = np.concatenate((self.R, np.array([[_t[0], _t[1], _t[2], 1]]).T), axis=1)
        # instead of the above, we could also represent translation matrix as [I|-C] and
        #                              [R -RC]
        # then compute Rt as R[I|-C] = [0   1]                [R]      [-RC]
        # but we basically do the same thing by concatenating [0] with [  1]

        if changedIntrinsic:
            # Computing intrinsic matrix
            f, px, py = self.f, self.p[0], self.p[1]
            self.K = np.array([
                [f, 0, px, 0],
                [0, f, py, 0],
                [0, 0, 1,  0]])

        # Full Camera Projection Matrix
        self.P = self.K.dot(self.Rt)
        
    ################################################################################################
    ## Intrinsic parameter setters
    def setF(self, f, auto_adjust = True):
        self.f = f
        if auto_adjust:
            self.computeFOV(self.sw, self.sh)
            self.recomputeCameraMatrix(False, True)
    def setP(self, p, auto_adjust = True):
        self.p = p
        if auto_adjust:
            self.recomputeCameraMatrix(False, True)
    def computeFOV(self, sw, sh):
        self.sw = sw
        self.sh = sh
        self.hfov = 2 * atan2(sw/2., self.f)
        self.vfov = 2 * atan2(sh/2., self.f)
        # 2 x f x tan(hfov/2) should give sw
        # 2 x f x tan(vfov/2) should give sh
    def setXYRes(self, xres, yres):
        self.xres = xres
        self.yres = yres 
    ################################################################################################
    ## Extrinsic parameter setters
    def setT(self, t, auto_adjust = True):
        self.t = t
        if auto_adjust:
            self.recomputeCameraMatrix(False, False)
    def setR(self, r, auto_adjust = True):
      self.r = r
      if auto_adjust:
          self.recomputeCameraMatrix(True, False)
    
    ################################################################################################
    ## Main methods
    def project(self, p):
        '''
        Computes projection of a point p in world coordinate system
        using its homogeneous coordinates [x, y, z, 1]
        '''
        if len(p) < 4: p = (p[0], p[1], p[2], 1)
        projection = self.P.dot(np.array(p))
        # dividing by the Z value to get a 2D point from homogeneous coordinates
        return np.array((projection[0], projection[1]))/projection[2]

    def getPixel(self, p):
        '''
        Given a projected point on the camera plane, converts its coordinates
        to pixels.
        (This already assumes image resolution and sensor size are provided)
        ----------------------
        |(0,0)    y>0        |
        |                    |
        |  x<0  self.p   x>0 |
        |                    |
        |  y<0    (xres,yres)|
        ----------------------
        Note: this method returns pixel coordinates even if they are outside
        the image plane. boundary checks must be applied by user.
        '''
        x, y = p[0], p[1]
        px, py = self.p[0], self.p[1]
        tl_x = x + px + self.sw/2. # x in mm from top left of camera image
        tl_y = self.sh/2. - (y + py) # y in mm from top left of camera image
        out_x = (tl_x / self.sw)*self.xres
        out_y = (tl_y / self.sh)*self.yres
        return np.array((out_x, out_y))


        