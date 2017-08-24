# -*- coding: utf-8 -*-
"""
Created on Tue Aug 22 15:41:12 2017

@author: Mohsen
"""

import numpy as np
import cv2

import pyglet
from pyglet.gl import *

from rio import loadHorizontalLF
from lf import *
from geom import rayPlaneIntersection

# IO Parameters
data_path = 'G:\HiWi\LF_compression\data\couch_image'
img_prefix = 'couch_image_' # e.g. couch_image_0019.jpg
img_ext = 'jpg'
# LF parameters
n_horizontal_views = 10
n_vertical_views = 1
global C
# LF data parameters (disney couch scene)
x_separating = 2 # mm
y_separating = 2 # mm
focal_length = 50 # mm
sensor_width = 35 # mm
sensor_height = 24 # mm
img_res = (4020, 2679) # x and y in pixels
global lf
# Viewpoint parameters
K_res = (300, 200) # output image resolution
K_f = 50 # mm (focal length of the desired camera K)
F = 150 # mm (initial distance of the focusal surface)
f0 = np.array((0, 0, F))
# We assume focal surface is parallel to xy plane (i.e. also parallel to C)
delta = 2 # box size around an s,t location to look for data cameras
global K

# GUI parameters
cv_window_name = 'render_view'
global win
# The following parameters control the bounding box in which the camera K
# can move. (i.e. camera origin)
# Here we assume top left camera in C is positioned at world origin (0,0,0)
min_viewpoint_x = 0
max_viewpoint_x = n_horizontal_views * x_separating
min_viewpoint_y = 0
max_viewpoint_y = n_vertical_views * y_separating
global img
img = np.zeros((K_res[1], K_res[0], 3), np.uint8)


def renderView():
    global C, K
    img = np.zeros((K_res[1], K_res[0], 3), np.uint8)
    for px in xrange(K_res[0]):
        for py in xrange(K_res[1]):
            # raytracing pixel located at px, py
            r = K.getRayFromPixel((px, py))
            # r.o # ray origin
            # r.d # ray direction (normalized)
            # Intersect r with C to get (s', t')
            rc = intersectWithC(r)
            sptp = getST(rc, x_separating, y_separating)
            # Intersect r with F to get (f, g)_F
            fgz = rayPlaneIntersection(r, f0, np.array((0, 0, 1)))
            for st in getCamerasWithinSTrange(sptp[0], sptp[1], delta, n_horizontal_views, n_vertical_views):
                s, t = st[0], st[1]


def main():
#    global lf
#    print 'Loading LF views...'
#    lf = loadHorizontalLF(data_path, img_prefix, \
#                          n_horizontal_views, \
#                          img_res[1], img_res[0], \
#                          ext = img_ext)

    global C, K
    C = createCameraSurfaceC(n_horizontal_views, n_vertical_views, focal_length, x_separating, y_separating)
    K = createCameraK((0, 0, 0), K_f, sensor_width, sensor_height, K_res[0], K_res[1])
    
    
    global win
    cv2.namedWindow(cv_window_name)
    win = pyglet.window.Window()
    
    cv2.createTrackbar('X', cv_window_name, min_viewpoint_x, max_viewpoint_x, onXchange)
    cv2.createTrackbar('Y', cv_window_name, min_viewpoint_y, max_viewpoint_y, onYchange)
    
    cv2.imshow(cv_window_name, img)
    
    @win.event
    def on_draw():
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT)
        
        img = np.zeros((K_res[1], K_res[0], 3), np.uint8)
        
        x = cv2.getTrackbarPos('X', cv_window_name)
        y = cv2.getTrackbarPos('Y', cv_window_name)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img,'%d - %d' % (x, y), (10, 100), font, 1, (255,255,255), 2)
        
        cv2.imshow(cv_window_name, img)
        
        # Draw some stuff
        glBegin(GL_LINES)
        glVertex2i(50, 50)
        glVertex2i(75, x)
        glVertex2i(100, y)
        glVertex2i(200, x*y)
        glEnd()
        
    @win.event
    def on_close():
        cv2.destroyAllWindows()
    
    pyglet.app.run()
    
#    print 'Disposing LF views...'
#    del lf
#    print 'Finished.'
 
    
def onXchange(x):
    # TODO
    redrawGL()
    
def onYchange(y):
    # TODO
    redrawGL()
    
def redrawGL():
    # Force redraw (calls on_draw method for pyglet window)
    global win
    win.dispatch_event('on_draw')    

if __name__ == '__main__':
    main()