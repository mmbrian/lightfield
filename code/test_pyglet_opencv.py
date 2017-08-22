# -*- coding: utf-8 -*-
"""
Created on Sun Aug 13 13:43:57 2017

@author: Mohsen

This show how to create two separate openCV and pyglet windows and modify the 
pyglet window based on events happening in openCV window and vice versa
"""

import cv2

import pyglet
from pyglet.gl import *

def nothing(x):
    global win
    win.dispatch_event('on_draw')
    pass

# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow('image')
# create trackbars for color change
cv2.createTrackbar('R','image',0,255,nothing)
cv2.createTrackbar('G','image',0,255,nothing)
cv2.createTrackbar('B','image',0,255,nothing)
# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)
 
win = pyglet.window.Window()
 
@win.event
def on_draw():
 
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT)
        
        r = cv2.getTrackbarPos('R','image')
        g = cv2.getTrackbarPos('G','image')
        b = cv2.getTrackbarPos('B','image')
 
        # Draw some stuff
        glBegin(GL_LINES)
        glVertex2i(50, 50)
        glVertex2i(75, r)
        glVertex2i(100, g)
        glVertex2i(200, b)
        glEnd()
        
        cv2.imshow('image',img)
#        k = cv2.waitKey(1) & 0xFF
#        if k == 27:
#            break
        # get current positions of four trackbars
        r = cv2.getTrackbarPos('R','image')
        g = cv2.getTrackbarPos('G','image')
        b = cv2.getTrackbarPos('B','image')
        s = cv2.getTrackbarPos(switch,'image')
        if s == 0:
            img[:] = 0
        else:
            img[:] = [b,g,r]
 

pyglet.app.run()
