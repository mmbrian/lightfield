# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 14:41:41 2017

@author: Mohsen
"""

import numpy as np
import cv2

def loadHorizontalLF(data_path, fn_prefix, nviews, yres, xres, ext = 'jpg'):
    lf = np.zeros((1, nviews, yres, xres, 3), np.uint8)
    for i in xrange(0, nviews):
        fname = '%s/%s%0.4d.%s' % (data_path, fn_prefix, i, ext)
        print 'Loading image from %s...' % fname
        img = cv2.imread(fname, cv2.IMREAD_COLOR) # BGR and float64 format
        img = img.astype(np.uint8)
        # uncomment for visualizing with plt.imshow or to have RGB order
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        lf[0][i] = img
    return lf