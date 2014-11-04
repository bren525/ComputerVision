#!/usr/bin/env python

import cv2
import numpy as np

img = cv2.imread('intersection.jpeg',0)
ret,thresh = cv2.threshold(img,127,255,0)
contours,hierarchy = cv2.findContours(thresh, 1, 2)

cnt = contours[0]
M = cv2.moments(cnt)
cv2.drawContours(img,0,0,(0,0,255),2)
print M
