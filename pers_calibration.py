#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('im2.png')
img = cv2.resize(img,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
print img.shape
rows,cols,ch = img.shape
pts1 = np.float32([[90,122],[313,122],[35,242],[385,242]])
pts2 = np.float32([[0,0],[400,0],[0,400],[400,400]])

M = cv2.getPerspectiveTransform(pts1,pts2)
img_size = (img.shape[1], img.shape[0])
dst = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size

plt.subplot(121),plt.imshow(img),plt.title('Input')
plt.subplot(122),plt.imshow(dst),plt.title('Output')
plt.show()
