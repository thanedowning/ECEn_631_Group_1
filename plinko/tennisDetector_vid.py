#!/usr/bin/env python3

# This script attempts HSV tennis ball detection on a video file or
# camera feed.
# Instructions: Specify video file location or camera device.
# Run script. Type 'q' to quit.

from __future__ import print_function
import cv2
import numpy as np
import os

######## Specify video file or camera (i.e. '/dev/video1') here ########
# cap = cv2.VideoCapture('/home/seth/Videos/A4.MOV')
cap = cv2.VideoCapture(0)

alpha = 0.05
prev_score = 0
while cap.isOpened():
    # Read image
    ret, im = cap.read()
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV);
    # green = im[:,:,1]

    hue = hsv[:,:,0]
    sat = hsv[:,:,1]
    val = hsv[:,:,2]

    # mask = cv2.inRange(hue, 75, 100)
    # mask = cv2.inRange(sat, 200, 255)
    # mask = cv2.inRange(val, 100, 255)

    # cv2.imshow('filter', mask)
    # cv2.waitKey(10)
    # continue

    # define range of blue color in HSV
    lower_blue = np.array([75,200,100])
    upper_blue  = np.array([100,255,255])
    # lower = np.array([0,150,0])
    # upper = np.array([255,255,255])
    # lower = np.array([100])
    # upper = np.array([255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # mask = cv2.inRange(green, lower, upper)

    kernel = np.ones((8,8),np.uint8)
    erosion = cv2.erode(mask,kernel,iterations = 1)
    dilation = cv2.dilate(erosion,kernel,iterations = 2)
    #opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    mask = dilation

    ################ OpenCV's blob detection ####################
    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.2

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.8

    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    reversemask=255-mask
    keypoints = detector.detect(reversemask)

    if keypoints:
        # print("found %d blobs" % len(keypoints))
        if len(keypoints) > 1:
            keypoints.sort(key=(lambda s: s.size))
            keypoints=keypoints[len(keypoints)-1:len(keypoints)]
            score = 1.
    else:
        # print("no blobs")
        score = 0

    # filt_score = (1-alpha)*prev_score + alpha*score
    # prev_score = filt_score

    # Draw green circles around detected blobs
    im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(im,im, mask= mask)

    cv2.imshow('masked Image',res)
    cv2.imshow('mask',mask)
    cv2.imshow('Frame',im_with_keypoints)
    # if filt_score > 0.5:
        # print('Found tennis ball - prob =',filt_score)
    # else:
        # cv2.imshow('Frame',im)
        # print('No tennis ball - prob =',filt_score)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        cap.release()
        break
