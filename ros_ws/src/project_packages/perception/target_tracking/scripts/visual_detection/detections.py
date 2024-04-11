#!/usr/bin/env python
# Made by: Jonathan Mikler
# Creation date: 2024-04-11

import cv2 as cv
import numpy as np

def detect_circles(image: np.ndarray)->np.ndarray:
    rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)

    gray = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)
    # Reduce the noise to avoid false circle detection
    gray = cv.medianBlur(gray, 5)

    rows = gray.shape[0]
    circles = cv.HoughCircles(image=gray, method=cv.HOUGH_GRADIENT,dp= 1,
                                minDist= rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=int(0.8*rows)
                                )
    
    return np.array([]) if circles is None else circles


def paint_circles_in_image(image: np.ndarray, circles: np.ndarray, n:int=1)->np.ndarray:
    """
    paints the fist n circles from 'circles' in the image
    """
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i, circle_i in enumerate(circles[0, :], start=1):
            if i > n:
                break
            center = (circle_i[0], circle_i[1])
            radius = circle_i[2]

            cv.circle(image, center, 1, (0, 100, 100), 3)
            cv.circle(image, center, radius, (255, 0, 255), 3)
            # write index next to circle center
            cv.putText(image, str(i), (circle_i[0], circle_i[1]), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 5)

        # first_circle = circles[0, :][0]
        # center = (first_circle[0], first_circle[1])
        # radius = first_circle[2]
        # cv.circle(image, center, radius, (0, 255, 0), 3)
        # cv.circle(image, center, 2, (0, 255, 0), 3)

    return image