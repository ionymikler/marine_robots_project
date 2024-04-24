#!/usr/bin/env python
# Made by: Jonathan Mikler
# Creation date: 2024-04-17

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
                                minRadius=1, maxRadius=int(0.4*rows)
                                )
    
    return np.array([]) if circles is None else circles

def find_targets(image: np.ndarray)->np.ndarray:
    return detect_circles(image)
