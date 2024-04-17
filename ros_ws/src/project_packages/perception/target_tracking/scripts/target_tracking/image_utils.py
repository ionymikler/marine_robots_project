#!/usr/bin/env python
# Made by: Jonathan Mikler
# Creation date: 2024-04-17
import cv2 as cv
import numpy as np

def img_error(target_center: tuple, img_dims:tuple)->np.ndarray:
    """
    Calculates the error of the circle center in the image
    ---
    target_center: (u,v) tuple: center of the target circle in image space
    img_dims: (rows, cols) tuple: dimensions of the image
    """
    assert len(target_center) == 2, f"target_center must be a tuple with 2 elements. target_center: {target_center}"
    assert len(img_dims) == 2, "img_dims must be a tuple with 2 elements"

    center_x, center_y = target_center

    error_h = center_x - img_dims[0] / 2
    error_v = center_y - img_dims[1] / 2
    return error_h, error_v


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

    return image

def paint_x_in_image(image: np.ndarray, x:int, y:int, x_line_length:int=10)->np.ndarray:
    """
    Paints a cross in the image at the given coordinates
    ---
    image: np.ndarray: image to paint the cross in
    x: int: x coordinate of the cross
    y: int: y coordinate of the cross
    """
    red_color = (0, 0, 255)
    blue_color = (255, 0, 0)
    cv.line(image, (x - x_line_length, y), (x + x_line_length, y), red_color, 10)
    cv.line(image, (x, y - x_line_length), (x, y + x_line_length), red_color, 10)
    return image

def paint_centerlines_in_image(image: np.ndarray)->np.ndarray:
    """
    Paints a horizontal and vertical line in the center of the image
    ---
    image: np.ndarray: image to paint the lines in
    """
    rows, cols = image.shape[:2]
    cv.line(image, (0, rows // 2), (cols, rows // 2), (0, 255, 0), 3)
    cv.line(image, (cols // 2, 0), (cols // 2, rows), (0, 255, 0), 3)
    return image

def paint_img_errors(img: np.ndarray, target_center: tuple, error_h:int, error_v:int)->np.ndarray:
    assert isinstance(target_center[0], int) and isinstance(target_center[1], int), f"target_center must be a tuple of integers. target_center: {target_center}"

    cv.putText(img, f"error_h: {error_h}", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
    cv.putText(img, f"error_v: {error_v}", (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    x,y = target_center[:2]
    cv.line(img, (x, y), (img.shape[1] // 2, y), (255, 0, 0), 10)
    cv.line(img, (x, y), (x, img.shape[0] // 2), (255, 0, 0), 10)
    return img
