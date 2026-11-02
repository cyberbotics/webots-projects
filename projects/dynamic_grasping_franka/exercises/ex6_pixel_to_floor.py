"""
Exercise 6 · From a pixel to a position on the floor
====================================================

A pinhole camera at position c (3-vector) with orientation R (3×3; its
columns are the camera's x = forward, y = left, z = up axes in the world)
sees the ball's centre at pixel (u, v) of a W×H image with horizontal field
of view fov.

    f       = (W / 2) / tan(fov / 2)                       focal length in pixels
    ray_cam = ( f,  −(u + 0.5 − W/2),  −(v + 0.5 − H/2) )   viewing ray, camera frame
    ray     = R · ray_cam                                   same ray, world frame
    t       = (height − c_z) / ray_z                        walk until z = height
    p       = c + t · ray

Check your answer:   python3 exercises/check.py 6
Reference solution:  controllers/dynamic_grasping/perception.py → pixel_to_floor
"""

import numpy as np


def pixel_to_floor(u, v, width, height_px, fov, c, R, height):
    """World point at the given height seen at pixel (u, v)."""
    # TODO
    raise NotImplementedError
