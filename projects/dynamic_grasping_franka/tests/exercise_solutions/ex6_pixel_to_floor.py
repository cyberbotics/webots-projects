import numpy as np


def pixel_to_floor(u, v, width, height_px, fov, c, R, height):
    f = (width / 2) / np.tan(fov / 2)
    ray = R @ np.array([f, -(u + 0.5 - width / 2), -(v + 0.5 - height_px / 2)])
    t = (height - c[2]) / ray[2]
    return np.asarray(c) + t * ray
