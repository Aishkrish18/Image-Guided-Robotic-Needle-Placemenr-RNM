import numpy as np


def look_at_the_point(pose0, a, b):
    z_e = normalize_vector(b - a)
    x_0 = pose0[:3, 0]
    x_e = normalize_vector(x_0 - (z_e @ x_0) * z_e)
    y_e = np.cross(z_e, x_e)
    pose1 = np.eye(4)
    pose1[:-1, :-1] = np.vstack([x_e, y_e, z_e]).T
    pose1[:-1, -1] = a.T
    return pose1


def normalize_vector(vector):
    return vector / np.linalg.norm(vector)


def spherical_to_cartesian(center, r, the, phi):
    x = r * np.sin(the) * np.cos(phi) + center[0]
    y = r * np.sin(the) * np.sin(phi) + center[1]
    z = r * np.cos(the) + center[2]
    return np.array([x, y, z])
