"""
Geometry utilities for 3D transformations
"""
import math
import numpy as np


def make_T(R, t3x1):
    """Make 4x4 transformation matrix from rotation and translation
    
    Args:
        R: 3x3 rotation matrix
        t3x1: 3x1 translation vector
        
    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3:] = t3x1
    return T


def inv_T(T):
    """Invert 4x4 transformation matrix
    
    Args:
        T: 4x4 transformation matrix
        
    Returns:
        Inverted 4x4 transformation matrix
    """
    R = T[:3, :3]
    t = T[:3, 3:]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3:] = -R.T @ t
    return Ti


def Rx(deg):
    """Rotation matrix around x-axis
    
    Args:
        deg: Rotation angle in degrees
        
    Returns:
        3x3 rotation matrix
    """
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0,   c,  -s],
        [0.0,   s,   c]
    ], dtype=np.float64)


def rotation_matrix_from_axes(x_axis, y_axis, z_axis):
    """Create rotation matrix from three orthonormal axes (column vectors)
    
    Args:
        x_axis, y_axis, z_axis: 3D vectors representing the axes
        
    Returns:
        3x3 rotation matrix
    """
    R = np.column_stack([x_axis, y_axis, z_axis])
    return R
