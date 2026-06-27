"""Utility functions for the simulation module."""

import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the range [-π, π].
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-π, π]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def rotate_corners(corners_local: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
    return corners_local @ rot.T + np.array([x, y])
