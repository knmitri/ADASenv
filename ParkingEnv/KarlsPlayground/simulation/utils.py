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
