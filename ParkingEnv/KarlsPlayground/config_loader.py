"""Configuration loader for KarlsPlayground parking simulation."""

import json
import numpy as np
from pathlib import Path
from typing import Dict, Any


def load_config() -> Dict[str, Any]:
    """Load configuration from JSON file.
    
    Returns:
        Dictionary with all configuration parameters
    """
    config_path = Path(__file__).parent / "config.json"
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # Convert degree values to radians
    config['vehicle']['audi_a5']['max_steering_angle'] = np.deg2rad(
        config['vehicle']['audi_a5']['max_steering_angle_deg']
    )
    config['simulation']['goal_heading_tolerance'] = np.deg2rad(
        config['simulation']['goal_heading_tolerance_deg']
    )
    config['planner']['yaw_resolution'] = np.deg2rad(
        config['planner']['yaw_resolution_deg']
    )
    
    return config


# Global config instance (lazy-loaded)
_config = None


def get_config() -> Dict[str, Any]:
    """Get global configuration instance (lazy-loaded).
    
    Returns:
        Dictionary with all configuration parameters
    """
    global _config
    if _config is None:
        _config = load_config()
    return _config
