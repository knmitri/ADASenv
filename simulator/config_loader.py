"""Configuration loader for simulator parking simulation."""

import json
from functools import lru_cache
import numpy as np
from pathlib import Path
from typing import Dict, Any


def load_config() -> Dict[str, Any]:
    """Load configuration from JSON file.
    
    Returns:
        Dictionary with all configuration parameters
    """
    config_path = Path(__file__).parent / "config.json"
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    # Convert degree values to radians
    config['simulation']['goal_heading_tolerance'] = np.deg2rad(
        config['simulation']['goal_heading_tolerance_deg']
    )
    return config


@lru_cache(maxsize=1)
def get_config() -> Dict[str, Any]:
    """Get global configuration instance (lazy-loaded).
    
    Returns:
        Dictionary with all configuration parameters
    """
    return load_config()
