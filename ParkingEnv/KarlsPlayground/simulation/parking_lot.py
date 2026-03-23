"""
Parking lot geometry and parking space definitions with realistic dimensions.

Standard EU parking space: 2.5m (width) x 5.0m (length)
"""

import numpy as np
from typing import List, Tuple


class ParkingSlot:
    """Represents a single parking slot with realistic dimensions in meters."""
    
    # Standard EU parking space dimensions (in meters)
    STANDARD_WIDTH = 2.5   # Width of parking space
    STANDARD_LENGTH = 5.0  # Length of parking space
    
    def __init__(self, x: float, y: float, width: float = 2.5, length: float = 5.0):
        """Initialize a parking slot.
        
        Args:
            x, y: Center position of the slot (in meters)
            width: Width of the slot in meters (default: 2.5m - EU standard)
            length: Length of the slot in meters (default: 5.0m - EU standard)
        """
        self.x = x
        self.y = y
        self.width = width
        self.length = length
    
    def get_corners(self) -> np.ndarray:
        """Get the four corners of the parking slot in meters.
        
        Returns:
            Array of shape (4, 2) with corner coordinates in meters
        """
        half_w = self.width / 2
        half_l = self.length / 2
        return np.array([
            [self.x - half_w, self.y - half_l],
            [self.x + half_w, self.y - half_l],
            [self.x + half_w, self.y + half_l],
            [self.x - half_w, self.y + half_l],
        ])
    
    def get_goal_pose(self) -> Tuple[float, float, float]:
        """Get the goal pose for parking in this slot (center, heading=0).
        
        Returns:
            Tuple (x, y, heading) in meters and radians
        """
        return (self.x, self.y, 0.0)


class ParkingLot:
    """Manages a parking lot with multiple parking slots in meters."""
    
    def __init__(self, num_slots: int = 12, width: float = 15.0, depth: float = 12.0):
        """Initialize a parking lot with parking slots arranged in a grid.
        
        Args:
            num_slots: Number of parking slots (arranged in 3 rows x 4 columns = 12)
            width: Total width of parking lot in meters (default 15.0m)
            depth: Total depth of parking lot in meters (default 12.0m)
        """
        self.num_slots = num_slots
        self.width = width
        self.depth = depth
        self.slots: List[ParkingSlot] = []
        
        self._create_slots()
    
    def _create_slots(self) -> None:
        """Create parking slots in a 3x4 grid arrangement with realistic dimensions."""
        # 3 rows x 4 columns arrangement = 12 parking spaces
        rows = 3
        cols = 4
        
        # Standard EU parking space dimensions
        slot_width = ParkingSlot.STANDARD_WIDTH    # 2.5m
        slot_length = ParkingSlot.STANDARD_LENGTH  # 5.0m
        
        # Spacing between slots (gap for aisles)
        aisle_width = 1.0  # 1.0m gap between parking rows
        
        # Calculate total width and depth needed
        total_width_needed = cols * slot_width + (cols - 1) * 0.5  # minimum gaps
        total_depth_needed = rows * slot_length + (rows - 1) * aisle_width
        
        # Starting positions (left and top margins)
        x_start = slot_width / 2 + 0.5  # 0.5m margin from left
        y_start = slot_length / 2 + 0.5  # 0.5m margin from top
        
        # Create slots in grid
        for row in range(rows):
            for col in range(cols):
                # Alternate orientation: even rows point in Y direction, odd rows in X
                x = x_start + col * (slot_width + 0.5)
                y = y_start + row * (slot_length + aisle_width)
                
                slot = ParkingSlot(x, y, width=slot_width, length=slot_length)
                self.slots.append(slot)
    
    def get_slot(self, index: int) -> ParkingSlot:
        """Get a parking slot by index.
        
        Args:
            index: Index of the parking slot (0 to num_slots-1)
            
        Returns:
            ParkingSlot object
        """
        return self.slots[index % self.num_slots]
    
    def get_random_slot(self) -> Tuple[int, ParkingSlot]:
        """Get a random parking slot.
        
        Returns:
            Tuple of (slot_index, ParkingSlot)
        """
        index = np.random.randint(0, self.num_slots)
        return index, self.slots[index]
    
    def get_all_slots(self) -> List[ParkingSlot]:
        """Get all parking slots.
        
        Returns:
            List of all ParkingSlot objects
        """
        return self.slots

    def check_collision(self, vehicle_corners: np.ndarray) -> bool:
        """Check if vehicle is out of lot bounds.
        
        Args:
            vehicle_corners: Array [4,2] of vehicle corner points

        Returns:
            True if out of bounds, False otherwise
        """
        vehicle_min_x = vehicle_corners[:, 0].min()
        vehicle_max_x = vehicle_corners[:, 0].max()
        vehicle_min_y = vehicle_corners[:, 1].min()
        vehicle_max_y = vehicle_corners[:, 1].max()

        boundary_margin = 0.3
        if (vehicle_min_x < boundary_margin or vehicle_max_x > self.width - boundary_margin or
            vehicle_min_y < boundary_margin or vehicle_max_y > self.depth - boundary_margin):
            return True

        return False
