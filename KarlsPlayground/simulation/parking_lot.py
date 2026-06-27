
import numpy as np
from typing import List, Tuple

"""
Parking lot geometry and parking space definitions with realistic dimensions.
"""
class ParkingSlot:
    """Represents a single parking slot with realistic dimensions in meters."""
    
    def __init__(self, x: float, y: float, heading: float = 0.0, width: float = 2.5, length: float = 5.0, orientation: str = "perpendicular"):
        """Initialize a parking slot.
        Args:
            x, y: Center position of the slot (in meters)
            heading: Orientation of the slot (radians), default 0 (facing forward)
            width: Width of the slot in meters (default: 2.5m - EU standard)
            length: Length of the slot in meters (default: 5.0m - EU standard)
            orientation: 'perpendicular', 'parallel', or 'angled' (default: 'perpendicular')
        """
        self.x = x
        self.y = y
        self.heading = heading
        self.width = width
        self.length = length
        self.orientation = orientation  # Track slot orientation type
    
    def get_corners(self) -> np.ndarray:
        """Get the four corners of the parking slot in meters.
        
        Returns:
            Array of shape (4, 2) with corner coordinates in meters
        """
        half_l = self.length / 2
        half_w = self.width / 2
        corners_local = np.array([
            [-half_l, -half_w],
            [half_l, -half_w],
            [half_l, half_w],
            [-half_l, half_w],
        ])
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        rot = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        return corners_local @ rot.T + np.array([self.x, self.y])

    def get_goal_pose(self) -> Tuple[float, float, float]:
        """Get the goal pose for parking in this slot (center, heading=0).
        
        Returns:
            Tuple (x, y, heading) in meters and radians
        """
        return (self.x, self.y, self.heading)


class ParkingLot:
    """Manages a parking lot with multiple parking slots in meters."""
    
    def __init__(self, parking_lot_config: dict):
        """Initialize a parking lot with configurable slot orientation (perpendicular, parallel, angled). Args: parking_lot_config: Configuration dictionary containing parking lot parameters. Must include  'slot_orientation_deg' for angled slots."""
        self.rows = parking_lot_config['rows']
        self.cols = parking_lot_config['cols']
        self.num_slots =  self.rows * self.cols
        self.slot_width = parking_lot_config['slot_width']
        self.slot_length = parking_lot_config['slot_length']
        self.gap_between_slots = parking_lot_config['gap_between_slots']
        self.aisle_width = parking_lot_config['aisle_width']
        self.layout = parking_lot_config.get('layout', 'single_side')
        self.origin_x = parking_lot_config.get('origin_x', 0.0)
        self.origin_y = parking_lot_config.get('origin_y', 0.0)
        
        slot_orientation_deg = parking_lot_config.get('slot_orientation_deg', 0.0)
        self.slot_orientation_rad = np.deg2rad(slot_orientation_deg)
        self.slot_orientation_type = 'perpendicular' if self.slot_orientation_rad == np.pi/2  else ('parallel' if self.slot_orientation_rad == 0 else 'angled')

        # Calculate total width and length needed
        if self.layout == 'single_side':
            self.lot_width = self.origin_x + self.slot_length + self.aisle_width
            self.lot_length = self.origin_y + self.rows * self.slot_width + max(0, self.rows - 1) * self.gap_between_slots
        elif self.slot_orientation_type == 'perpendicular':
            self.lot_width = self.cols * self.slot_length + (self.cols - 1) * self.aisle_width + self.slot_length
            self.lot_length = self.rows * self.slot_width + (self.rows - 1) * self.gap_between_slots + self.slot_width
        elif self.slot_orientation_type == 'parallel':
            self.lot_width = self.cols * self.slot_width + (self.cols - 1) * self.gap_between_slots + self.slot_width
            self.lot_length = self.rows * self.slot_length + (self.rows - 1) * self.aisle_width + self.slot_length
        elif self.slot_orientation_type == 'angled':
            self.lot_width = self.cols * (self.slot_length * np.cos(self.slot_orientation_rad) + self.slot_width * np.sin(self.slot_orientation_rad)) + (self.cols - 1) * self.aisle_width + self.slot_length
            self.lot_length = self.rows * (self.slot_length * np.sin(self.slot_orientation_rad) + self.slot_width * np.cos(self.slot_orientation_rad)) + (self.rows - 1) * self.gap_between_slots + self.slot_width
        else:
            self.lot_width = self.cols * self.slot_length + (self.cols - 1) * self.aisle_width + self.slot_length
            self.lot_length = self.rows * self.slot_width + (self.rows - 1) * self.gap_between_slots + self.slot_width

        self.slots: List[ParkingSlot] = []

        self.create_parking_layout()

    def create_parking_layout(self) -> None:
        """Create parking slots in a rxc grid arrangement with orientation type tracked."""
        slot_width = self.slot_width
        slot_length = self.slot_length
        orientation = self.slot_orientation_type
        heading = 0.0
        if orientation == 'perpendicular':
            heading = 0.0
        elif orientation == 'parallel':
            heading = np.pi / 2
        elif orientation == 'angled':
            heading = self.slot_orientation_rad

        if self.layout == 'single_side':
            x = self.origin_x + slot_length / 2
            y_start = self.origin_y + slot_width / 2
            for row in range(self.rows):
                y = y_start + row * (slot_width + self.gap_between_slots)
                slot = ParkingSlot(
                    x,
                    y,
                    heading=heading,
                    width=slot_width,
                    length=slot_length,
                    orientation=orientation,
                )
                self.slots.append(slot)
            return

        x_start = self.origin_x + slot_length / 2 + 0.5
        y_start = self.origin_y + slot_width / 2 + 0.5

        for row in range(self.rows):
            for col in range(self.cols):
                x = x_start + col * (slot_length + self.aisle_width)
                y = y_start + row * (slot_width + self.aisle_width)
                slot = ParkingSlot(x, y, heading=heading, width=slot_width, length=slot_length, orientation=orientation)
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
    
