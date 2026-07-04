
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
        if np.isclose(self.slot_orientation_rad, np.pi / 2):
            self.slot_orientation_type = 'perpendicular'
        elif np.isclose(self.slot_orientation_rad, 0.0):
            self.slot_orientation_type = 'parallel'
        else:
            self.slot_orientation_type = 'angled'

        self.slots: List[ParkingSlot] = []
        self.lot_width = 0.0
        self.lot_length = 0.0
        self.create_parking_layout()
        self._update_bounds_from_slots()

    def _slot_heading(self) -> float:
        if self.slot_orientation_type == 'perpendicular':
            return 0.0
        if self.slot_orientation_type == 'parallel':
            return np.pi / 2
        return float(self.slot_orientation_rad)

    def _single_side_pitch(self, heading: float) -> float:
        """Return spacing along the row direction for one slot."""

        return abs(self.slot_length * np.sin(heading)) + abs(self.slot_width * np.cos(heading))

    def _single_side_center_x(self, heading: float) -> float:
        """Place the slot so its aisle-facing edge starts at origin_x."""

        half_length_x = abs((self.slot_length / 2.0) * np.cos(heading))
        half_width_x = abs((self.slot_width / 2.0) * np.sin(heading))
        return self.origin_x + half_length_x + half_width_x

    def _update_bounds_from_slots(self) -> None:
        """Set rectangular world extents that contain all generated slots."""

        if not self.slots:
            self.lot_width = self.origin_x
            self.lot_length = self.origin_y
            return

        corners = np.vstack([slot.get_corners() for slot in self.slots])
        self.lot_width = float(max(self.origin_x, np.max(corners[:, 0])))
        self.lot_length = float(max(self.origin_y, np.max(corners[:, 1])))

    def create_parking_layout(self) -> None:
        """Create parking slots in a rxc grid arrangement with orientation type tracked."""
        slot_width = self.slot_width
        slot_length = self.slot_length
        orientation = self.slot_orientation_type
        heading = self._slot_heading()

        if self.layout == 'single_side':
            x = self._single_side_center_x(heading)
            pitch = self._single_side_pitch(heading)
            y_start = self.origin_y + pitch / 2.0
            for row in range(self.rows):
                y = y_start + row * (pitch + self.gap_between_slots)
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

        x_start = self.origin_x + self._single_side_center_x(heading) - self.origin_x
        y_start = self.origin_y + self._single_side_pitch(heading) / 2.0

        for row in range(self.rows):
            for col in range(self.cols):
                x = x_start + col * (slot_length + self.aisle_width)
                y = y_start + row * (self._single_side_pitch(heading) + self.aisle_width)
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
    
