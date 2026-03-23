"""Realistic parking scenarios with two-lane layout and parked cars."""

import numpy as np
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass


@dataclass
class ParkedCar:
    """Represents an already parked car in the parking lot."""
    
    x: float  # Center X position
    y: float  # Center Y position
    yaw: float  # Heading angle in radians
    slot_index: int = -1  # Slot index for this parked car
    length: float = 4.67  # Vehicle length in meters
    width: float = 1.86  # Vehicle width in meters
    
    def get_corners(self) -> np.ndarray:
        """Get the four corners of the parked car in global frame.
        
        Returns:
            Array of shape (4, 2) with corner positions [x, y]
        """
        cos_y = np.cos(self.yaw)
        sin_y = np.sin(self.yaw)
        
        # Corners relative to center (local frame)
        corners_local = np.array([
            [-self.length/2, -self.width/2],
            [-self.length/2, self.width/2],
            [self.length/2, self.width/2],
            [self.length/2, -self.width/2],
        ])
        
        # Transform to global frame
        corners_global = np.zeros_like(corners_local)
        for i, corner in enumerate(corners_local):
            corners_global[i, 0] = self.x + corner[0] * cos_y - corner[1] * sin_y
            corners_global[i, 1] = self.y + corner[0] * sin_y + corner[1] * cos_y
        
        return corners_global


class ParkingScenario:
    """Realistic parking scenario with two lanes and parked cars."""
    
    def __init__(self):
        """Initialize a realistic two-lane parking scenario."""
        # Parking lot dimensions
        self.lot_width = 20.0   # Total width (in meters)
        self.lot_depth = 30.0   # Total depth (in meters)
        
        # Lane configuration
        self.num_lanes = 2      # Two lanes
        self.lane_width = 3.5   # Width of each lane (3.5m allows for maneuvering)
        
        # Parking spaces
        self.space_width = 2.5  # EU standard width
        self.space_length = 5.0  # EU standard length
        self.spaces_per_side = 6  # 6 spaces on each side of each lane
        
        self.parking_slots: List[Dict[str, Any]] = []
        self.parked_cars: List[ParkedCar] = []
        
        self._create_parking_layout()
        self._populate_parked_cars()
    
    def _create_parking_layout(self) -> None:
        """Create a two-lane parking lot with perpendicular spaces on both sides."""
        lane_spacing = self.lot_width / self.num_lanes

        # Parking slots are perpendicular to vertical lanes:
        # slot width in x direction = space_length, slot length in y direction = space_width.
        slot_width = self.space_length
        slot_length = self.space_width

        # For each lane, generate slots on outer sides only (to keep the center corridor open)
        for lane in range(self.num_lanes):
            lane_center_x = lane * lane_spacing + lane_spacing / 2
            lane_left_edge = lane_center_x - self.lane_width / 2
            lane_right_edge = lane_center_x + self.lane_width / 2

            # Decide outer side by lane parity
            row_spacing = self.space_width + 1.0
            base_y = 1.25

            if lane % 2 == 0:
                # even lane: populate left side only
                slot_x = lane_left_edge - slot_width / 2 - 0.5
                side = 'left'
            else:
                # odd lane: populate right side only
                slot_x = lane_right_edge + slot_width / 2 + 0.5
                side = 'right'

            for space in range(self.spaces_per_side):
                y = base_y + space * row_spacing
                self.parking_slots.append({
                    'index': len(self.parking_slots),
                    'x': slot_x,
                    'y': y,
                    'width': slot_width,
                    'length': slot_length,
                    'side': side,
                    'lane': lane,
                })
    
    def _populate_parked_cars(self) -> None:
        """Add some already parked cars to make scenario more realistic.
        
        We'll park some cars in random slots (about 50% occupancy).
        """
        # Seed for reproducibility
        rng = np.random.RandomState(42)
        
        # Pick random slots to have parked cars (50% occupancy)
        num_occupied = len(self.parking_slots) // 2
        occupied_indices = rng.choice(len(self.parking_slots), num_occupied, replace=False)
        
        for idx in occupied_indices:
            slot = self.parking_slots[idx]
            
            # Parked cars are aligned with the parking space (heading = 0)
            parked_car = ParkedCar(
                x=slot['x'],
                y=slot['y'],
                yaw=0.0,  # Parallel parked, heading along lane
                slot_index=idx,
            )
            self.parked_cars.append(parked_car)
    
    def get_slot(self, index: int) -> Dict[str, Any]:
        """Get a parking slot by index."""
        return self.parking_slots[index % len(self.parking_slots)]
    
    def get_random_empty_slot(self) -> Tuple[int, Dict[str, Any]]:
        """Get a random empty parking slot.
        
        Returns:
            Tuple of (slot_index, slot_dict)
        """
        occupied_indices = {car.slot_index for car in self.parked_cars if car.slot_index >= 0}
        empty_slots = [s for s in self.parking_slots if s['index'] not in occupied_indices]
        
        if not empty_slots:
            # If all occupied, return a random slot anyway
            idx = np.random.randint(0, len(self.parking_slots))
            return idx, self.parking_slots[idx]
        
        slot = empty_slots[np.random.randint(0, len(empty_slots))]
        return slot['index'], slot
    
    def get_all_slots(self) -> List[Dict[str, Any]]:
        """Get all parking slots."""
        return self.parking_slots
    
    def get_parked_cars(self) -> List[ParkedCar]:
        """Get all already parked cars."""
        return self.parked_cars
    
    def check_collision(self, vehicle_corners: np.ndarray) -> bool:
        """Check if vehicle collides with any parked cars or boundaries.
        
        Args:
            vehicle_corners: Vehicle corners from vehicle.get_corners()
            
        Returns:
            True if collision detected, False otherwise
        """
        # Check collision with parked cars using separating axis theorem (simple AABB check)
        vehicle_min_x = vehicle_corners[:, 0].min()
        vehicle_max_x = vehicle_corners[:, 0].max()
        vehicle_min_y = vehicle_corners[:, 1].min()
        vehicle_max_y = vehicle_corners[:, 1].max()
        
        # Add collision margin (safety distance)
        collision_margin = 0.2  # 20cm buffer
        
        # Check against parked cars
        for parked_car in self.parked_cars:
            parked_corners = parked_car.get_corners()
            parked_min_x = parked_corners[:, 0].min() - collision_margin
            parked_max_x = parked_corners[:, 0].max() + collision_margin
            parked_min_y = parked_corners[:, 1].min() - collision_margin
            parked_max_y = parked_corners[:, 1].max() + collision_margin
            
            # AABB collision check
            if not (vehicle_max_x < parked_min_x or vehicle_min_x > parked_max_x or
                    vehicle_max_y < parked_min_y or vehicle_min_y > parked_max_y):
                return True  # Collision detected
        
        # Check against lot boundaries
        boundary_margin = 0.3  # Don't allow driving outside lot
        if (vehicle_min_x < boundary_margin or vehicle_max_x > self.lot_width - boundary_margin or
            vehicle_min_y < boundary_margin or vehicle_max_y > self.lot_depth - boundary_margin):
            return True  # Out of bounds
        
        return False  # No collision
