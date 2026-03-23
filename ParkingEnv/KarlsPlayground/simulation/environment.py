"""
Realistic parking simulation environment using exact vehicle dynamics.

Inputs: Acceleration (m/s²) and steering angle at wheels (radians)
Units: SI (meters, seconds, radians)
"""

import numpy as np
from typing import Tuple, Dict, Any, Optional, List
from .vehicle import Vehicle
from .scenario import ParkingScenario
from .utils import normalize_angle
from ..config_loader import get_config


class AudiA5(Vehicle):
    """Audi A5 vehicle model with realistic dimensions and kinematic bicycle model."""
    
    def __init__(self):
        """Initialize AudiA5 with configuration parameters."""
        super().__init__()
        config = get_config()
        vehicle_config = config['vehicle']['audi_a5']
        
        self._LENGTH = vehicle_config['length']
        self._WIDTH = vehicle_config['width']
        self._WHEELBASE = vehicle_config['wheelbase']
        self._REAR_TO_CENTER = self._WHEELBASE / 2
        self.MAX_SPEED = vehicle_config['max_speed']

        # Steering angle config from loader is already radians, but support explicit degree values
        if 'max_steering_angle' in vehicle_config:
            self.MAX_STEERING_ANGLE = vehicle_config['max_steering_angle']
        else:
            steering_deg = vehicle_config.get('max_steering_angle_deg', 45.0)
            self.MAX_STEERING_ANGLE = np.deg2rad(steering_deg)

        self.MAX_ACCELERATION = vehicle_config['max_acceleration']
        self.MAX_DECELERATION = vehicle_config['max_deceleration']
    
    # Vehicle dimensions (in meters)
    _LENGTH = 4.67      # Front bumper to rear bumper
    _WIDTH = 1.86       # Side mirror to side mirror
    _WHEELBASE = 2.82   # Front axle to rear axle distance
    
    # Rear axle to center offset (for correct position reference)
    _REAR_TO_CENTER = _WHEELBASE / 2
    
    @property
    def length(self) -> float:
        """Vehicle length in meters."""
        return self._LENGTH
    
    @property
    def width(self) -> float:
        """Vehicle width in meters."""
        return self._WIDTH
    
    @property
    def wheelbase(self) -> float:
        """Vehicle wheelbase in meters."""
        return self._WHEELBASE
    
    def get_corners(self) -> np.ndarray:
        """Get the four corners of the vehicle footprint in global frame.
        
        Returns:
            Array of shape (4, 2) with corner positions [x, y]
        """
        cos_y = np.cos(self.yaw)
        sin_y = np.sin(self.yaw)
        
        # Corners relative to rear axle (local frame)
        corners_local = np.array([
            [0, -self._WIDTH/2],           # Rear left
            [0, self._WIDTH/2],            # Rear right
            [self._WHEELBASE, -self._WIDTH/2],  # Front left
            [self._WHEELBASE, self._WIDTH/2],   # Front right
        ])
        
        # Transform to global frame with rear axle reference
        center_offset = self._REAR_TO_CENTER
        ax_x = self.x + center_offset * cos_y
        ax_y = self.y + center_offset * sin_y
        
        corners_global = np.zeros_like(corners_local)
        for i, corner in enumerate(corners_local):
            corners_global[i, 0] = ax_x + corner[0] * cos_y - corner[1] * sin_y
            corners_global[i, 1] = ax_y + corner[0] * sin_y + corner[1] * cos_y
        
        return corners_global
    
    def update(self, acceleration: float, steering_angle: float, dt: float) -> None:
        """Update vehicle state using kinematic bicycle model.
        
        Args:
            acceleration: Acceleration in m/s² (positive forward, negative reverse)
            steering_angle: Steering angle at front wheels (radians)
            dt: Time step (seconds)
        """
        # Clamp inputs to vehicle limits
        acceleration = np.clip(acceleration, self.MAX_DECELERATION, self.MAX_ACCELERATION)
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        
        # Update velocity (allow negative for reversing)
        self.vx += acceleration * dt
        self.vx = np.clip(self.vx, -self.MAX_SPEED, self.MAX_SPEED)
        
        # Kinematic bicycle model
        if abs(self.vx) > 1e-6:
            # Position update
            self.x += self.vx * dt * np.cos(self.yaw)
            self.y += self.vx * dt * np.sin(self.yaw)
            
            # Heading update based on steering angle
            # For reversing, the steering direction should be flipped
            steering_effect = np.tan(steering_angle) / self._WHEELBASE
            if self.vx < 0:  # Reversing
                steering_effect = -steering_effect
            
            self.yaw += self.vx * dt * steering_effect
            self.yaw = self.normalize_angle(self.yaw)


class ParkingSimulation:
    """Realistic parking simulation environment.
    
    This is a custom simulation environment for testing parking controllers.
    Input: acceleration (m/s²) and steering angle (radians)
    Output: vehicle state (position, velocity, heading)
    """
    
    def __init__(self, vehicle: Vehicle, render: bool = False, use_scenario: bool = True, speed_factor: float = 1.0):
        """Initialize the parking simulation.
        
        Args:
            vehicle: Vehicle instance (e.g., AudiA5)
            render: Whether to render visualization in real-time
            use_scenario: Whether to use new scenario (two-lane with parked cars)
            speed_factor: Factor to speed up simulation (e.g., 2.0 runs 2x faster)
        """
        config = get_config()
        
        self.vehicle = vehicle
        
        # Always create legacy parking lot for backward compatibility
        from .parking_lot import ParkingLot
        parking_lot_config = config['parking_lot']
        self.parking_lot = ParkingLot(
            num_slots=parking_lot_config['num_slots'],
            width=parking_lot_config['width'],
            depth=parking_lot_config['depth']
        )

        # Optional advanced scenario
        if use_scenario:
            self.scenario = ParkingScenario()
        else:
            self.scenario = None
        
        # Simulation parameters from config
        sim_config = config['simulation']
        self.dt = sim_config['dt']
        self.speed_factor = speed_factor
        self.render_enabled = render
        self.max_episode_steps = sim_config['max_episode_steps']
        self.step_count = 0
        
        # Goal state
        self.goal_slot_index = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_heading = 0.0
        self.goal_slot = None  # Store full slot info for whole-car check
        
        # Trajectory tracking
        self.trajectory = []
        self.planned_path = []
        self.planned_path_width = vehicle.width if hasattr(vehicle, 'width') else 2.0

        # Tolerances from config (support both rad and deg names)
        self.position_tolerance = sim_config['goal_position_tolerance']
        heading_tol_deg = sim_config.get('goal_heading_tolerance_deg')
        if heading_tol_deg is not None:
            self.heading_tolerance = np.deg2rad(heading_tol_deg)
        else:
            self.heading_tolerance = sim_config.get('goal_heading_tolerance', np.deg2rad(5.0))
        
        # Collision flag
        self.collision_occurred = False
        
        # Visualization (lazy-loaded)
        self.visualizer = None

    def get_static_obstacles(self) -> List[Dict[str, Any]]:
        """Return static obstacles (parked cars or lot slots) for planning."""
        obstacles = []
        if self.scenario is not None:
            # parked cars are obstacles (static)
            for car in self.scenario.get_parked_cars():
                obstacles.append({
                    'x': car.x,
                    'y': car.y,
                    'width': car.length + 0.6,
                    'length': car.width + 0.6
                })
        elif self.parking_lot is not None:
            # no parked vehicles in legacy parking lot; treat slot borders as weak obstacles
            for slot in self.parking_lot.get_all_slots():
                obstacles.append({
                    'x': slot.x,
                    'y': slot.y,
                    'width': slot.width + 0.3,
                    'length': slot.length + 0.3
                })
        elif self.parking_lot is not None:
            for slot in self.parking_lot.get_all_slots():
                obstacles.append({
                    'x': slot.x,
                    'y': slot.y,
                    'width': slot.length,
                    'length': slot.width
                })
        return obstacles

    def get_dynamic_obstacles(self) -> List[Dict[str, Any]]:
        """Return current dynamic obstacles (empty in static scenario)."""
        # extend with moving objects in future; for now no dynamic obstacles.
        return []
    
    def reset(self, goal_slot_index: Optional[int] = None) -> Dict[str, Any]:
        """Reset the simulation to initial state.
        
        Args:
            goal_slot_index: Specific goal slot index (default: random empty slot)
            
        Returns:
            Initial state dictionary with vehicle position, velocity, heading
        """
        config = get_config()
        start_config = config['simulation']
        
        # Use scenario if available
        if self.scenario:
            # Start from one of the lane centers
            self.vehicle.x = np.random.uniform(2.0, 8.0)
            self.vehicle.y = np.random.uniform(1.0, 3.0)
        else:
            # Legacy mode
            x_range = start_config['start_position_x_range']
            y_range = start_config['start_position_y_range']
            self.vehicle.x = np.random.uniform(x_range[0], x_range[1])
            self.vehicle.y = np.random.uniform(y_range[0], y_range[1])
        
        # Random starting heading
        self.vehicle.yaw = np.random.uniform(-np.pi/4, np.pi/4)  # Face forward-ish
        self.vehicle.vx = 0.0

        # Place ego vehicle between lanes in scenario mode
        if self.scenario:
            lane_spacing = self.scenario.lot_width / self.scenario.num_lanes
            left_lane_right_edge = lane_spacing/2 + self.scenario.lane_width / 2
            right_lane_left_edge = lane_spacing*1.5 - self.scenario.lane_width / 2
            self.vehicle.x = np.random.uniform(left_lane_right_edge + 0.2, right_lane_left_edge - 0.2)
            self.vehicle.y = np.random.uniform(1.5, self.scenario.lot_depth - 1.5)

        # Set goal slot
        if goal_slot_index is None:
            if self.scenario:
                self.goal_slot_index, goal_slot = self.scenario.get_random_empty_slot()
            else:
                self.goal_slot_index, goal_slot = self.parking_lot.get_random_slot()
        else:
            self.goal_slot_index = goal_slot_index
            if self.scenario:
                # Ensure goal slot is free
                self.scenario.parked_cars = [c for c in self.scenario.parked_cars if c.slot_index != goal_slot_index]
                goal_slot = self.scenario.get_slot(goal_slot_index)
            else:
                goal_slot = self.parking_lot.get_slot(goal_slot_index)

        self.goal_x = goal_slot['x'] if self.scenario else goal_slot.x
        self.goal_y = goal_slot['y'] if self.scenario else goal_slot.y
        self.goal_heading = 0.0  # Always park facing forward
        self.goal_slot = goal_slot
        
        # Reset tracking
        self.trajectory = [(self.vehicle.x, self.vehicle.y)]
        self.step_count = 0
        self.collision_occurred = False
        
        return self._get_state()
    
    def step(self, acceleration: float, steering_angle: float,
             waypoints: List[Tuple[float, float, int]] = None,
             planned_path: List[Tuple[float, float]] = None,
             planned_path_width: float = None) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """Execute one simulation step.
        
        Args:
            acceleration: Acceleration command in m/s² (positive forward)
            steering_angle: Steering angle at wheels in radians
            waypoints: Optional planned waypoints for visualization
            planned_path: Optional smooth spline path for visualization
            planned_path_width: Optional width of the planned path band
            
        Returns:
            state: Current vehicle state (position, velocity, heading)
            info: Dictionary with episode information (done, errors, rewards)
        """
        # Update vehicle dynamics
        self.vehicle.update(acceleration, steering_angle, self.dt * self.speed_factor)
        
        # Record trajectory
        self.trajectory.append((self.vehicle.x, self.vehicle.y))
        
        # Check if goal is reached
        pos_error = np.sqrt((self.vehicle.x - self.goal_x)**2 + (self.vehicle.y - self.goal_y)**2)
        heading_error = normalize_angle(self.vehicle.yaw - self.goal_heading)
        
        # Check if whole vehicle is inside the slot
        if self.goal_slot is not None:
            slot_x = self.goal_slot['x'] if isinstance(self.goal_slot, dict) else self.goal_slot.x
            slot_y = self.goal_slot['y'] if isinstance(self.goal_slot, dict) else self.goal_slot.y
            slot_width = self.goal_slot['width'] if isinstance(self.goal_slot, dict) else self.goal_slot.width
            slot_length = self.goal_slot['length'] if isinstance(self.goal_slot, dict) else self.goal_slot.length
            
            x_min = slot_x - slot_width / 2
            x_max = slot_x + slot_width / 2
            y_min = slot_y - slot_length / 2
            y_max = slot_y + slot_length / 2
            
            vehicle_corners = self.vehicle.get_corners()
            all_corners_inside = all(
                x_min <= corner[0] <= x_max and y_min <= corner[1] <= y_max
                for corner in vehicle_corners
            )
            
            goal_reached = all_corners_inside and abs(heading_error) < self.heading_tolerance
        else:
            # Fallback to old logic if no slot info
            goal_reached = (
                pos_error < self.position_tolerance and
                abs(heading_error) < self.heading_tolerance
            )

        # Collision check
        vehicle_corners = self.vehicle.get_corners()
        if self.scenario is not None:
            collision = self.scenario.check_collision(vehicle_corners)
        elif self.parking_lot is not None:
            collision = self.parking_lot.check_collision(vehicle_corners)
        else:
            collision = False

        if collision:
            self.collision_occurred = True

        # Increment step counter
        self.step_count += 1
        # Do not terminate immediately on collision in test-loop, allow attempt to continue
        done = goal_reached or self.step_count >= self.max_episode_steps

        # Get state
        state = self._get_state()

        # Create info dictionary
        info = {
            'done': done,
            'position_error': float(pos_error),
            'heading_error': float(heading_error),
            'goal_reached': goal_reached,
            'collision': collision,
            'collision_occurred': self.collision_occurred,
            'episode_step': self.step_count,
            'goal_slot': self.goal_slot_index,
        }
        
        # Update planned path information (for visualization)
        if planned_path is not None:
            self.planned_path = planned_path
        if planned_path_width is not None:
            self.planned_path_width = planned_path_width

        # Render if enabled
        if self.render_enabled and (self.speed_factor <= 1.0 or self.step_count % max(1, int(self.speed_factor)) == 0):
            self.render(waypoints, planned_path=self.planned_path, planned_path_width=self.planned_path_width)
        
        return state, info
    
    def _get_state(self) -> Dict[str, Any]:
        """Get current vehicle state.
        
        Returns:
            Dictionary with position, velocity, heading
        """
        return {
            'position': np.array([self.vehicle.x, self.vehicle.y], dtype=np.float32),
            'velocity': np.array([self.vehicle.vx], dtype=np.float32),
            'heading': np.float32(self.vehicle.yaw),
            'goal_position': np.array([self.goal_x, self.goal_y], dtype=np.float32),
            'goal_heading': np.float32(self.goal_heading),
        }
    
    def render(self,
               waypoints: List[Tuple[float, float, int]] = None,
               planned_path: List[Tuple[float, float]] = None,
               planned_path_width: float = None) -> None:
        """Render the simulation environment using the visualizer."""
        if not self.render_enabled:
            return

        if planned_path is not None:
            self.planned_path = planned_path
        if planned_path_width is not None:
            self.planned_path_width = planned_path_width
        
        # Lazy-load visualizer on first render
        if self.visualizer is None:
            from ..visualization import SimulationVisualizer
            self.visualizer = SimulationVisualizer()
        
        # Render current frame
        parking_source = self.scenario if self.scenario is not None else self.parking_lot
        self.visualizer.render(
            vehicle=self.vehicle,
            parking_lot=parking_source,
            goal_slot_index=self.goal_slot_index,
            trajectory=self.trajectory,
            step_count=self.step_count,
            waypoints=waypoints,
            planned_path=self.planned_path,
            planned_path_width=self.planned_path_width
        )
    
    def close(self) -> None:
        """Close the visualization."""
        if self.visualizer is not None:
            self.visualizer.close()
            self.visualizer = None
