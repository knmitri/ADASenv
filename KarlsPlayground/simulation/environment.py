"""
Realistic parking simulation environment using exact vehicle dynamics.

Inputs: Acceleration (m/s²) and steering angle at wheels (radians)
Units: SI (meters, seconds, radians)
"""

import numpy as np
from typing import Tuple, Dict, Any, List
from KarlsPlayground.scenarios import ParkingScenario, ParkingScenarioFactory
from .vehicle import Vehicle
from .utils import normalize_angle

class ParkingSimulationEnvironment:
    """Realistic parking simulation environment.
    
    This is a custom simulation environment for testing parking controllers.
    Input: acceleration (m/s²) and steering angle (radians)
    Output: vehicle state (position, velocity, heading)
    """
    
    def __init__(
        self,
        vehicle: Vehicle,
        config: Dict[str, Any],
        scenario_factory: ParkingScenarioFactory | None = None,
        scenario: ParkingScenario | None = None,
    ):
        """Initialize the parking simulation.
        
        Args:
            vehicle: Vehicle instance (e.g., AudiA5)
            config: Configuration dictionary
        """
        
        self.vehicle = vehicle
        self.config = config
        self.sim_config = config['simulation']

        self.scenario_factory = scenario_factory or ParkingScenarioFactory.from_config(config)
        self.scenario = scenario or self.scenario_factory.create(goal_slot_index=0)
        self.fixed_scenario = scenario is not None

        # Load config values 

        # Simulation parameters
        self.dt = self.sim_config['dt']
        self.speed_factor = self.sim_config['speed_factor']
        self.render_enabled = self.sim_config['render']
        self.max_episode_steps = self.sim_config['max_episode_steps']

        self.spawn_x = self.sim_config.get('spawn_x')
        self.spawn_y = self.sim_config.get('spawn_y')
        self.spawn_heading = self.sim_config.get('spawn_heading', 0.0)
        self.random_spawn = self.sim_config.get('random_spawn', False)
        self.x_range = self.sim_config.get('start_position_x_range', [0.0, 0.0])
        self.y_range = self.sim_config.get('start_position_y_range', [0.0, 0.0])
        self.world_min_x = float(self.sim_config.get('world_min_x', -self.vehicle.length))
        self.world_min_y = float(self.sim_config.get('world_min_y', -self.vehicle.width))

        # Tolerances from config (support both rad and deg names)
        self.position_tolerance = self.sim_config['goal_position_tolerance']
        self.heading_tolerance = np.deg2rad(self.sim_config['goal_heading_tolerance_deg'])
   
    
        # Trajectory tracking for visualization
        self.trajectory = []
        self.planned_path = []
        self.planned_path_width = vehicle.width

        # Collision flag
        self.collision_occurred = False
        
        # Visualization (lazy-loaded)
        self.visualizer = None

        self.step_count = 0
                # Goal state
        self.goal_slot_index = 0
        self.goal_slot = None  # Store full slot info for whole-car check

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

        wall_t = 0.25
        min_x = self.world_min_x
        min_y = self.world_min_y
        lot_width = self.scenario.lot.lot_width
        lot_length = self.scenario.lot.lot_length

        obstacles.extend([
            {'x': (min_x + lot_width) * 0.5, 'y': min_y - wall_t * 0.5, 'width': lot_width - min_x + 2 * wall_t, 'length': wall_t, 'boundary_wall': True},
            {'x': (min_x + lot_width) * 0.5, 'y': lot_length + wall_t * 0.5, 'width': lot_width - min_x + 2 * wall_t, 'length': wall_t, 'boundary_wall': True},
            {'x': min_x - wall_t * 0.5, 'y': (min_y + lot_length) * 0.5, 'width': wall_t, 'length': lot_length - min_y + 2 * wall_t, 'boundary_wall': True},
            {'x': lot_width + wall_t * 0.5, 'y': (min_y + lot_length) * 0.5, 'width': wall_t, 'length': lot_length - min_y + 2 * wall_t, 'boundary_wall': True},
        ])
        return obstacles

    def get_dynamic_obstacles(self) -> List[Dict[str, Any]]:
        """Return current dynamic obstacles (empty in static scenario)."""
        # extend with moving objects in future; for now no dynamic obstacles.
        return []
    
    def spawn_ego_vehicle(self) -> None:
        """Spawn the ego vehicle."""
        if self.random_spawn:
            self.vehicle.x = np.random.uniform(self.x_range[0], self.x_range[1])
            self.vehicle.y = np.random.uniform(self.y_range[0], self.y_range[1])
            self.vehicle.yaw = np.random.uniform(-np.pi/4, np.pi/4)
        else:
            self.vehicle.x = float(self.spawn_x)
            self.vehicle.y = float(self.spawn_y)
            self.vehicle.yaw = float(self.spawn_heading)
        self.vehicle.vx = 0.0
        self.vehicle.steering_angle = 0.0

    def reset(self, goal_slot_index: int | None = None) -> Dict[str, Any]:
        """Reset the simulation to initial state.
        
        Args:
            goal_slot_index: Specific goal slot index (default: random empty slot)
            
        Returns:
            Initial state dictionary with vehicle position, velocity, heading
        """

        if self.fixed_scenario:
            self.goal_slot_index = self.scenario.goal_slot_index
        else:
            self.goal_slot_index = 0 if goal_slot_index is None else goal_slot_index
            self.scenario = self.scenario_factory.create(goal_slot_index=self.goal_slot_index)

        # spawn the ego vehicle at a random position in the lot (not necessarily the same as start of episode, for more robustness)
        self.spawn_ego_vehicle()

        self.goal_slot = self.scenario.goal_slot

        
        # Reset tracking
        self.trajectory = [(self.vehicle.x, self.vehicle.y)]
        self.step_count = 0
        self.collision_occurred = False
        
        return self._get_state()
    
    def step(self, acceleration: float, steering_angle: float) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """Execute one simulation step.
        
        Args:
            acceleration: Acceleration command in m/s² (positive forward)
            steering_angle: Steering angle at wheels in radians
            waypoints: Optional planned waypoints for visualization
            
        Returns:
            state: Current vehicle state (position, velocity, heading)
            info: Dictionary with episode information (done, errors, rewards)
        """
        # Update vehicle dynamics
        self.vehicle.update(acceleration, steering_angle, self.dt * self.speed_factor)
        
        # Record trajectory
        self.trajectory.append((self.vehicle.x, self.vehicle.y))
        
        # Check if goal is reached
        pos_error = np.sqrt((self.vehicle.x - self.goal_slot.x)**2 + (self.vehicle.y - self.goal_slot.y)**2)
        heading_error = normalize_angle(self.vehicle.yaw - self.goal_slot.heading)
        
        # Check if whole vehicle is inside the slot
        x_min = self.goal_slot.x - self.goal_slot.length / 2
        x_max = self.goal_slot.x + self.goal_slot.length / 2
        y_min = self.goal_slot.y - self.goal_slot.width / 2
        y_max = self.goal_slot.y + self.goal_slot.width / 2
            
        vehicle_corners = self.vehicle.get_corners()
        all_corners_inside = all(
            x_min <= corner[0] <= x_max and y_min <= corner[1] <= y_max
            for corner in vehicle_corners
        )
        
        goal_reached = all_corners_inside and abs(heading_error) < self.heading_tolerance

        # Collision check
        vehicle_corners = self.vehicle.get_corners()
        collision = self.check_collision(vehicle_corners)

        min_x = self.world_min_x
        min_y = self.world_min_y
        lot_width = self.scenario.lot.lot_width
        lot_length = self.scenario.lot.lot_length

        out_of_bounds = np.any(vehicle_corners[:, 0] < min_x) or np.any(vehicle_corners[:, 0] > lot_width) or \
                        np.any(vehicle_corners[:, 1] < min_y) or np.any(vehicle_corners[:, 1] > lot_length)
        if out_of_bounds:
            collision = True
            self.vehicle.x = float(np.clip(self.vehicle.x, min_x + 0.1, lot_width - 0.1))
            self.vehicle.y = float(np.clip(self.vehicle.y, min_y + 0.1, lot_length - 0.1))
            self.vehicle.vx = 0.0

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
        
        # Render if enabled
        if self.render_enabled:
            self.render()
        
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
            'steering_angle': np.float32(self.vehicle.steering_angle)
        }
    
    def render(self) -> None:
        """Render the simulation environment using the visualizer."""
        if not self.render_enabled:
            return
        
        # Lazy-load visualizer on first render
        if self.visualizer is None:
            from ..visualization import SimulationVisualizer
            self.visualizer = SimulationVisualizer(self.config.get("visualization"))
        
        # Render current frame
        self.visualizer.render(
            vehicle=self.vehicle,
            parking_lot=self.scenario.lot,
            goal_slot_index=self.goal_slot_index,
            trajectory=self.trajectory,
            step_count=self.step_count,
            planned_path=self.planned_path,
        )
    

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
        if (vehicle_min_x < self.world_min_x + boundary_margin or
            vehicle_max_x > self.scenario.lot.lot_width - boundary_margin or
            vehicle_min_y < self.world_min_y + boundary_margin or
            vehicle_max_y > self.scenario.lot.lot_length - boundary_margin):
            return True

        return False
    
    def close(self) -> None:
        """Close the visualization."""
        if self.visualizer is not None:
            self.visualizer.close()
            self.visualizer = None
