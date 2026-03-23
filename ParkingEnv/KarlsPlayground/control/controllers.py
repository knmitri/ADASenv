"""Parking control strategies and controllers."""

import logging
import numpy as np
from typing import List, Tuple, Dict, Any
from ..simulation.utils import normalize_angle


class SimpleParkingController:
    """A basic parking controller using proportional control.
    
    This controller uses simple proportional control to steer the vehicle
    towards a goal position and heading.
    """
    
    def __init__(self, kp_lateral: float = 0.5, kp_angular: float = 1.0, 
                 max_forward_speed: float = 1.0):
        """Initialize the parking controller.
        
        Args:
            kp_lateral: Proportional gain for lateral control
            kp_angular: Proportional gain for angular control
            max_forward_speed: Maximum forward speed in m/s
        """
        self.kp_lateral = kp_lateral
        self.kp_angular = kp_angular
        self.max_forward_speed = max_forward_speed
    
    def compute_control(self, state: dict, goal_position: np.ndarray, 
                       goal_heading: float) -> tuple:
        """Compute control commands to reach the goal.
        
        Args:
            state: Current vehicle state dictionary with keys:
                - position: [x, y] in meters
                - heading: yaw angle in radians
                - velocity: [vx] in m/s
            goal_position: Goal position [x, y]
            goal_heading: Goal heading angle in radians
            
        Returns:
            acceleration: Forward acceleration in m/s²
            steering_angle: Steering angle at wheels in radians
        """
        x, y = state['position']
        heading = state['heading']
        velocity = state['velocity'][0]
        
        goal_x, goal_y = goal_position
        
        # Calculate distance and angle to goal
        dx = goal_x - x
        dy = goal_y - y
        distance_to_goal = np.sqrt(dx**2 + dy**2)
        
        # Desired heading to point towards goal
        desired_heading_to_goal = np.arctan2(dy, dx)
        
        # Heading error
        heading_error = normalize_angle(desired_heading_to_goal - heading)
        
        # Steering command (proportional to heading error)
        steering_angle = np.clip(
            self.kp_angular * heading_error, 
            -np.pi/6, 
            np.pi/6
        )
        
        # Acceleration command (proportional to distance)
        if distance_to_goal > 0.5:
            target_speed = min(self.max_forward_speed, distance_to_goal)
        else:
            target_speed = 0.1
        
        speed_error = target_speed - velocity
        acceleration = np.clip(speed_error, -1.0, 1.0)
        
        return acceleration, steering_angle


from .path_planner import MotionPlanner, PathWaypoint
from .trajectory_planner import TrajectoryTracker


class AdvancedParkingController:
    """High level controller that uses path and trajectory planning with obstacle avoidance."""

    def __init__(self,
                 planner_resolution: float = 0.25,
                 lookahead_dist: float = 1.5,
                 max_speed: float = 2.0,
                 max_reverse_speed: float = 1.0):
        self.motion_planner = MotionPlanner(resolution=planner_resolution)
        self.tracker = TrajectoryTracker(lookahead_dist=lookahead_dist,
                                       max_speed=max_speed,
                                       max_reverse_speed=max_reverse_speed)
        self.path: List[PathWaypoint] = []
        self.goal: Tuple[float, float] = (0.0, 0.0)

    def plan(self,
             start: Tuple[float, float],
             goal: Tuple[float, float],
             x_bounds: Tuple[float, float],
             y_bounds: Tuple[float, float],
             static_obstacles: List[Dict[str, Any]],
             start_direction: int = None,
             start_heading: float = 0.0,
             goal_heading: float = 0.0,
             vehicle_size: Tuple[float, float] = (4.67, 1.86)):
        self.goal = goal

        # Determine initial best start direction based on alignment
        if start_direction is None:
            angle_diff = abs(np.arctan2(np.sin(goal_heading - start_heading), np.cos(goal_heading - start_heading)))
            start_direction = -1 if angle_diff > np.pi / 2 else 1

        candidates = []
        for dir_try in (1, -1):
            candidate_path = self.motion_planner.plan(
                start,
                goal,
                x_bounds,
                y_bounds,
                static_obstacles,
                dir_try,
                start_heading,
                goal_heading,
                vehicle_size
            )
            dir_changes = sum(1 for i in range(1, len(candidate_path)) if candidate_path[i].direction != candidate_path[i-1].direction)
            candidate_score = len(candidate_path) + 15 * dir_changes
            candidates.append((candidate_score, dir_try, candidate_path))

        best_score, best_direction, best_path = min(candidates, key=lambda c: c[0])
        self.path = best_path

        if best_direction != start_direction:
            logging.info(f"AdvancedParkingController selected direction {best_direction} over {start_direction} (score {best_score})")

        self.tracker.set_path(self.path)
        return self.path
        self.tracker.set_path(self.path)
        return self.path

    def get_planned_path(self):
        return self.path

    def get_spline_path(self):
        return self.tracker.spline_path if hasattr(self.tracker, 'spline_path') else []

    def compute_control(self,
                        state: Dict[str, Any],
                        goal_heading: float,
                        dynamic_obstacles: List[Dict[str, Any]] = None) -> Tuple[float, float]:
        current_direction = 1
        if self.tracker.path and 0 <= self.tracker.current_index < len(self.tracker.path):
            current_direction = self.tracker.path[self.tracker.current_index].direction
        return self.tracker.compute_control(state, goal_heading, current_direction, dynamic_obstacles)

    def is_done(self, state: Dict[str, Any]) -> bool:
        # Include goal heading in completion check to get into parking orientation
        return self.tracker.is_complete(state, self.goal, goal_heading=0.0)
