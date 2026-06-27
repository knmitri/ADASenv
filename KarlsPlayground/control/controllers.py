"""Parking control strategies and controllers."""

import logging
import numpy as np
from typing import List, Tuple, Dict, Any, Optional

from KarlsPlayground.simulation.vehicle import Vehicle
from .path_planner import MotionPlanner, PathWaypoint
from .trajectory_planner import TrajectoryTracker


class ParkingController:
    """High level controller that uses path and trajectory planning with obstacle avoidance."""

    def __init__(self,
                planner_resolution: float,
                max_speed: float,
                max_reverse_speed: float,
                two_stage_parking: bool, 
                vehicle: Vehicle,
                x_bounds: Tuple[float, float],
                y_bounds: Tuple[float, float]):  
        
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds    

        self.motion_planner = MotionPlanner(vehicle=vehicle, resolution=planner_resolution, x_bounds=x_bounds, y_bounds=y_bounds)
        self.tracker = TrajectoryTracker(max_speed=max_speed, max_reverse_speed=max_reverse_speed)
        self.vehicle = vehicle
        self.path: List[PathWaypoint] = []
        
        # Two-stage parking mode
        self.two_stage_parking = two_stage_parking
        self.stage = 1  # 1 = approach to staging point, 2 = reverse into spot
        self.staging_point: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.staging_distance_buffer: float = 0.5  # 0.5m buffer beyond vehicle width

    def _calculate_staging_point(self, goal: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Calculate staging point ahead of the slot along the slot heading.

        This gives the vehicle room to approach, then reverse straight into
        the parking goal while keeping the requested final heading.
        """
        staging_dist = self.vehicle.width + self.staging_distance_buffer  # ~2.36m

        staging_x = goal[0] + staging_dist * np.cos(goal[2])
        staging_y = goal[1] + staging_dist * np.sin(goal[2])
        staging_heading = goal[2] + np.pi/2  # Perpendicular to slot heading for better approach angle
        
        return (staging_x, staging_y, staging_heading)
    
    def _create_reverse_path(self, start: Tuple[float, float], goal: Tuple[float, float, float]) -> List[PathWaypoint]:
        """Create a simple straight-line reverse path into the parking spot.
        
        This creates a short path with 3-5 waypoints in a straight line from 
        staging point to final goal, all with reverse direction.
        """
        num_waypoints = 5
        path = []
        
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            path.append(PathWaypoint(x, y, direction=-1, heading=goal[2]))
        
        return path

    def plan(self,
            start: Tuple[float, float],
            start_heading: float,
            goal: Tuple[float, float, float],          
            static_obstacles: List[Dict[str, Any]],
            start_direction: Optional[int] = None
             ) -> List[PathWaypoint]:
        
        # Store vehicle dimensions for staging point calculation
        vehicle_size = (self.vehicle.length, self.vehicle.width)
        
        self.tracker.wheelbase = self.vehicle.wheelbase
        self.tracker.max_steering = self.vehicle.MAX_STEERING_ANGLE
        
        # **TWO-STAGE PARKING LOGIC**
        if self.two_stage_parking:
            # Calculate staging point (adjacent to parking slot)
            self.staging_point = self._calculate_staging_point(goal)
            self.stage = 1
            
            # Stage 1: Plan from start to staging point (parallel to slot)
            logging.info(f"Stage 1: Planning approach from {start} to staging point {self.staging_point}")
            
            # Clamp staging point to bounds
            staging_x = np.clip(self.staging_point[0], self.x_bounds[0], self.x_bounds[1])
            staging_y = np.clip(self.staging_point[1], self.y_bounds[0], self.y_bounds[1])
            staging_point_clamped = (staging_x, staging_y)

            # Determine initial best start direction based on alignment
            if start_direction is None:
                angle_diff = abs(np.arctan2(np.sin(goal[2] - start_heading), np.cos(goal[2] - start_heading)))
                start_direction = -1 if angle_diff > np.pi / 2 else 1

            candidates = []
            for dir_try in (1, -1):
                # Plan to staging point with goal_heading alignment
                candidate_path = self.motion_planner.plan(
                    start,
                    staging_point_clamped,
                    static_obstacles,
                    dir_try,
                    start_heading,
                    goal[2]  # Align with parking slot heading at staging point
                )
                if not candidate_path:
                    continue

                dir_changes = sum(1 for i in range(1, len(candidate_path)) if candidate_path[i].direction != candidate_path[i-1].direction)
                if len(candidate_path) >= 2:
                    dxf = candidate_path[-1].x - candidate_path[-2].x
                    dyf = candidate_path[-1].y - candidate_path[-2].y
                    end_tangent = np.arctan2(dyf, dxf)
                    end_heading = end_tangent if candidate_path[-1].direction > 0 else (end_tangent + np.pi) % (2 * np.pi)
                    final_heading_err = abs(np.arctan2(np.sin(end_heading - goal[2]), np.cos(end_heading - goal[2])))
                else:
                    final_heading_err = np.pi

                candidate_score = (
                    len(candidate_path)
                    + 28 * dir_changes
                    + 22.0 * final_heading_err
                )
                candidates.append((candidate_score, dir_try, candidate_path))

            if not candidates:
                self.path = self._create_reverse_path(staging_point_clamped, goal)
                self.tracker.set_path(self.path)
                return self.path

            best_score, best_direction, approach_path = min(candidates, key=lambda c: c[0])

            approach_forward_count = sum(1 for p in approach_path if p.direction > 0)
            if approach_forward_count < 2 or approach_path[-1].direction < 0:
                logging.info("Stage 1 fallback: synthesizing forward-only approach to staging point")
                mx = float(np.clip((start[0] + staging_point_clamped[0]) * 0.5, self.x_bounds[0], self.x_bounds[1]))
                my = float(np.clip((start[1] + staging_point_clamped[1]) * 0.5, self.y_bounds[0], self.y_bounds[1]))
                approach_path = [
                    PathWaypoint(float(start[0]), float(start[1]), 1, float(start_heading)),
                    PathWaypoint(mx, my, 1, float(goal[2])),
                    PathWaypoint(float(staging_point_clamped[0]), float(staging_point_clamped[1]), 1, float(goal[2])),
                ]
                best_direction = 1
            
            # Stage 2: Create simple reverse path from staging point to goal
            logging.info(f"Stage 2: Planning reverse from staging point {staging_point_clamped} to goal {goal}")
            reverse_path = self._create_reverse_path(staging_point_clamped, goal)
            
            # Combine both stages into one path
            if not approach_path:
                self.path = reverse_path
            elif not reverse_path:
                self.path = approach_path
            else:
                seam_dist = float(np.hypot(
                    approach_path[-1].x - reverse_path[0].x,
                    approach_path[-1].y - reverse_path[0].y,
                ))
                if seam_dist < 1e-6 and approach_path[-1].direction == reverse_path[0].direction:
                    self.path = approach_path[:-1] + reverse_path
                else:
                    # Keep both seam points when direction changes so tracker can commit cleanly.
                    self.path = approach_path + reverse_path
            
            if best_direction != start_direction:
                logging.info(f"ParkingController selected direction {best_direction} over {start_direction} (approach score {best_score})")
            
            self.tracker.set_path(self.path)
            return self.path
        
        # **ORIGINAL SINGLE-STAGE PLANNING** (if two_stage_parking is False)
        # Determine initial best start direction based on alignment
        if start_direction is None:
            angle_diff = abs(np.arctan2(np.sin(goal[2] - start_heading), np.cos(goal[2] - start_heading)))
            start_direction = -1 if angle_diff > np.pi / 2 else 1

        candidates = []
        for dir_try in (1, -1):
            candidate_path = self.motion_planner.plan(
                start,
                goal,
                static_obstacles,
                dir_try,
                start_heading,
                goal[2]
            )
            dir_changes = sum(1 for i in range(1, len(candidate_path)) if candidate_path[i].direction != candidate_path[i-1].direction)
            if len(candidate_path) >= 2:
                dx0 = candidate_path[1].x - candidate_path[0].x
                dy0 = candidate_path[1].y - candidate_path[0].y
                seg_heading = np.arctan2(dy0, dx0)
                if candidate_path[0].direction < 0:
                    seg_heading = (seg_heading + np.pi) % (2 * np.pi)
                init_heading_err = abs(np.arctan2(np.sin(seg_heading - start_heading), np.cos(seg_heading - start_heading)))

                dxf = candidate_path[-1].x - candidate_path[-2].x
                dyf = candidate_path[-1].y - candidate_path[-2].y
                end_tangent = np.arctan2(dyf, dxf)
                end_heading = end_tangent if candidate_path[-1].direction > 0 else (end_tangent + np.pi) % (2 * np.pi)
                final_heading_err = abs(np.arctan2(np.sin(end_heading - goal[2]), np.cos(end_heading - goal[2])))
            else:
                init_heading_err = np.pi
                final_heading_err = np.pi
            candidate_score = len(candidate_path) + 28 * dir_changes + 18.0 * init_heading_err + 22.0 * final_heading_err
            candidates.append((candidate_score, dir_try, candidate_path))

        best_score, best_direction, best_path = min(candidates, key=lambda c: c[0])
        self.path = best_path

        if best_direction != start_direction:
            logging.info(f"ParkingController selected direction {best_direction} over {start_direction} (score {best_score})")

        self.tracker.set_path(self.path)
        return self.path


    def get_spline_path(self):
        return self.tracker.spline_path if hasattr(self.tracker, 'spline_path') else []

    def compute_control(self,
                        state: Dict[str, Any],
                        goal_heading: float,
                        dynamic_obstacles: List[Dict[str, Any]] = None) -> Tuple[float, float]:
        current_direction = 1
        if hasattr(self.tracker, 'spline_path') and self.tracker.spline_path:
            if 0 <= self.tracker.current_index < len(self.tracker.spline_path):
                current_direction = int(self.tracker.spline_path[self.tracker.current_index][2])
        elif self.tracker.path and 0 <= self.tracker.current_index < len(self.tracker.path):
            current_direction = self.tracker.path[self.tracker.current_index].direction
        return self.tracker.compute_control(state, goal_heading, current_direction, dynamic_obstacles)
