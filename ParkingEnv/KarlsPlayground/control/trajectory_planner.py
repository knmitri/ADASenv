"""Trajectory generation and tracking for planned waypoints."""

import numpy as np
from typing import List, Tuple, Dict, Any
from .path_planner import PathWaypoint


class TrajectoryTracker:
    """Track and follow a path with pure-pursuit style control and direction changes."""

    def __init__(self, lookahead_dist: float = 1.5, max_speed: float = 2.0, max_reverse_speed: float = 1.0):
        self.lookahead_dist = lookahead_dist
        self.max_speed = max_speed
        self.max_reverse_speed = max_reverse_speed
        self.min_speed = 0.35
        self.min_reverse_speed = 0.2
        self.path: List[PathWaypoint] = []
        self.current_index = 0
        self.stall_counter = 0
        self.last_progress_dist = float('inf')

    def set_path(self, path: List[PathWaypoint]) -> None:
        self.path = path
        self.current_index = 0
        self.spline_path = self._generate_spline_path(path)

    def _catmull_rom_point(self, p0, p1, p2, p3, t: float):
        # Catmull-Rom basis
        t2 = t * t
        t3 = t2 * t
        x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t + (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 + (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
        y = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t + (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 + (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
        return x, y

    def _generate_spline_path(self, path: List[PathWaypoint], points_per_segment: int = 10):
        if not path:
            return []
        if len(path) < 3:
            # Densify short path (min two points) to smooth long direct segments
            dense = []
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                segment_len = np.hypot(p2.x - p1.x, p2.y - p1.y)
                steps = max(5, int(np.ceil(segment_len / 0.5)))
                for t in np.linspace(0, 1, steps, endpoint=False):
                    x = p1.x + t * (p2.x - p1.x)
                    y = p1.y + t * (p2.y - p1.y)
                    dense.append((x, y, p1.direction))
            dense.append((path[-1].x, path[-1].y, path[-1].direction))
            return dense

        points = []
        for i in range(len(path) - 1):
            p0 = path[max(i-1, 0)]
            p1 = path[i]
            p2 = path[i+1]
            p3 = path[min(i+2, len(path)-1)]
            for t in np.linspace(0, 1, points_per_segment, endpoint=False):
                x, y = self._catmull_rom_point((p0.x, p0.y), (p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y), t)
                points.append((x, y, p1.direction))
        points.append((path[-1].x, path[-1].y, path[-1].direction))
        return points

    def _find_goal_point(self, position: Tuple[float, float], current_direction: int, lookahead: float) -> Tuple[float, float, int]:
        if not hasattr(self, 'spline_path') or not self.spline_path:
            # Fallback to raw path points if spline is missing
            if not self.path:
                return position[0], position[1], current_direction
            else:
                self.spline_path = [(wp.x, wp.y, wp.direction) for wp in self.path]

        # Keep following the spline path with a moving index
        x, y = position

        # Update current_index to nearest path point if vehicle drifted off path
        closest_index = self.current_index
        closest_dist = float('inf')
        for i in range(max(0, self.current_index - 3), min(len(self.spline_path), self.current_index + 8)):
            px, py, _ = self.spline_path[i]
            dist = np.hypot(px - x, py - y)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        self.current_index = closest_index

        # Find the lookahead point from the current index
        for i in range(self.current_index, len(self.spline_path)):
            px, py, pd = self.spline_path[i]
            dist = np.hypot(px - x, py - y)
            if dist >= lookahead:
                self.current_index = i
                return px, py, pd

        # Fall back to the final goal point
        self.current_index = len(self.spline_path) - 1
        px, py, pd = self.spline_path[-1]
        return px, py, pd

    def compute_control(self,
                        state: Dict[str, Any],
                        goal_heading: float,
                        current_direction: int = 1,
                        dynamic_obstacles: List[Dict[str, Any]] = None) -> Tuple[float, float]:
        """Compute acceleration and steering to follow path."""
        x, y = state['position']
        yaw = state['heading']
        vx = state['velocity'][0]

        if not self.path:
            return 0.0, 0.0

        # Build adaptive lookahead based on orientation error: smaller when sharply misaligned
        base_lookahead = self.lookahead_dist
        # temporary choose first waypoint for yaw checking, fallback if none
        if self.path:
            dx_init = self.path[min(self.current_index, len(self.path)-1)].x - x
            dy_init = self.path[min(self.current_index, len(self.path)-1)].y - y
            target_yaw_init = np.arctan2(dy_init, dx_init)
            yaw_diff = abs(np.arctan2(np.sin(target_yaw_init - yaw), np.cos(target_yaw_init - yaw)))
        else:
            yaw_diff = 0.0

        adaptive_scale = 1.0 - min(0.9, yaw_diff / (np.pi / 2.0))
        lookahead = max(0.4, base_lookahead * adaptive_scale)

        target_x, target_y, target_direction = self._find_goal_point((x, y), current_direction, lookahead)

        dx = target_x - x
        dy = target_y - y
        target_dist = np.hypot(dx, dy)

        # Estimate path tangent direction from current index on spline for smoother steering.
        path_yaw = np.arctan2(dy, dx)
        if hasattr(self, 'spline_path') and self.current_index < len(self.spline_path) - 1:
            nx, ny, _ = self.spline_path[min(self.current_index + 1, len(self.spline_path) - 1)]
            path_yaw = np.arctan2(ny - target_y, nx - target_x)

        # For backward direction, reverse the intended heading
        if target_direction < 0:
            path_yaw = (path_yaw + np.pi) % (2 * np.pi)

        heading_error = np.arctan2(np.sin(path_yaw - yaw), np.cos(path_yaw - yaw))

        # Stronger steering stabilization for yaw error
        kp_steer = 2.5
        steering = np.clip(kp_steer * heading_error, -np.pi/4, np.pi/4)

        # Reduce speed on sharp turns to avoid circling behavior
        heading_penalty = min(1.0, max(0.15, 1.0 - abs(heading_error) / (np.pi/2)))
        # Slow-down on large heading errors to prevent circling
        if abs(heading_error) > 0.5:
            heading_penalty *= 0.5

        # Strong reorientation phase: if heading badly mismatched while path is far,
        # keep moving slowly to rotate and maintain progress
        if abs(heading_error) > 1.0 and target_dist > 4.0:
            if target_direction > 0:
                target_speed = 0.2
            else:
                target_speed = -0.2
            heading_penalty = 1.0

        # Speed target based on direction with minimum speed limits
        if target_direction > 0:
            base_speed = min(self.max_speed, target_dist)
            target_speed = base_speed * heading_penalty
            # if far and mostly aligned, hold higher speed
            if target_dist > 5.0 and abs(heading_error) < 0.2:
                target_speed = min(self.max_speed, target_speed * 1.2)
            target_speed = np.sign(target_speed) * max(self.min_speed, abs(target_speed))
        else:
            base_speed = min(self.max_reverse_speed, target_dist)
            target_speed = -base_speed * heading_penalty
            target_speed = np.sign(target_speed) * max(self.min_reverse_speed, abs(target_speed))

        # If we have big heading error near goal, slow down to allow controlled steering rather than full stop
        if target_dist < 3.0 and abs(heading_error) > 0.5:
            if target_direction > 0:
                target_speed = max(0.12, target_speed * 0.5)
            else:
                target_speed = -max(0.12, abs(target_speed) * 0.5)

        # Smooth braking near end
        if self.current_index >= len(self.path) - 2:
            target_speed *= 0.4

        # Stuck detection: no progress toward goal
        if target_dist >= self.last_progress_dist - 0.05:
            self.stall_counter += 1
        else:
            self.stall_counter = 0
        self.last_progress_dist = target_dist

        if self.stall_counter >= 15:
            # Force stronger acceleration/deceleration to unstick
            if target_direction > 0:
                target_speed = min(self.max_speed, abs(target_speed) + 0.5)
            else:
                target_speed = -min(self.max_reverse_speed, abs(target_speed) + 0.4)
            self.stall_counter = 0

        # Dynamic obstacle buffer
        slowdown_factor = 1.0
        if dynamic_obstacles:
            for obs in dynamic_obstacles:
                ox, oy = obs['x'], obs['y']
                dist = np.hypot(ox - x, oy - y)
                if dist < obs.get('radius', 1.0) + 1.0:
                    slowdown_factor = min(slowdown_factor, dist / (obs.get('radius', 0.5) + 1.0))

        target_speed *= slowdown_factor

        kp_speed = 1.25
        acceleration = np.clip(kp_speed * (target_speed - vx), -2.8, 2.8)

        return float(acceleration), float(steering)

    def is_complete(self, state: Dict[str, Any], goal: Tuple[float, float], goal_heading: float = 0.0) -> bool:
        x, y = state['position']
        yaw = state['heading']
        pos_err = np.hypot(goal[0] - x, goal[1] - y)
        heading_err = abs(np.arctan2(np.sin(yaw - goal_heading), np.cos(yaw - goal_heading)))

        return pos_err < 0.25 and heading_err < 0.1
