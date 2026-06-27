"""Reusable controller test harness and synthetic paths."""

from __future__ import annotations

from typing import Any

import numpy as np

from KarlsPlayground.control.path_planner import PathWaypoint
from KarlsPlayground.control.trajectory_planner import TrajectoryTracker
from KarlsPlayground.simulation.vehicle import AudiA5


class ControllerTestEnv:
    """Test harness for trajectory tracking controller."""

    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.vehicle = AudiA5()
        self.tracker = TrajectoryTracker(
            lookahead_dist=1.5,
            max_speed=2.0,
            max_reverse_speed=1.0,
        )

    @staticmethod
    def create_straight_path(start: tuple[float, float], end: tuple[float, float], num_points: int = 20) -> list[PathWaypoint]:
        xs = np.linspace(start[0], end[0], num_points)
        ys = np.linspace(start[1], end[1], num_points)
        heading = float(np.arctan2(ys[-1] - ys[0], xs[-1] - xs[0]))
        return [PathWaypoint(x=float(x), y=float(y), direction=1, heading=heading) for x, y in zip(xs, ys)]

    @staticmethod
    def create_s_curve_path(start: tuple[float, float], num_points: int = 50) -> list[PathWaypoint]:
        path: list[PathWaypoint] = []
        for i in range(num_points):
            s = i / max(1, num_points - 1)
            x = start[0] + 16.0 * s
            y = start[1] + 2.0 * np.sin(2.0 * np.pi * s)
            dx_ds = 16.0
            dy_ds = 4.0 * np.pi * np.cos(2.0 * np.pi * s)
            heading = float(np.arctan2(dy_ds, dx_ds))
            path.append(PathWaypoint(x=float(x), y=float(y), direction=1, heading=heading))
        return path

    @staticmethod
    def create_parking_maneuver(start: tuple[float, float], goal: tuple[float, float], num_points: int = 40) -> list[PathWaypoint]:
        path: list[PathWaypoint] = []
        approach_end = (goal[0], goal[1] + 4.0)

        for i in range(num_points // 2):
            t = i / (num_points // 2)
            x = start[0] + t * (approach_end[0] - start[0])
            y = start[1] + t * (approach_end[1] - start[1])
            heading = float(np.arctan2(approach_end[1] - start[1], approach_end[0] - start[0]))
            path.append(PathWaypoint(x=float(x), y=float(y), direction=1, heading=heading))

        for i in range(num_points // 2):
            t = i / (num_points // 2)
            x = approach_end[0] + t * (goal[0] - approach_end[0])
            y = approach_end[1] + t * (goal[1] - approach_end[1])
            heading = float(np.arctan2(goal[1] - approach_end[1], goal[0] - approach_end[0]) + np.pi)
            path.append(PathWaypoint(x=float(x), y=float(y), direction=-1, heading=heading))

        return path

    def run_test(
        self,
        path: list[PathWaypoint],
        test_name: str,
        max_steps: int = 500,
        fixed_speed_mps: float | None = None,
        initial_heading: float | None = None,
        verbose: bool = False,
    ) -> dict[str, Any]:
        if not path:
            return {"success": False, "error": "Empty path"}

        self.vehicle.x = path[0].x
        self.vehicle.y = path[0].y
        start_heading = float(path[0].heading if initial_heading is None else initial_heading)
        self.vehicle.yaw = start_heading
        self.vehicle.vx = 0.0
        self.vehicle.steering_angle = 0.0
        self.tracker.set_path(path)

        goal = (path[-1].x, path[-1].y)
        goal_heading = path[-1].heading

        positions: list[tuple[float, float]] = []
        velocities: list[float] = []
        headings: list[float] = []
        desired_headings: list[float] = []
        steering_angles: list[float] = []
        errors_pos: list[float] = []
        errors_heading: list[float] = []

        step_count = 0
        final_pos_error = 0.0
        final_heading_error = 0.0
        max_pos_error = 0.0
        success = False

        if verbose:
            print(f"\n{'=' * 70}\nTest: {test_name}\n{'=' * 70}")

        for step in range(max_steps):
            state = {
                "position": (self.vehicle.x, self.vehicle.y),
                "heading": self.vehicle.yaw,
                "velocity": (self.vehicle.vx, 0.0),
                "steering_angle": self.vehicle.steering_angle,
            }

            acceleration, steering = self.tracker.compute_control(
                state=state,
                goal_heading=goal_heading,
                current_direction=getattr(self.tracker, "active_direction", 1),
                dynamic_obstacles=[],
            )

            if fixed_speed_mps is not None:
                target_v = float(np.sign(fixed_speed_mps) * abs(fixed_speed_mps))
                acceleration = float(np.clip((target_v - self.vehicle.vx) / max(self.dt, 1e-3), -2.8, 2.8))

            self.vehicle.update(acceleration, steering, self.dt)

            positions.append((self.vehicle.x, self.vehicle.y))
            velocities.append(float(self.vehicle.vx))
            headings.append(float(self.vehicle.yaw))
            steering_angles.append(float(self.vehicle.steering_angle))

            if getattr(self.tracker, "filtered_path_yaw", None) is not None:
                desired_headings.append(float(self.tracker.filtered_path_yaw))
            elif hasattr(self.tracker, "spline_path") and self.tracker.spline_path:
                idx = min(self.tracker.current_index, len(self.tracker.spline_path) - 1)
                if len(self.tracker.spline_path[idx]) >= 4:
                    desired_headings.append(float(self.tracker.spline_path[idx][3]))
                else:
                    desired_headings.append(float(path[min(idx, len(path) - 1)].heading))
            else:
                desired_headings.append(float(path[min(self.tracker.current_index, len(path) - 1)].heading))

            pos_error = float(np.hypot(goal[0] - self.vehicle.x, goal[1] - self.vehicle.y))
            heading_error = float(abs(np.arctan2(np.sin(self.vehicle.yaw - goal_heading), np.cos(self.vehicle.yaw - goal_heading))))
            errors_pos.append(pos_error)
            errors_heading.append(heading_error)
            max_pos_error = max(max_pos_error, pos_error)

            if verbose and (step + 1) % 50 == 0:
                print(
                    f"Step {step+1:3d}: pos=({self.vehicle.x:7.2f}, {self.vehicle.y:7.2f}), "
                    f"pos_err={pos_error:6.2f}m, heading_err={heading_error:6.3f}rad, "
                    f"v={self.vehicle.vx:6.2f}m/s, steer={self.vehicle.steering_angle:6.3f}rad"
                )

            if pos_error < 0.3 and heading_error < 0.15:
                success = True
                step_count = step + 1
                final_pos_error = pos_error
                final_heading_error = heading_error
                break

            step_count = step + 1

        if not success:
            final_pos_error = errors_pos[-1] if errors_pos else float("inf")
            final_heading_error = errors_heading[-1] if errors_heading else float("inf")

        return {
            "success": success,
            "test_name": test_name,
            "steps": step_count,
            "final_pos_error": final_pos_error,
            "final_heading_error": final_heading_error,
            "max_pos_error": max_pos_error,
            "avg_velocity": float(np.mean(velocities)) if velocities else 0.0,
            "max_velocity": float(np.max(velocities)) if velocities else 0.0,
            "fixed_speed_mps": fixed_speed_mps,
            "positions": positions,
            "velocities": velocities,
            "headings": headings,
            "desired_headings": desired_headings,
            "steering_angles": steering_angles,
            "path": path,
            "start_pose": {
                "x": float(path[0].x),
                "y": float(path[0].y),
                "heading": start_heading,
            },
        }
