"""Trajectory generation and tracking for planned waypoints."""

import numpy as np
from typing import List, Tuple, Dict, Any
from .path_planner import PathWaypoint
from .path_utils import build_tracking_samples


class TrajectoryTracker:
    """Track and follow a path with pure-pursuit style control and direction changes."""

    def __init__(self, lookahead_dist: float = 1.5, max_speed: float = 2.0, max_reverse_speed: float = 1.0):
        self.lookahead_dist = lookahead_dist
        self.max_speed = max_speed
        self.max_reverse_speed = abs(max_reverse_speed)
        self.min_speed = 0.35
        self.min_reverse_speed = 0.2
        self.path: List[PathWaypoint] = []
        self.current_index = 0
        self.stall_counter = 0
        self.last_progress_dist = float('inf')
        self.wheelbase = 2.82
        self.max_steering = np.pi / 4
        self.filtered_path_yaw = None
        self.active_direction = 1
        self.direction_switch_indices: List[int] = []

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return float(np.arctan2(np.sin(a - b), np.cos(a - b)))

    @staticmethod
    def _circular_mean(angles: List[float]) -> float:
        if not angles:
            return 0.0
        s = float(np.mean(np.sin(angles)))
        c = float(np.mean(np.cos(angles)))
        return float(np.arctan2(s, c))

    def _steering_limit_from_speed(self, speed_abs: float, yaw_rate_limit: float = 0.35) -> float:
        """Return speed-dependent steering limit from yaw-rate constraint."""
        speed_for_limit = max(0.05, float(speed_abs))
        steer_limit_from_yawrate = np.arctan(yaw_rate_limit * self.wheelbase / speed_for_limit)
        return float(np.clip(steer_limit_from_yawrate, 0.08, self.max_steering))

    def _rollout(self,
                 x: float,
                 y: float,
                 yaw: float,
                 vx: float,
                 steer: float,
                 acceleration: float,
                 dt: float,
                 steps: int) -> Tuple[float, float, float, float]:
        nx, ny, nyaw, nvx = x, y, yaw, vx
        for _ in range(steps):
            nvx = float(np.clip(nvx + acceleration * dt, -self.max_reverse_speed, self.max_speed))
            nx += nvx * dt * np.cos(nyaw)
            ny += nvx * dt * np.sin(nyaw)
            if abs(nvx) > 1e-5:
                nyaw += nvx * dt * np.tan(steer) / max(self.wheelbase, 1e-3)
                nyaw = float(np.arctan2(np.sin(nyaw), np.cos(nyaw)))
        return nx, ny, nyaw, nvx

    def _predictive_control(self,
                            x: float,
                            y: float,
                            yaw: float,
                            vx: float,
                            target_x: float,
                            target_y: float,
                            target_direction: int,
                            heading_error: float,
                            dynamic_obstacles: List[Dict[str, Any]] = None) -> Tuple[float, float]:
        target_yaw = np.arctan2(target_y - y, target_x - x)
        if target_direction < 0:
            target_yaw = float((target_yaw + np.pi) % (2 * np.pi))

        steer_set = np.linspace(-self.max_steering, self.max_steering, 9)
        accel_set = np.array([-1.5, -0.8, -0.2, 0.0, 0.3, 0.8, 1.4], dtype=float)
        dt = 0.18
        horizon_steps = 8

        desired_speed = min(self.max_speed if target_direction > 0 else self.max_reverse_speed,
                            max(0.25, np.hypot(target_x - x, target_y - y)))
        if target_direction < 0:
            desired_speed = -desired_speed

        best = None
        best_cost = float('inf')

        for steer in steer_set:
            for accel in accel_set:
                px, py, pyaw, pv = self._rollout(x, y, yaw, vx, float(steer), float(accel), dt, horizon_steps)

                pos_cost = np.hypot(px - target_x, py - target_y)
                yaw_cost = abs(self._angle_diff(pyaw, target_yaw))
                speed_cost = abs(pv - desired_speed)
                steer_cost = abs(steer) / max(self.max_steering, 1e-5)

                obstacle_cost = 0.0
                if dynamic_obstacles:
                    for obs in dynamic_obstacles:
                        ox, oy = obs['x'], obs['y']
                        rr = obs.get('radius', 0.7) + 0.9
                        d = np.hypot(px - ox, py - oy)
                        if d < rr:
                            obstacle_cost += 50.0 + 100.0 * (rr - d)

                cost = 3.0 * pos_cost + 1.8 * yaw_cost + 0.7 * speed_cost + 0.3 * steer_cost + obstacle_cost

                # Prefer steering commands aligned with current heading correction sign.
                if heading_error * steer < 0:
                    cost += 0.2

                if cost < best_cost:
                    best_cost = cost
                    best = (float(accel), float(steer))

        if best is None:
            return 0.0, 0.0
        return best

    def set_path(self, path: List[PathWaypoint]) -> None:
        self.path = path
        self.current_index = 0
        self.stall_counter = 0
        self.last_progress_dist = float('inf')
        self.filtered_path_yaw = None
        self.spline_path = self._generate_spline_path(path)
        self.active_direction = int(path[0].direction) if path else 1
        self.direction_switch_indices = []
        if self.spline_path:
            for i in range(1, len(self.spline_path)):
                if int(self.spline_path[i][2]) != int(self.spline_path[i - 1][2]):
                    self.direction_switch_indices.append(i)

    def _next_direction_switch_index(self, start_idx: int) -> int:
        for idx in self.direction_switch_indices:
            if idx >= start_idx:
                return idx
        return -1

    def _generate_spline_path(self, path: List[PathWaypoint], points_per_segment: int = 10):
        return build_tracking_samples(path, points_per_segment=points_per_segment)

    def _find_goal_point(self, position: Tuple[float, float], current_direction: int, lookahead: float) -> Tuple[float, float, int]:
        if not hasattr(self, 'spline_path') or not self.spline_path:
            # Fallback to raw path points if spline is missing
            if not self.path:
                return position[0], position[1], current_direction
            else:
                self.spline_path = [(wp.x, wp.y, wp.direction, wp.heading) for wp in self.path]

        # Keep following the spline path with a moving index
        x, y = position

        # Keep direction memory coherent with upstream signal when available.
        if current_direction in (-1, 1):
            self.active_direction = int(current_direction)

        # Update current_index to nearest path point if vehicle drifted off path
        allowed_dir = int(self.active_direction)
        closest_index = self.current_index
        closest_dist = float('inf')
        for i in range(max(0, self.current_index - 3), min(len(self.spline_path), self.current_index + 8)):
            if int(self.spline_path[i][2]) != allowed_dir:
                continue
            px, py = self.spline_path[i][0], self.spline_path[i][1]
            dist = np.hypot(px - x, py - y)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        # If local window cannot reacquire, do global nearest search.
        if closest_dist > 1.5:
            for i, sp in enumerate(self.spline_path):
                if int(sp[2]) != allowed_dir:
                    continue
                px, py = sp[0], sp[1]
                dist = np.hypot(px - x, py - y)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_index = i

        # If direction-filtered reacquisition failed, relax as a last resort.
        if not np.isfinite(closest_dist) or closest_dist == float('inf'):
            closest_index = self.current_index
            closest_dist = float('inf')
            for i, sp in enumerate(self.spline_path):
                px, py = sp[0], sp[1]
                dist = np.hypot(px - x, py - y)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_index = i

        self.current_index = closest_index

        # Deterministic phase transition: once near the next seam in forward mode,
        # lock onto the seam index and switch active direction.
        next_switch = self._next_direction_switch_index(self.current_index)
        if self.active_direction > 0 and next_switch >= 0:
            sx, sy = self.spline_path[next_switch][0], self.spline_path[next_switch][1]
            seam_dist = np.hypot(sx - x, sy - y)
            if seam_dist < max(1.6, lookahead * 1.4) or self.current_index >= max(0, next_switch - 1):
                self.current_index = next_switch
                self.active_direction = int(self.spline_path[next_switch][2])
                return sx, sy, self.active_direction

        # If a direction switch point is close, snap target to the switch seam to execute maneuver change.
        for j in range(self.current_index + 1, len(self.spline_path)):
            prev_dir = self.spline_path[j - 1][2]
            cur_dir = self.spline_path[j][2]
            if cur_dir != prev_dir:
                sx, sy = self.spline_path[j][0], self.spline_path[j][1]
                if np.hypot(sx - x, sy - y) < 1.2:
                    self.current_index = j
                    self.active_direction = int(cur_dir)
                    return sx, sy, cur_dir
                break

        # Find lookahead along path arc length from current index (robust after sharp curves).
        acc_len = 0.0
        for i in range(self.current_index, len(self.spline_path) - 1):
            x0, y0 = self.spline_path[i][0], self.spline_path[i][1]
            x1, y1 = self.spline_path[i + 1][0], self.spline_path[i + 1][1]
            acc_len += float(np.hypot(x1 - x0, y1 - y0))
            if acc_len >= lookahead:
                self.current_index = i + 1
                px, py, pd = self.spline_path[i + 1][0], self.spline_path[i + 1][1], self.spline_path[i + 1][2]
                self.active_direction = int(pd)
                return px, py, pd

        # Fall back to the final goal point
        self.current_index = len(self.spline_path) - 1
        px, py, pd = self.spline_path[-1][0], self.spline_path[-1][1], self.spline_path[-1][2]
        self.active_direction = int(pd)
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
        current_steer = float(state.get('steering_angle', 0.0))

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

        final_x, final_y = self.path[-1].x, self.path[-1].y
        final_goal_dist = np.hypot(final_x - x, final_y - y)
        if final_goal_dist < 2.0:
            lookahead = min(lookahead, 0.3)

        target_x, target_y, target_direction = self._find_goal_point((x, y), current_direction, lookahead)

        dx = target_x - x
        dy = target_y - y
        target_dist = np.hypot(dx, dy)

        # Estimate path tangent direction from current index on spline for smoother steering.
        path_yaw = np.arctan2(dy, dx)
        heading_from_path = False
        if hasattr(self, 'spline_path') and self.spline_path and len(self.spline_path[self.current_index]) >= 4:
            # Use a short forward window to smooth target heading and reduce index-hopping jitter.
            i0 = self.current_index
            i1 = min(len(self.spline_path), self.current_index + 6)
            heading_window = [float(self.spline_path[k][3]) for k in range(i0, i1)]
            path_yaw = self._circular_mean(heading_window)
            heading_from_path = True
        if hasattr(self, 'spline_path') and self.current_index < len(self.spline_path) - 1:
            i0 = self.current_index
            i1 = min(self.current_index + 1, len(self.spline_path) - 1)
            px0 = self.spline_path[i0][0]
            py0 = self.spline_path[i0][1]
            nx = self.spline_path[i1][0]
            ny = self.spline_path[i1][1]
            if not heading_from_path:
                # Use global tangent at the local reference point (not target-relative delta).
                path_yaw = np.arctan2(ny - py0, nx - px0)

        # For backward direction, reverse the intended heading
        if target_direction < 0 and not heading_from_path:
            path_yaw = (path_yaw + np.pi) % (2 * np.pi)

        # Low-pass filter target heading on the circle to reduce jitter.
        # Use more aggressive filtering (slower convergence) when heading error is large.
        if self.filtered_path_yaw is None:
            self.filtered_path_yaw = float(path_yaw)
        else:
            # When vehicle is significantly misaligned, pull in path heading more slowly
            # to allow smooth curve building rather than sharp correction
            large_misalignment = abs(self._angle_diff(path_yaw, yaw)) > np.pi / 3  # > 60°
            alpha_yaw = 0.1 if large_misalignment else 0.22
            yaw_step = self._angle_diff(path_yaw, self.filtered_path_yaw)
            self.filtered_path_yaw = float(np.arctan2(
                np.sin(self.filtered_path_yaw + alpha_yaw * yaw_step),
                np.cos(self.filtered_path_yaw + alpha_yaw * yaw_step),
            ))
        path_yaw = self.filtered_path_yaw

        heading_error = np.arctan2(np.sin(path_yaw - yaw), np.cos(path_yaw - yaw))

        # Stanley-style steering to reduce cross-track deviation from planned spline.
        if hasattr(self, 'spline_path') and self.spline_path:
            ref_x = self.spline_path[self.current_index][0]
            ref_y = self.spline_path[self.current_index][1]
        else:
            ref_x, ref_y = target_x, target_y
        # Signed lateral error in path frame. Sign is chosen so Stanley term steers toward the path.
        cross_track_error = (x - ref_x) * np.sin(path_yaw) - (y - ref_y) * np.cos(path_yaw)
        
        # Reduce steering aggressiveness when heading error is large to promote smooth curves.
        # When misaligned (>40°), use lower gains to allow gentle curvature building rather than sharp turns.
        k_heading_base = 1.7
        k_cte_base = 1.2
        
        # Only reduce gains when there's moderate-to-large misalignment (not extreme 90°+ cases)
        if 0.7 < abs(heading_error) < 1.8:  # 40-100 degrees: promote smooth curves
            large_error_scale = max(0.65, 1.0 - (abs(heading_error) - 0.7) / (np.pi * 0.8))
            k_heading = k_heading_base * large_error_scale
            k_cte = k_cte_base * large_error_scale
        else:
            k_heading = k_heading_base
            k_cte = k_cte_base
        
        cte_term = np.arctan2(k_cte * cross_track_error, abs(vx) + 0.4)
        stanley_steer = np.clip(k_heading * heading_error + cte_term, -self.max_steering, self.max_steering)

        # Reduce speed on sharp turns to avoid circling behavior
        heading_penalty = min(1.0, max(0.15, 1.0 - abs(heading_error) / (np.pi/2)))
        # Slow-down on large heading errors to prevent circling
        if abs(heading_error) > 0.5:
            heading_penalty *= 0.5

        # Variable-speed tuning: respect path direction and adjust only speed magnitude.
        # This avoids direction flips that can break path tracking quality.
        if abs(heading_error) > 1.0 and target_dist > 4.0:
            target_speed = 0.15 if target_direction > 0 else -0.15
            heading_penalty = 1.0
        else:
            if target_direction > 0:
                base_speed = min(self.max_speed, target_dist)
                
                # Reduce speed gradually for moderate heading misalignment (40-70 degrees)
                # to allow smooth curve building, but not for extreme misalignments
                if 0.8 < abs(heading_error) < 1.4:
                    base_speed *= 0.65
                
                target_speed = base_speed * heading_penalty
                # Curvature-aware cap from local heading change to keep tracking stable at variable speed.
                if hasattr(self, 'spline_path') and self.spline_path:
                    j = min(len(self.spline_path) - 1, self.current_index + 6)
                    if len(self.spline_path[self.current_index]) >= 4 and len(self.spline_path[j]) >= 4:
                        local_yaw = float(self.spline_path[self.current_index][3])
                        look_yaw = float(self.spline_path[j][3])
                    else:
                        x0, y0 = self.spline_path[self.current_index][0], self.spline_path[self.current_index][1]
                        x1, y1 = self.spline_path[j][0], self.spline_path[j][1]
                        local_yaw = float(np.arctan2(y1 - y0, x1 - x0))
                        look_yaw = local_yaw
                    local_turn = abs(self._angle_diff(look_yaw, local_yaw))
                    curve_cap = float(np.clip(1.6 - 0.9 * local_turn, 0.7, self.max_speed))
                    target_speed = min(target_speed, curve_cap)
                if target_dist > 5.0 and abs(heading_error) < 0.2:
                    target_speed = min(self.max_speed, target_speed * 1.1)
                if final_goal_dist > 1.5:
                    target_speed = np.sign(target_speed) * max(self.min_speed, abs(target_speed))
            else:
                base_speed = min(self.max_reverse_speed, target_dist)
                target_speed = -base_speed * heading_penalty
                if final_goal_dist > 1.5:
                    target_speed = np.sign(target_speed) * max(self.min_reverse_speed, abs(target_speed))

        # If we have big heading error near goal, slow down to allow controlled steering rather than full stop
        if target_dist < 3.0 and abs(heading_error) > 0.5:
            if target_direction > 0:
                target_speed = max(0.12, target_speed * 0.5)
            else:
                target_speed = -max(0.12, abs(target_speed) * 0.5)

        # Smooth braking near end (use spline length, not raw path length)
        spline_len = len(self.spline_path) if hasattr(self, 'spline_path') else len(self.path)
        if self.current_index >= max(0, spline_len - 4):
            target_speed *= 0.4

        # Terminal hold: stop driving once close enough to final pose region.
        if final_goal_dist < 0.9:
            target_speed = 0.0

        # Stuck detection: no progress toward goal
        if target_dist >= self.last_progress_dist - 0.05:
            self.stall_counter += 1
        else:
            self.stall_counter = 0
        self.last_progress_dist = target_dist

        if self.stall_counter >= 15:
            # Force stronger acceleration/deceleration to unstick
            if target_direction > 0:
                target_speed = min(self.max_speed, max(abs(target_speed), 0.45))
            else:
                target_speed = -min(self.max_reverse_speed, max(abs(target_speed), 0.35))

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
        baseline_acceleration = np.clip(kp_speed * (target_speed - vx), -2.8, 2.8)

        # Stability-first tracking: use geometric steering and speed PID directly.
        steering = float(stanley_steer)
        steering = float(np.clip(steering, -self.max_steering, self.max_steering))

        steer_limit = self._steering_limit_from_speed(abs(vx), yaw_rate_limit=0.35)
        steering = float(np.clip(steering, -steer_limit, steer_limit))

        acceleration = float(np.clip(baseline_acceleration, -2.8, 2.8))

        # If nearly stopped but significantly misaligned, keep steering active and creep.
        if abs(vx) < 0.04 and abs(heading_error) > 0.35:
            steer_sign = 1.0 if heading_error >= 0 else -1.0
            steering = float(np.clip(0.8 * current_steer + 0.2 * steer_sign * self.max_steering,
                                     -self.max_steering,
                                     self.max_steering))
            acceleration = 0.35 if target_direction > 0 else -0.35

        return acceleration, steering

    def is_complete(self, state: Dict[str, Any], goal: Tuple[float, float], goal_heading: float = 0.0) -> bool:
        x, y = state['position']
        yaw = state['heading']
        pos_err = np.hypot(goal[0] - x, goal[1] - y)
        heading_err = abs(np.arctan2(np.sin(yaw - goal_heading), np.cos(yaw - goal_heading)))

        return pos_err < 0.25 and heading_err < np.deg2rad(4.5)
