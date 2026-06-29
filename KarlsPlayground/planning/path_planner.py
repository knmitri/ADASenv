"""Path planning utilities for parking maneuvers."""

import heapq
import logging
import numpy as np
from typing import List, Tuple, Dict, Any, NamedTuple
from KarlsPlayground.simulation.kinematics import (
    VehicleGeometry,
    kinematic_bicycle_arc_step,
    polygon_intersects_aabb,
    polygon_intersects_polygon,
)
from .trajectory_generator import catmull_rom_point

logger = logging.getLogger(__name__)


class PathWaypoint(NamedTuple):
    x: float
    y: float
    direction: int  # 1 for forward, -1 for reverse
    heading: float = 0.0

class PathPlanner:
    """Hybrid-A*-style bicycle-model path planner for parking maneuvers."""

    def __init__(
        self,
        vehicle,
        resolution: float = 0.25,
        x_bounds: Tuple[float, float] = None,
        y_bounds: Tuple[float, float] = None,
        max_expansions: int = 2000,
        steer_counts: Tuple[int, ...] = (7, 9),
    ):
        self.vehicle = vehicle
        self.resolution = resolution
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.max_expansions = int(max_expansions)
        self.steer_counts = tuple(int(c) for c in steer_counts if int(c) >= 3) or (9,)
        self.geometry = VehicleGeometry.from_vehicle(vehicle)

    def _clamp_point(self,
                     point: Tuple[float, float]) -> Tuple[float, float]:
        if self.x_bounds is None or self.y_bounds is None:
            return float(point[0]), float(point[1])
        return (
            float(np.clip(point[0], self.x_bounds[0] + self.resolution, self.x_bounds[1] - self.resolution)),
            float(np.clip(point[1], self.y_bounds[0] + self.resolution, self.y_bounds[1] - self.resolution)),
        )

    def _path_is_too_straight(self, path: List[PathWaypoint]) -> bool:
        if len(path) <= 3:
            return True

        p0 = np.array([path[0].x, path[0].y], dtype=float)
        p1 = np.array([path[-1].x, path[-1].y], dtype=float)
        base = p1 - p0
        base_len = np.linalg.norm(base)
        if base_len < 1e-6:
            return True

        max_dev = 0.0
        for wp in path[1:-1]:
            p = np.array([wp.x, wp.y], dtype=float)
            t = np.dot(p - p0, base) / (base_len * base_len)
            proj = p0 + t * base
            max_dev = max(max_dev, float(np.linalg.norm(p - proj)))
        return max_dev < 0.8

    def _footprint_hits_obstacle(self,
                                 pose: Tuple[float, float, float],
                                 obstacles: List[Dict[str, Any]],
                                 clearance: float) -> bool:
        x, y, yaw = pose
        footprint = self.geometry.corners(x, y, yaw)
        for obs in obstacles:
            if 'corners' in obs:
                obstacle_polygon = np.array(obs['corners'], dtype=float)
                if polygon_intersects_polygon(footprint, obstacle_polygon):
                    return True
                continue

            ox = obs['x']
            oy = obs['y']
            ow = float(obs.get('width', obs.get('length', 0.0))) + 2.0 * clearance
            oh = float(obs.get('length', obs.get('width', 0.0))) + 2.0 * clearance
            if polygon_intersects_aabb(footprint, (ox, oy), (ow, oh)):
                return True
        return False

    def _is_obstacle_feasible(self,
                              path: List[PathWaypoint],
                              obstacles: List[Dict[str, Any]]) -> bool:
        if not path:
            return False
        annotated = self._annotate_headings(path, start_heading=path[0].heading, goal_heading=path[-1].heading)
        clearance = 0.05
        for wp in annotated:
            if self._footprint_hits_obstacle((wp.x, wp.y, wp.heading), obstacles, clearance):
                return False
        return True

    def _normalize_angle(self, angle: float) -> float:
        return float(np.arctan2(np.sin(angle), np.cos(angle)))

    def _curvature_limit(self, margin: float = 1.0) -> float:
        """Return max achievable curvature for the vehicle bicycle model."""
        wheelbase = self.vehicle.wheelbase
        max_steer = self.vehicle.max_steer
        kappa_max = np.tan(max_steer) / max(wheelbase, 1e-3)
        return float(max(0.05, kappa_max * margin))

    def _max_path_curvature(self, path: List[PathWaypoint]) -> float:
        """Estimate maximum geometric curvature along piecewise-linear path."""
        if len(path) < 3:
            return 0.0

        max_kappa = 0.0
        for i in range(1, len(path) - 1):
            x0, y0 = float(path[i - 1].x), float(path[i - 1].y)
            x1, y1 = float(path[i].x), float(path[i].y)
            x2, y2 = float(path[i + 1].x), float(path[i + 1].y)

            a1 = np.arctan2(y1 - y0, x1 - x0)
            a2 = np.arctan2(y2 - y1, x2 - x1)
            ds = np.hypot(x2 - x1, y2 - y1)
            if ds < 0.12:
                continue

            dyaw = abs(self._normalize_angle(a2 - a1))
            max_kappa = max(max_kappa, float(dyaw / ds))

        return float(max_kappa)

    def _has_achievable_curvature(self,
                                  path: List[PathWaypoint],
                                  margin: float = 0.95) -> bool:
        """Check that path curvature stays under a practical steering margin."""
        return self._max_path_curvature(path) <= self._curvature_limit(margin=margin)

    def _annotate_headings(self,
                           path: List[PathWaypoint],
                           start_heading: float,
                           goal_heading: float) -> List[PathWaypoint]:
        if not path:
            return path
        if len(path) == 1:
            return [PathWaypoint(path[0].x, path[0].y, path[0].direction, self._normalize_angle(start_heading))]

        out: List[PathWaypoint] = []
        for i, p in enumerate(path):
            if i == 0:
                dx = path[1].x - p.x
                dy = path[1].y - p.y
            elif i == len(path) - 1:
                dx = p.x - path[i - 1].x
                dy = p.y - path[i - 1].y
            else:
                dx = path[i + 1].x - path[i - 1].x
                dy = path[i + 1].y - path[i - 1].y

            tangent = float(np.arctan2(dy, dx)) if (abs(dx) + abs(dy)) > 1e-6 else start_heading
            vehicle_heading = tangent if p.direction > 0 else tangent + np.pi
            out.append(PathWaypoint(p.x, p.y, p.direction, self._normalize_angle(vehicle_heading)))

        out[0] = PathWaypoint(out[0].x, out[0].y, out[0].direction, self._normalize_angle(start_heading))
        out[-1] = PathWaypoint(out[-1].x, out[-1].y, out[-1].direction, self._normalize_angle(goal_heading))
        return out

    def _is_dynamically_feasible(self,
                                 path: List[PathWaypoint],
                                 start_heading: float) -> bool:
        if len(path) < 2:
            return False

        max_curvature = self._curvature_limit(margin=0.98)

        v0 = np.array([path[1].x - path[0].x, path[1].y - path[0].y], dtype=float)
        if np.linalg.norm(v0) < 1e-5:
            return False

        first_seg_yaw = float(np.arctan2(v0[1], v0[0]))
        first_vehicle_yaw = first_seg_yaw if path[0].direction > 0 else first_seg_yaw + np.pi
        first_err = abs(self._normalize_angle(first_vehicle_yaw - start_heading))
        if first_err > np.deg2rad(40.0):
            return False

        prev_yaw = first_seg_yaw
        for i in range(1, len(path) - 1):
            dx = path[i + 1].x - path[i].x
            dy = path[i + 1].y - path[i].y
            ds = float(np.hypot(dx, dy))
            if ds < 0.12:
                continue
            yaw = float(np.arctan2(dy, dx))
            dyaw = abs(self._normalize_angle(yaw - prev_yaw))
            curvature = dyaw / max(ds, 0.15)
            if curvature > max_curvature:
                return False
            prev_yaw = yaw

        return True

    def _prepend_heading_transition(self,
                                    path: List[PathWaypoint],
                                    start_heading: float) -> List[PathWaypoint]:
        if len(path) < 2:
            return path

        p0 = np.array([path[0].x, path[0].y], dtype=float)
        p1 = np.array([path[1].x, path[1].y], dtype=float)
        seg_yaw = float(np.arctan2((p1 - p0)[1], (p1 - p0)[0]))
        yaw_err = abs(self._normalize_angle(seg_yaw - start_heading))

        if yaw_err <= np.deg2rad(45.0):
            return path

        lead = p0 + np.array([np.cos(start_heading), np.sin(start_heading)], dtype=float) * 1.0
        lead_x, lead_y = self._clamp_point((float(lead[0]), float(lead[1])))
        lead_wp = PathWaypoint(lead_x, lead_y, path[0].direction)
        return [path[0], lead_wp] + path[1:]

    def _enforce_terminal_heading(self,
                                  path: List[PathWaypoint],
                                  goal_heading: float) -> List[PathWaypoint]:
        """Shape final segment so geometric tangent is consistent with terminal vehicle heading."""
        if len(path) < 2:
            return path

        terminal_dir = 1 if path[-1].direction >= 0 else -1
        tangent_at_goal = goal_heading if terminal_dir > 0 else self._normalize_angle(goal_heading - np.pi)
        backoff = max(0.75, 2.5 * self.resolution)

        gx, gy = path[-1].x, path[-1].y
        anchor_x = gx - backoff * np.cos(tangent_at_goal)
        anchor_y = gy - backoff * np.sin(tangent_at_goal)
        anchor_x, anchor_y = self._clamp_point((float(anchor_x), float(anchor_y)))

        if np.hypot(anchor_x - gx, anchor_y - gy) < max(0.35, self.resolution * 1.2):
            return path

        terminal_anchor = PathWaypoint(anchor_x, anchor_y, path[-1].direction)
        out = list(path)
        if len(out) >= 3 and out[-2].direction == out[-1].direction:
            out[-2] = terminal_anchor
        else:
            out.insert(len(out) - 1, terminal_anchor)
        return out

    def _build_human_like_maneuver(self,
                                   start: Tuple[float, float],
                                   goal: Tuple[float, float],
                                   start_heading: float,
                                   goal_heading: float,
                                   force_turn_sign: float = None) -> List[PathWaypoint]:
        s = np.array(start, dtype=float)
        g = np.array(goal, dtype=float)

        hs = np.array([np.cos(start_heading), np.sin(start_heading)], dtype=float)
        hg = np.array([np.cos(goal_heading), np.sin(goal_heading)], dtype=float)
        ng = np.array([-np.sin(goal_heading), np.cos(goal_heading)], dtype=float)

        dist = float(np.linalg.norm(g - s))
        dist = max(dist, 2.0)

        # Human-like parking: approach, arc, then reverse into slot.
        approach_len = min(4.0, max(1.8, dist * 0.30))
        pre_goal_backoff = min(2.8, max(1.6, dist * 0.22))
        lateral_arc = min(2.2, max(1.2, dist * 0.15))

        if force_turn_sign is None:
            turn_sign = 1.0 if np.dot(s - g, ng) >= 0.0 else -1.0
        else:
            turn_sign = 1.0 if force_turn_sign >= 0 else -1.0

        p0 = self._clamp_point((s[0], s[1]))
        p1 = self._clamp_point(tuple(s + hs * approach_len))

        entry = g - hg * pre_goal_backoff
        p3 = self._clamp_point((entry[0], entry[1]))
        p2 = self._clamp_point(tuple((np.array(p1) + np.array(p3)) * 0.5 + ng * turn_sign * lateral_arc))

        pre_slot = self._clamp_point(tuple(g - hg * 0.8))
        p_goal = self._clamp_point((g[0], g[1]))

        # Ensure pre-slot and goal are distinct; otherwise reverse seam is degenerate.
        if np.hypot(pre_slot[0] - p_goal[0], pre_slot[1] - p_goal[1]) < 0.45:
            alt1 = self._clamp_point(tuple(g + ng * turn_sign * 0.9))
            alt2 = self._clamp_point(tuple(g - ng * turn_sign * 0.9))
            d1 = np.hypot(alt1[0] - p_goal[0], alt1[1] - p_goal[1])
            d2 = np.hypot(alt2[0] - p_goal[0], alt2[1] - p_goal[1])
            pre_slot = alt1 if d1 >= d2 else alt2

        return [
            PathWaypoint(p0[0], p0[1], 1),
            PathWaypoint(p1[0], p1[1], 1),
            PathWaypoint(p2[0], p2[1], 1),
            PathWaypoint(p3[0], p3[1], 1),
            PathWaypoint(pre_slot[0], pre_slot[1], -1),
            PathWaypoint(p_goal[0], p_goal[1], -1),
        ]

    def _catmull_rom(self,
                     p0: Tuple[float, float],
                     p1: Tuple[float, float],
                     p2: Tuple[float, float],
                     p3: Tuple[float, float],
                     t: float) -> Tuple[float, float]:
        return catmull_rom_point(p0, p1, p2, p3, t)

    def _downsample_for_spline(self,
                               path: List[PathWaypoint],
                               spacing: float = 1.0) -> List[PathWaypoint]:
        if len(path) <= 3:
            return path

        anchors = [path[0]]
        last = np.array([path[0].x, path[0].y], dtype=float)

        for i in range(1, len(path) - 1):
            cur = path[i]
            cur_pt = np.array([cur.x, cur.y], dtype=float)
            dist = float(np.linalg.norm(cur_pt - last))
            dir_switch = cur.direction != path[i - 1].direction
            if dir_switch or dist >= spacing:
                anchors.append(cur)
                last = cur_pt

        if anchors[-1] != path[-1]:
            anchors.append(path[-1])
        return anchors

    def _segment_to_spline(self,
                           segment: List[PathWaypoint],
                           direction: int,
                           spacing: float = 0.35) -> List[PathWaypoint]:
        if len(segment) <= 2:
            return segment

        points = [(p.x, p.y) for p in segment]
        sampled: List[PathWaypoint] = [PathWaypoint(points[0][0], points[0][1], direction)]

        for i in range(len(points) - 1):
            p0 = points[max(i - 1, 0)]
            p1 = points[i]
            p2 = points[i + 1]
            p3 = points[min(i + 2, len(points) - 1)]

            seg_len = max(1e-3, float(np.hypot(p2[0] - p1[0], p2[1] - p1[1])))
            n = max(3, int(np.ceil(seg_len / spacing)))

            for k in range(1, n + 1):
                t = k / n
                x, y = self._catmull_rom(p0, p1, p2, p3, t)
                sampled.append(PathWaypoint(x, y, direction))

        return sampled

    def _to_spline_path(self,
                        path: List[PathWaypoint],
                        spacing: float = 0.35) -> List[PathWaypoint]:
        if len(path) <= 2:
            return path

        anchors = self._downsample_for_spline(path, spacing=1.1)
        if len(anchors) <= 2:
            return anchors

        # Build spline per direction block to preserve forward/reverse transitions.
        blocks: List[List[PathWaypoint]] = []
        current_block = [anchors[0]]
        for i in range(1, len(anchors)):
            current_block.append(anchors[i])
            if anchors[i].direction != anchors[i - 1].direction:
                blocks.append(current_block[:-1])
                current_block = [anchors[i - 1], anchors[i]]
        blocks.append(current_block)

        merged: List[PathWaypoint] = []
        for block in blocks:
            if not block:
                continue
            direction = block[0].direction
            spline_block = self._segment_to_spline(block, direction=direction, spacing=spacing)
            if merged and spline_block:
                # Avoid duplicate seam point.
                if np.hypot(merged[-1].x - spline_block[0].x, merged[-1].y - spline_block[0].y) < 1e-3:
                    spline_block = spline_block[1:]
            merged.extend(spline_block)

        return merged if merged else anchors

    def _maneuver_blocks(self, path: List[PathWaypoint]) -> int:
        if not path:
            return 0
        blocks = 1
        for i in range(1, len(path)):
            if path[i].direction != path[i - 1].direction:
                blocks += 1
        return blocks

    def _compress_maneuvers(self, path: List[PathWaypoint], max_blocks: int = 3) -> List[PathWaypoint]:
        if not path:
            return path

        # Build contiguous direction blocks.
        blocks: List[List[PathWaypoint]] = []
        current = [path[0]]
        for i in range(1, len(path)):
            current.append(path[i])
            if path[i].direction != path[i - 1].direction:
                blocks.append(current[:-1])
                current = [path[i - 1], path[i]]
        blocks.append(current)

        while len(blocks) > max_blocks:
            # Remove the shortest middle block first to preserve start/end behavior.
            mid_indices = list(range(1, len(blocks) - 1))
            if not mid_indices:
                break
            shortest_i = min(mid_indices, key=lambda idx: len(blocks[idx]))

            left = blocks[shortest_i - 1]
            right = blocks[shortest_i + 1]
            merged_mid = left + right[1:]
            merged_dir = left[0].direction
            merged_mid = [PathWaypoint(p.x, p.y, merged_dir) for p in merged_mid]

            blocks = blocks[:shortest_i - 1] + [merged_mid] + blocks[shortest_i + 2:]

        flat: List[PathWaypoint] = []
        for b in blocks:
            if not b:
                continue
            if flat and np.hypot(flat[-1].x - b[0].x, flat[-1].y - b[0].y) < 1e-6:
                flat.extend(b[1:])
            else:
                flat.extend(b)
        return flat

    def _curved_parking_candidates(self,
                                   start: Tuple[float, float],
                                   goal: Tuple[float, float],
                                   start_heading: float,
                                   goal_heading: float,
                                   obstacles: List[Dict[str, Any]]) -> List[List[PathWaypoint]]:
        shaped: List[List[PathWaypoint]] = []
        for attempt in range(10):
            turn_sign = 1.0 if attempt % 2 == 0 else -1.0
            candidate = self._build_human_like_maneuver(
                start=start,
                goal=goal,
                start_heading=start_heading + 0.05 * attempt,
                goal_heading=goal_heading,
                force_turn_sign=turn_sign,
            )
            candidate = self._prepend_heading_transition(candidate, start_heading)
            candidate = self._to_spline_path(candidate, spacing=max(0.30, self.resolution * 1.2))
            candidate = self._compress_maneuvers(candidate, max_blocks=3)
            if self._is_obstacle_feasible(candidate, obstacles) and self._is_dynamically_feasible(candidate, start_heading):
                shaped.append(candidate)
        return shaped

    def _bezier_maneuver(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        start_heading: float,
        goal_heading: float,
        direction: int,
        obstacles: List[Dict[str, Any]],
    ) -> List[PathWaypoint]:
        """Generate a smooth single-direction maneuver and accept it only if feasible."""

        p0 = np.array(start, dtype=float)
        p3 = np.array(goal, dtype=float)
        chord = float(np.linalg.norm(p3 - p0))
        if chord < 1e-6:
            return []

        path_start_heading = start_heading if direction > 0 else start_heading + np.pi
        path_goal_heading = goal_heading if direction > 0 else goal_heading + np.pi
        t0 = np.array([np.cos(path_start_heading), np.sin(path_start_heading)], dtype=float)
        t1 = np.array([np.cos(path_goal_heading), np.sin(path_goal_heading)], dtype=float)

        for handle_scale in (0.35, 0.55, 0.8, 1.1, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0):
            handle = chord * handle_scale
            p1 = p0 + t0 * handle
            p2 = p3 - t1 * handle

            candidate: List[PathWaypoint] = []
            for t in np.linspace(0.0, 1.0, 60):
                u = 1.0 - float(t)
                point = (
                    u**3 * p0
                    + 3.0 * u * u * float(t) * p1
                    + 3.0 * u * float(t) * float(t) * p2
                    + float(t) ** 3 * p3
                )
                if not candidate or np.hypot(point[0] - candidate[-1].x, point[1] - candidate[-1].y) > self.resolution * 1.5:
                    candidate.append(PathWaypoint(float(point[0]), float(point[1]), direction))
            candidate.append(PathWaypoint(float(p3[0]), float(p3[1]), direction))
            candidate = self._annotate_headings(candidate, start_heading=start_heading, goal_heading=goal_heading)
            if (
                self._is_obstacle_feasible(candidate, obstacles)
                and self._is_dynamically_feasible(candidate, start_heading)
                and self._has_achievable_curvature(candidate)
            ):
                return candidate
        return []

    def _state_key(self, x: float, y: float, yaw: float, direction: int) -> Tuple[int, int, int, int]:
        yaw_bins = 72
        yaw_bin = int(round(((yaw + np.pi) % (2 * np.pi)) / (2 * np.pi) * yaw_bins)) % yaw_bins
        gx = int(round(x / self.resolution))
        gy = int(round(y / self.resolution))
        return gx, gy, yaw_bin, 1 if direction >= 0 else -1

    def _bicycle_step(self,
                      x: float,
                      y: float,
                      yaw: float,
                      steer: float,
                      direction: int,
                      distance: float,
                      wheelbase: float) -> Tuple[float, float, float]:
        signed_distance = distance * (1.0 if direction >= 0 else -1.0)
        return kinematic_bicycle_arc_step(x, y, yaw, steer, signed_distance, wheelbase)

    def _segment_is_collision_free(self,
                                   x: float,
                                   y: float,
                                   yaw: float,
                                   steer: float,
                                   direction: int,
                                   step_len: float,
                                   wheelbase: float,
                                   obstacles: List[Dict[str, Any]],
                                   clearance: float) -> Tuple[bool, Tuple[float, float, float]]:

        substeps = 3
        nx, ny, nyaw = x, y, yaw
        for _ in range(substeps):
            nx, ny, nyaw = self._bicycle_step(nx, ny, nyaw, steer, direction, step_len / substeps, wheelbase)
            if self.x_bounds is not None and (nx < self.x_bounds[0] or nx > self.x_bounds[1]):
                return False, (x, y, yaw)
            if self.y_bounds is not None and (ny < self.y_bounds[0] or ny > self.y_bounds[1]):
                return False, (x, y, yaw)
            if self._footprint_hits_obstacle((nx, ny, nyaw), obstacles, clearance):
                return False, (x, y, yaw)
        return True, (nx, ny, nyaw)

    def _search_bicycle_path(self,
                             start: Tuple[float, float],
                             goal: Tuple[float, float],
                             start_heading: float,
                             goal_heading: float,
                             start_direction: int,
                             obstacles: List[Dict[str, Any]],
                             steer_count: int,
                             goal_yaw_tolerance_rad: float = np.deg2rad(18.0)) -> List[PathWaypoint]:
        wheelbase = self.vehicle.wheelbase
        max_steer = self.vehicle.max_steer
        steers = np.linspace(-max_steer, max_steer, steer_count)
        step_len = max(0.45, self.resolution * 2.0)
        clearance = max(0.15, self.vehicle.width * 0.22)    

        start = self._clamp_point(start)
        goal = self._clamp_point(goal)

        start_state = (start[0], start[1], self._normalize_angle(start_heading), 1 if start_direction >= 0 else -1)
        start_key = self._state_key(*start_state)

        open_heap: List[Tuple[float, float, Tuple[float, float, float, int]]] = []
        g_cost = {start_key: 0.0}
        state_by_key = {start_key: start_state}
        came_from: Dict[Tuple[int, int, int, int], Tuple[int, int, int, int]] = {}

        def heuristic(x: float, y: float, yaw: float) -> float:
            d = float(np.hypot(goal[0] - x, goal[1] - y))
            h = abs(self._normalize_angle(goal_heading - yaw))
            return d + 0.7 * h

        f0 = heuristic(start_state[0], start_state[1], start_state[2])
        heapq.heappush(open_heap, (f0, 0.0, start_state))

        best_goal_key = None
        max_expansions = self.max_expansions
        expansions = 0

        while open_heap and expansions < max_expansions:
            _, _, current = heapq.heappop(open_heap)
            cx, cy, cyaw, cdir = current
            ckey = self._state_key(cx, cy, cyaw, cdir)
            expansions += 1

            goal_dist = np.hypot(goal[0] - cx, goal[1] - cy)
            goal_yaw_err = abs(self._normalize_angle(goal_heading - cyaw))
            if goal_dist < 0.45 and goal_yaw_err < goal_yaw_tolerance_rad:
                best_goal_key = ckey
                break

            for ndir in (cdir, -cdir):
                switch_penalty = 0.0 if ndir == cdir else 8.0
                for steer in steers:
                    if ndir != cdir and abs(steer) > max_steer * 0.4:
                        continue

                    ok, (nx, ny, nyaw) = self._segment_is_collision_free(
                        cx,
                        cy,
                        cyaw,
                        float(steer),
                        ndir,
                        step_len,
                        wheelbase,
                        obstacles,
                        clearance,
                    )
                    if not ok:
                        continue

                    nkey = self._state_key(nx, ny, nyaw, ndir)
                    turn_penalty = abs(float(steer)) / max_steer
                    reverse_penalty = 0.4 if ndir < 0 else 0.0
                    new_g = g_cost[ckey] + step_len + 0.35 * turn_penalty + reverse_penalty + 2.2 * switch_penalty

                    if nkey not in g_cost or new_g < g_cost[nkey]:
                        g_cost[nkey] = new_g
                        state_by_key[nkey] = (nx, ny, nyaw, ndir)
                        came_from[nkey] = ckey
                        f = new_g + heuristic(nx, ny, nyaw)
                        heapq.heappush(open_heap, (f, new_g, (nx, ny, nyaw, ndir)))

        if best_goal_key is None:
            return []

        keys = [best_goal_key]
        while keys[-1] in came_from:
            keys.append(came_from[keys[-1]])
        keys.reverse()

        planned = [PathWaypoint(state_by_key[k][0], state_by_key[k][1], state_by_key[k][3]) for k in keys]
        if planned:
            # Never snap directly to goal; that creates non-bicycle "teleport" segments.
            tail_dist = float(np.hypot(goal[0] - planned[-1].x, goal[1] - planned[-1].y))
            if tail_dist < 0.35:
                planned.append(PathWaypoint(goal[0], goal[1], planned[-1].direction))
        return planned

    def plan(self,
             start: Tuple[float, float],
             goal: Tuple[float, float],
             obstacles: List[Dict[str, Any]],
             start_direction: int,
             start_heading: float,
             goal_heading: float) -> List[PathWaypoint]:
        """Plan a dynamically feasible path using a bicycle-model state expansion.

        This keeps planning and vehicle dynamics aligned and retries until a feasible
        candidate is found (forward and reverse options, multiple steering densities).
        """
        start = self._clamp_point(start)
        goal = self._clamp_point(goal)
        base_dir = 1 if start_direction >= 0 else -1

        candidates: List[List[PathWaypoint]] = []
        for steer_count in self.steer_counts:
            candidate = self._search_bicycle_path(
                start=start,
                goal=goal,
                start_heading=start_heading,
                goal_heading=goal_heading,
                start_direction=base_dir,
                obstacles=obstacles,
                steer_count=steer_count,
                goal_yaw_tolerance_rad=np.deg2rad(18.0),
            )
            if candidate:
                candidates.append(candidate)
                break

        # Only evaluate opposite start direction if preferred direction did not produce candidates.
        if not candidates:
            for steer_count in self.steer_counts:
                candidate = self._search_bicycle_path(
                    start=start,
                    goal=goal,
                    start_heading=start_heading,
                    goal_heading=goal_heading,
                    start_direction=-base_dir,
                    obstacles=obstacles,
                    steer_count=steer_count,
                    goal_yaw_tolerance_rad=np.deg2rad(18.0),
                )
                if candidate:
                    candidates.append(candidate)
                    break

        # If bicycle search is sparse, synthesize a parking maneuver candidate.
        if not candidates:
            for attempt in range(8):
                turn_sign = 1.0 if attempt % 2 == 0 else -1.0
                human_candidate = self._build_human_like_maneuver(
                    start=start,
                    goal=goal,
                    start_heading=start_heading + 0.06 * attempt,
                    goal_heading=goal_heading,
                    force_turn_sign=turn_sign,
                )
                candidates.append(human_candidate)

        feasible: List[List[PathWaypoint]] = []
        for c in candidates:
            c2 = self._prepend_heading_transition(c, start_heading)
            c2 = self._enforce_terminal_heading(c2, goal_heading)
            if (
                self._is_obstacle_feasible(c2, obstacles)
                and self._is_dynamically_feasible(c2, start_heading)
                and self._has_achievable_curvature(c2)
            ):
                spline_c = self._to_spline_path(c2, spacing=max(0.28, self.resolution * 1.1))
                spline_c = self._enforce_terminal_heading(spline_c, goal_heading)
                if (
                    self._is_obstacle_feasible(spline_c, obstacles)
                    and self._is_dynamically_feasible(spline_c, start_heading)
                    and self._has_achievable_curvature(spline_c)
                ):
                    final_candidate = self._compress_maneuvers(spline_c, max_blocks=3)
                else:
                    final_candidate = self._compress_maneuvers(c2, max_blocks=3)

                if (
                    self._is_obstacle_feasible(final_candidate, obstacles)
                    and self._is_dynamically_feasible(final_candidate, start_heading)
                    and self._has_achievable_curvature(final_candidate)
                ):
                    feasible.append(final_candidate)

        if feasible:
            def score(path: List[PathWaypoint]) -> float:
                total = 0.0
                for i in range(1, len(path)):
                    total += float(np.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y))
                switches = sum(1 for i in range(1, len(path)) if path[i].direction != path[i - 1].direction)
                blocks = self._maneuver_blocks(path)
                extra_blocks = max(0, blocks - 3)
                straight_penalty = 16.0 if self._path_is_too_straight(path) else 0.0
                low_block_penalty = 14.0 if blocks < 2 else 0.0
                return total + 8.0 * switches + 35.0 * extra_blocks + straight_penalty + low_block_penalty

            best = min(feasible, key=score)
            if self._path_is_too_straight(best) or self._maneuver_blocks(best) < 2:
                shaped = self._curved_parking_candidates(
                    start=start,
                    goal=goal,
                    start_heading=start_heading,
                    goal_heading=goal_heading,
                    obstacles=obstacles,
                )
                if shaped:
                    best = min(shaped, key=score)
            best = self._annotate_headings(best, start_heading=start_heading, goal_heading=goal_heading)
            logger.info("Accepted dynamically feasible spline path with %d waypoints", len(best))
            return best

        if candidates:
            logger.info("No fully feasible path found; filtering best available candidate by curvature/dynamics.")
            filtered: List[List[PathWaypoint]] = []
            for candidate in candidates:
                candidate = self._enforce_terminal_heading(candidate, goal_heading)
                if (
                    self._is_obstacle_feasible(candidate, obstacles)
                    and self._is_dynamically_feasible(candidate, start_heading)
                    and self._has_achievable_curvature(candidate)
                ):
                    filtered.append(candidate)

            if filtered:
                best = min(filtered, key=lambda p: (self._path_is_too_straight(p), len(p)))
                return self._annotate_headings(best, start_heading=start_heading, goal_heading=goal_heading)

        shaped = self._curved_parking_candidates(
            start=start,
            goal=goal,
            start_heading=start_heading,
            goal_heading=goal_heading,
            obstacles=obstacles,
        )
        if shaped:
            logger.warning("Falling back to curved parking maneuver candidate.")
            filtered = [
                p for p in shaped
                if self._has_achievable_curvature(p)
                and self._is_dynamically_feasible(p, start_heading)
            ]
            if filtered:
                best = min(filtered, key=lambda p: len(p))
                return self._annotate_headings(best, start_heading=start_heading, goal_heading=goal_heading)

        for direction in (base_dir, -base_dir):
            bezier = self._bezier_maneuver(
                start=start,
                goal=goal,
                start_heading=start_heading,
                goal_heading=goal_heading,
                direction=direction,
                obstacles=obstacles,
            )
            if bezier:
                logger.warning("Falling back to validated smooth Bezier maneuver.")
                return bezier

        logger.warning("No feasible curved path found.")
        return []
