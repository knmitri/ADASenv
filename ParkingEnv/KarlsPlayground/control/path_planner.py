"""Path planning utilities for parking maneuvers."""

import heapq
import logging
import numpy as np
from typing import List, Tuple, Dict, Any, NamedTuple

logger = logging.getLogger(__name__)


class PathWaypoint(NamedTuple):
    x: float
    y: float
    direction: int  # 1 for forward, -1 for reverse


def _grid_coord(point: Tuple[float, float], origin: Tuple[float, float], resolution: float) -> Tuple[int, int]:
    x, y = point
    ox, oy = origin
    return int(round((x - ox) / resolution)), int(round((y - oy) / resolution))


def _world_coord(cell: Tuple[int, int], origin: Tuple[float, float], resolution: float) -> Tuple[float, float]:
    i, j = cell
    ox, oy = origin
    return ox + i * resolution, oy + j * resolution


class MotionPlanner:
    """Simple 2D grid-based A* planner with obstacle avoidance and direction changes."""

    def __init__(self, resolution: float = 0.25):
        self.resolution = resolution

    def _create_grid(self,
                     x_bounds: Tuple[float, float],
                     y_bounds: Tuple[float, float],
                     obstacles: List[Dict[str, Any]],
                     margin: float = 0.3):
        x_min, x_max = x_bounds
        y_min, y_max = y_bounds

        nx = int(np.ceil((x_max - x_min) / self.resolution)) + 1
        ny = int(np.ceil((y_max - y_min) / self.resolution)) + 1

        grid = np.zeros((nx, ny), dtype=np.uint8)

        for obs in obstacles:
            cx = obs['x']
            cy = obs['y']
            w = obs.get('width', obs.get('length', 0))
            h = obs.get('length', obs.get('width', 0))
            r = max(w, h) / 2.0 + margin

            x0 = int(np.floor((cx - r - x_min) / self.resolution))
            x1 = int(np.ceil((cx + r - x_min) / self.resolution))
            y0 = int(np.floor((cy - r - y_min) / self.resolution))
            y1 = int(np.ceil((cy + r - y_min) / self.resolution))

            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(nx - 1, x1)
            y1 = min(ny - 1, y1)

            grid[x0:x1+1, y0:y1+1] = 1

        return grid, (x_min, y_min), (nx, ny)

    def plan(self,
             start: Tuple[float, float],
             goal: Tuple[float, float],
             x_bounds: Tuple[float, float],
             y_bounds: Tuple[float, float],
             obstacles: List[Dict[str, Any]],
             start_direction: int = 1,
             start_heading: float = 0.0,
             goal_heading: float = 0.0,
             vehicle_size: Tuple[float, float] = (4.67, 1.86)) -> List[PathWaypoint]:
        """Find path from start to goal as a list of waypoints with direction and heading.

        Includes vehicle footprint inflation, heading bin state, and final yaw target.
        """
        grid, origin, (nx, ny) = self._create_grid(x_bounds, y_bounds, obstacles,
                                                 margin=0.4 + max(vehicle_size)/2)
        start_cell = _grid_coord(start, origin, self.resolution)
        goal_cell = _grid_coord(goal, origin, self.resolution)

        # If vehicle has drifted outside (given speed factor), clamp to bound region and keep planning.
        if not (0 <= start_cell[0] < nx and 0 <= start_cell[1] < ny):
            clamped_start = (
                float(np.clip(start[0], x_bounds[0] + self.resolution, x_bounds[1] - self.resolution)),
                float(np.clip(start[1], y_bounds[0] + self.resolution, y_bounds[1] - self.resolution))
            )
            logger.warning("Start outside bounds, clamping from %s to %s", start, clamped_start)
            start = clamped_start
            start_cell = _grid_coord(start, origin, self.resolution)

        if not (0 <= goal_cell[0] < nx and 0 <= goal_cell[1] < ny):
            clamped_goal = (
                float(np.clip(goal[0], x_bounds[0] + self.resolution, x_bounds[1] - self.resolution)),
                float(np.clip(goal[1], y_bounds[0] + self.resolution, y_bounds[1] - self.resolution))
            )
            logger.warning("Goal outside bounds, clamping from %s to %s", goal, clamped_goal)
            goal = clamped_goal
            goal_cell = _grid_coord(goal, origin, self.resolution)

        if grid[start_cell] == 1 or grid[goal_cell] == 1:
            return [PathWaypoint(start[0], start[1], start_direction), PathWaypoint(goal[0], goal[1], 1)]

        num_yaw_bins = 16
        angle_step = 2 * np.pi / num_yaw_bins
        start_heading_bin = int(round(start_heading / angle_step)) % num_yaw_bins

        motion_angles = [i * angle_step for i in range(num_yaw_bins)]
        motion_vectors = [(int(round(np.cos(a))), int(round(np.sin(a)))) for a in motion_angles]

        open_set = []
        heapq.heappush(open_set, (0.0, start_cell, start_heading_bin, start_direction))
        came_from = {}
        g_score = {(start_cell, start_heading_bin, start_direction): 0.0}

        def _heuristic(a, b):
            return np.hypot(a[0] - b[0], a[1] - b[1])

        while open_set:
            _, current, heading_bin, direction = heapq.heappop(open_set)
            if current == goal_cell:
                break

            current_cost = g_score[(current, heading_bin, direction)]

            for steer_delta in (-1, 0, 1):
                new_heading_bin = (heading_bin + steer_delta) % num_yaw_bins
                dx, dy = motion_vectors[new_heading_bin]
                next_cell = (current[0] + dx * direction, current[1] + dy * direction)

                if not (0 <= next_cell[0] < nx and 0 <= next_cell[1] < ny):
                    continue
                if grid[next_cell] == 1:
                    continue

                for next_direction in (direction, -direction):
                    if next_direction != direction and abs(steer_delta) > 0:
                        continue

                    direction_switch_cost = 0.0 if next_direction == direction else 8.0
                    move_cost = np.hypot(dx, dy)
                    heading_cost = abs(steer_delta) * 0.5
                    tentative_g = current_cost + move_cost + heading_cost + direction_switch_cost

                    state_key = (next_cell, new_heading_bin, next_direction)
                    if state_key not in g_score or tentative_g < g_score[state_key]:
                        g_score[state_key] = tentative_g
                        priority = tentative_g + _heuristic(next_cell, goal_cell)
                        heapq.heappush(open_set, (priority, next_cell, new_heading_bin, next_direction))
                        came_from[state_key] = (current, heading_bin, direction)

        goal_states = [s for s in g_score.keys() if s[0] == goal_cell]
        if not goal_states:
            logger.warning("MotionPlanner failed to find a state at goal_cell. Returning fallback segment.")
            return [
                PathWaypoint(start[0], start[1], start_direction),
                PathWaypoint(start[0] + np.sign(goal[0] - start[0]) * self.resolution, start[1] + np.sign(goal[1] - start[1]) * self.resolution, start_direction),
                PathWaypoint(goal[0], goal[1], -start_direction)
            ]

        # score goal by distance + heading difference to prefer goal-angle-aligned routes
        goal_heading_bin = int(round(goal_heading / (2*np.pi/16))) % 16
        def goal_eval(state):
            _, yaw_bin, _ = state
            yaw_diff = abs(np.minimum((yaw_bin - goal_heading_bin) % 16, (goal_heading_bin - yaw_bin) % 16))
            return g_score[state] + 0.5 * yaw_diff

        best_state = min(goal_states, key=goal_eval)

        path_states = []
        node = best_state
        while node in came_from:
            path_states.append(node)
            node = came_from[node]
        path_states.append((start_cell, start_heading_bin, start_direction))
        path_states.reverse()

        path = [PathWaypoint(_world_coord(cell, origin, self.resolution)[0],
                             _world_coord(cell, origin, self.resolution)[1],
                             dir_state)
                for cell, _, dir_state in path_states]

        logger.debug("MotionPlanner found %d path points (raw)", len(path))

        # Path smoothing: remove wobbly heads, preserve reversals.
        smoothed = [path[0]]
        for p in path[1:]:
            if np.linalg.norm(np.array([p.x, p.y]) - np.array([smoothed[-1].x, smoothed[-1].y])) > self.resolution * 0.8:
                # if direction change and close, keep it for accurate reverse arc
                if p.direction != smoothed[-1].direction and np.linalg.norm(np.array([p.x, p.y]) - np.array([smoothed[-1].x, smoothed[-1].y])) < self.resolution * 1.5:
                    smoothed.append(p)
                elif p.direction == smoothed[-1].direction:
                    smoothed.append(p)

        if smoothed[-1].x != goal[0] or smoothed[-1].y != goal[1]:
            smoothed.append(PathWaypoint(goal[0], goal[1], smoothed[-1].direction))

        # Post-smooth with simple line simplification (Ramer–Douglas–Peucker-like) for curvature
        def _simplify(points, epsilon=0.3):
            if len(points) < 3:
                return points
            first, last = points[0], points[-1]
            line_vec = np.array([last.x - first.x, last.y - first.y])
            line_len = np.hypot(line_vec[0], line_vec[1])
            max_dist, index = 0.0, 0
            for i in range(1, len(points)-1):
                pt = points[i]
                if line_len == 0:
                    dist = np.hypot(pt.x - first.x, pt.y - first.y)
                else:
                    t = ((pt.x - first.x) * line_vec[0] + (pt.y - first.y) * line_vec[1]) / (line_len**2)
                    proj_x = first.x + t * line_vec[0]
                    proj_y = first.y + t * line_vec[1]
                    dist = np.hypot(pt.x - proj_x, pt.y - proj_y)
                if dist > max_dist:
                    max_dist = dist
                    index = i
            if max_dist > epsilon:
                left = _simplify(points[:index+1], epsilon)
                right = _simplify(points[index:], epsilon)
                return left[:-1] + right
            else:
                return [first, last]

        smooth2 = _simplify(smoothed, epsilon=self.resolution * 0.9)
        # keep direction info from original waypoints via nearest correspondence
        final_path = []
        for p in smooth2:
            # approximate direction by nearest original path waypoint
            nearest = min(path, key=lambda q: np.hypot(p.x - q.x, p.y - q.y))
            final_path.append(PathWaypoint(p.x, p.y, nearest.direction))

        return final_path
