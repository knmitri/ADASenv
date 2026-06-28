"""Trajectory generation and path-sampling helpers."""

from __future__ import annotations

from typing import Iterable, Protocol

import numpy as np


class WaypointLike(Protocol):
    x: float
    y: float
    direction: int
    heading: float


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def direction_heading(tangent: float, direction: int) -> float:
    """Convert path tangent into vehicle heading for forward/reverse travel."""
    if direction < 0:
        return normalize_angle(tangent + np.pi)
    return normalize_angle(tangent)


def catmull_rom_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Sample one Catmull-Rom point."""
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        (2.0 * p1[0])
        + (-p0[0] + p2[0]) * t
        + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        (2.0 * p1[1])
        + (-p0[1] + p2[1]) * t
        + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
    )
    return float(x), float(y)


def build_tracking_samples(
    path: Iterable[WaypointLike],
    points_per_segment: int = 10,
    linear_spacing: float = 0.5,
) -> list[tuple[float, float, int, float]]:
    """Densify waypoints into tracker samples of (x, y, direction, heading)."""
    waypoints = list(path)
    if not waypoints:
        return []

    xy_dir: list[tuple[float, float, int]] = []
    if len(waypoints) < 3:
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            segment_len = float(np.hypot(p2.x - p1.x, p2.y - p1.y))
            steps = max(5, int(np.ceil(segment_len / linear_spacing)))
            for t in np.linspace(0.0, 1.0, steps, endpoint=False):
                x = p1.x + t * (p2.x - p1.x)
                y = p1.y + t * (p2.y - p1.y)
                xy_dir.append((float(x), float(y), int(p1.direction)))
    else:
        for i in range(len(waypoints) - 1):
            p0 = waypoints[max(i - 1, 0)]
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            p3 = waypoints[min(i + 2, len(waypoints) - 1)]
            for t in np.linspace(0.0, 1.0, points_per_segment, endpoint=False):
                x, y = catmull_rom_point(
                    (p0.x, p0.y),
                    (p1.x, p1.y),
                    (p2.x, p2.y),
                    (p3.x, p3.y),
                    float(t),
                )
                xy_dir.append((x, y, int(p1.direction)))

    last = waypoints[-1]
    xy_dir.append((float(last.x), float(last.y), int(last.direction)))

    if len(xy_dir) == 1:
        return [(xy_dir[0][0], xy_dir[0][1], xy_dir[0][2], float(waypoints[0].heading))]

    samples: list[tuple[float, float, int, float]] = []
    for i, (x, y, direction) in enumerate(xy_dir):
        if i < len(xy_dir) - 1:
            nx, ny = xy_dir[i + 1][0], xy_dir[i + 1][1]
            tangent = float(np.arctan2(ny - y, nx - x))
        else:
            px, py = xy_dir[i - 1][0], xy_dir[i - 1][1]
            tangent = float(np.arctan2(y - py, x - px))
        samples.append((x, y, direction, direction_heading(tangent, direction)))
    return samples
