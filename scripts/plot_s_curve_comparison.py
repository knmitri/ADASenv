"""Plot planned S-curve path against the tracked vehicle path."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from _project import PROJECT_ROOT, add_project_root_to_path

DEFAULT_OUTPUT = PROJECT_ROOT / "artifacts" / "s_curve_comparison.png"
add_project_root_to_path()

from KarlsPlayground.testing import ControllerTestEnv


def run_s_curve_case(
    dt: float,
    num_points: int,
    max_steps: int,
    fixed_speed_mps: float | None,
) -> dict[str, Any]:
    """Run the reusable S-curve tracking scenario."""
    env = ControllerTestEnv(dt=dt)
    path = env.create_s_curve_path(start=(0.0, 0.0), num_points=num_points)
    return env.run_test(
        path,
        "S-Curve Plot Run",
        max_steps=max_steps,
        fixed_speed_mps=fixed_speed_mps,
    )


def compute_cross_track_error(
    planned_xy: np.ndarray,
    positions: list[tuple[float, float]],
) -> np.ndarray:
    """Compute nearest-neighbor distance from each tracked point to the planned path."""
    if planned_xy.size == 0 or not positions:
        return np.array([], dtype=float)

    errors = []
    for px, py in positions:
        delta = planned_xy - np.array([px, py], dtype=float)
        errors.append(float(np.sqrt(np.min(np.sum(delta * delta, axis=1)))))
    return np.array(errors, dtype=float)


def plot_result(result: dict[str, Any], output_path: Path, dpi: int) -> None:
    """Create and save the S-curve comparison plot."""
    planned_x = np.array([wp.x for wp in result["path"]], dtype=float)
    planned_y = np.array([wp.y for wp in result["path"]], dtype=float)
    actual_x = np.array([p[0] for p in result["positions"]], dtype=float)
    actual_y = np.array([p[1] for p in result["positions"]], dtype=float)

    planned_xy = np.column_stack((planned_x, planned_y))
    cte = compute_cross_track_error(planned_xy, result["positions"])

    actual_heading = np.array(result["headings"], dtype=float)
    desired_heading = np.array(result.get("desired_headings") or np.zeros_like(actual_heading))
    desired_heading_deg = np.degrees(np.unwrap(desired_heading))
    actual_heading_deg = np.degrees(np.unwrap(actual_heading))
    steps = np.arange(1, len(result["positions"]) + 1)

    fig, axes = plt.subplots(1, 4, figsize=(26, 6))
    try:
        ax0 = axes[0]
        ax0.plot(planned_x, planned_y, "--", linewidth=2.2, label="Planned S path")
        ax0.plot(actual_x, actual_y, "-", linewidth=2.2, label="Actual vehicle path")
        ax0.scatter([planned_x[0]], [planned_y[0]], s=55, marker="o", label="Start")
        ax0.scatter([planned_x[-1]], [planned_y[-1]], s=65, marker="x", label="Goal")
        ax0.set_title("S-Curve: Planned vs Actual (XY)")
        ax0.set_xlabel("X [m]")
        ax0.set_ylabel("Y [m]")
        ax0.axis("equal")
        ax0.grid(True, alpha=0.3)
        ax0.legend(loc="best")

        ax1 = axes[1]
        ax1.plot(steps, desired_heading_deg, "--", linewidth=1.8, label="Path heading [deg]")
        ax1.plot(steps, actual_heading_deg, "-", linewidth=1.8, label="Vehicle heading [deg]")
        ax1.set_title("Heading Over Time")
        ax1.set_xlabel("Step")
        ax1.set_ylabel("Heading [deg]")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="best")

        ax2 = axes[2]
        ax2.plot(steps, result["velocities"], linewidth=1.8, label="Velocity [m/s]")
        ax2.plot(steps, result["steering_angles"], linewidth=1.8, label="Steering [rad]")
        ax2.set_title("Control Signals Over Time")
        ax2.set_xlabel("Step")
        ax2.set_ylabel("Value")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="best")

        ax3 = axes[3]
        ax3.plot(steps, cte, linewidth=1.8, label="Cross-track error [m]")
        ax3.set_title("Tracking Error Over Time")
        ax3.set_xlabel("Step")
        ax3.set_ylabel("Error [m]")
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc="best")

        success_txt = "SUCCESS" if result["success"] else "FAILED"
        fig.suptitle(
            f"S-Curve Controller Comparison | {success_txt} | "
            f"Final Pos Err={result['final_pos_error']:.3f}m, "
            f"Final Heading Err={result['final_heading_error']:.3f}rad, "
            f"Fixed speed={result.get('fixed_speed_mps')} m/s",
            fontsize=11,
        )
        fig.tight_layout(rect=[0, 0.02, 1, 0.95])

        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=dpi)
    finally:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--num-points", type=int, default=50)
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--fixed-speed", type=float, default=None)
    parser.add_argument("--dpi", type=int, default=150)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = run_s_curve_case(
        dt=args.dt,
        num_points=args.num_points,
        max_steps=args.max_steps,
        fixed_speed_mps=args.fixed_speed,
    )
    output_path = args.output.resolve()
    plot_result(result, output_path=output_path, dpi=args.dpi)

    print(f"Saved plot: {output_path}")
    print(f"Success: {result['success']}")
    print(f"Steps: {result['steps']}")
    print(f"Final position error: {result['final_pos_error']:.4f} m")
    print(f"Final heading error: {result['final_heading_error']:.4f} rad")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
