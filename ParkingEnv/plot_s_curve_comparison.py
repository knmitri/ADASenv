"""Plot planned S-curve path vs actual tracked path."""

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from KarlsPlayground.testing import ControllerTestEnv


def main() -> None:
    env = ControllerTestEnv(dt=0.05)

    s_path = env.create_s_curve_path(start=(0.0, 0.0), num_points=50)
    result = env.run_test(s_path, "S-Curve Plot Run (variable speed)", max_steps=600)

    planned_x = [wp.x for wp in result["path"]]
    planned_y = [wp.y for wp in result["path"]]

    actual_x = [p[0] for p in result["positions"]]
    actual_y = [p[1] for p in result["positions"]]
    actual_heading = result["headings"]
    desired_heading = result.get("desired_headings", [])
    if not desired_heading:
        desired_heading = [0.0 for _ in actual_heading]

    desired_heading_deg = np.degrees(np.unwrap(np.array(desired_heading)))
    actual_heading_deg = np.degrees(np.unwrap(np.array(actual_heading)))

    # Cross-track error to planned path (nearest-neighbor distance) for diagnosis.
    path_xy = np.column_stack((planned_x, planned_y))
    cte = []
    for px, py in result["positions"]:
        d2 = np.sum((path_xy - np.array([px, py])) ** 2, axis=1)
        cte.append(float(np.sqrt(np.min(d2))))
    cte = np.array(cte)

    fig, axes = plt.subplots(1, 4, figsize=(26, 6))

    # XY path comparison
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

    # Heading comparison over time
    ax1 = axes[1]
    steps = np.arange(1, len(result["positions"]) + 1)
    ax1.plot(steps, desired_heading_deg, "--", linewidth=1.8, label="Calculated path heading [deg]")
    ax1.plot(steps, actual_heading_deg, "-", linewidth=1.8, label="Actual vehicle heading [deg]")
    ax1.set_title("Heading Over Time")
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Heading [deg]")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")

    # Control signals over time
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
        f"Final Pos Err={result['final_pos_error']:.3f}m, Final Heading Err={result['final_heading_error']:.3f}rad, "
        f"Speed mode={result.get('fixed_speed_mps', None)} m/s",
        fontsize=11,
    )

    fig.tight_layout(rect=[0, 0.02, 1, 0.95])

    out_path = Path(__file__).parent / "s_curve_comparison.png"
    fig.savefig(out_path, dpi=150)

    print(f"Saved plot: {out_path}")
    print(f"Success: {result['success']}")
    print(f"Steps: {result['steps']}")
    print(f"Final position error: {result['final_pos_error']:.4f} m")
    print(f"Final heading error: {result['final_heading_error']:.4f} rad")


if __name__ == "__main__":
    main()
