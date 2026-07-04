"""Visualization system for parking simulation."""

import numpy as np
import matplotlib.pyplot as plt
from typing import Any, Dict, List, Tuple
from simulator.config_loader import get_config
from simulator.simulation.utils import normalize_angle


class SimulationVisualizer:
    """Handles visualization and rendering of the parking simulation."""
    
    def __init__(self, config: Dict[str, Any] = None):
        """Initialize the visualizer with matplotlib figure and axes."""
        self.fig = None
        self.ax = None
        self.config = config

    def _visualization_config(self) -> Dict[str, Any]:
        if self.config is not None:
            return self.config
        return get_config()["visualization"]
    
    def render(
        self,
        vehicle,
        parking_lot,
        goal_slot_index: int,
        trajectory: List[Tuple[float, float]],
        step_count: int,
        planned_path: List[Tuple[float, float]] = None,
        goal_pose: Tuple[float, float, float] | None = None,
    ) -> None:
        """Render a single frame of the simulation.
        
        Args:
            vehicle: Vehicle object with position and state
            parking_lot: ParkingLot object with slot information
            goal_slot_index: Index of the goal parking slot
            trajectory: List of (x, y) trajectory points visited
            step_count: Current simulation step number
            planned_path: Optional list of (x, y) points for the planned path
        """
        vis_config = self._visualization_config()
        
        if self.fig is None:
            self.fig, self.ax = plt.subplots(
                figsize=(vis_config['figure_width'], vis_config['figure_height'])
            )
            plt.ion()
        
        self.ax.clear()
        
        # Set axis limits and properties with configurable margin for zoom
        margin = vis_config['margin']

        lot_width = parking_lot.lot_width
        lot_length = parking_lot.lot_length

        self.ax.set_xlim(-margin, lot_width + margin)
        self.ax.set_ylim(-margin, lot_length + margin)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, alpha=vis_config['grid_alpha'])
        
        # Draw all parking slots
        self._draw_parking_slots(parking_lot, goal_slot_index)
        
        # Draw vehicle
        self._draw_vehicle(vehicle)
        
        # Draw trajectory
        self._draw_trajectory(trajectory)

        # Draw planned path as smooth spline approximated line with width
        if planned_path is not None and len(planned_path) > 1:
            self._draw_planned_path(planned_path, vehicle.width)

        
        # Draw info text overlay
        self._draw_info(vehicle, parking_lot.get_slot(goal_slot_index), step_count, goal_pose=goal_pose)
        
        # Set labels and title
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title(f'Parking Simulation - Goal Slot {goal_slot_index}')
        
        plt.pause(0.001)
    
    def _draw_parking_slots(self, parking_lot, goal_slot_index: int) -> None:
        """Draw all parking slots on the axes.
        
        Args:
            parking_lot: ParkingLot or ParkingScenario object
            goal_slot_index: Index of goal slot to highlight
        """
        if parking_lot is None:
            return

        vis_config = self._visualization_config()

        slots = parking_lot.get_all_slots()
        for i, slot in enumerate(slots):
            # Color goal slot differently
            if i == goal_slot_index:
                color = vis_config['slot_color_goal']
                alpha = vis_config['slot_alpha_goal']
                linewidth = 2
            else:
                color = vis_config['slot_color_other']
                alpha = vis_config['slot_alpha_other']
                linewidth = 1

            slot_patch = plt.Polygon(
                slot.get_corners(),
                linewidth=linewidth, edgecolor=vis_config['slot_edge_color'], 
                facecolor=color, alpha=alpha
            )
            self.ax.add_patch(slot_patch)

            self.ax.text(slot.x, slot.y, f'{i}', fontsize=8, ha='center', va='center')
    
    def _draw_vehicle(self, vehicle) -> None:
        """Draw the vehicle with heading indicator.
        
        Args:
            vehicle: Vehicle object with position and heading
        """
        vis_config = self._visualization_config()
        
        corners = vehicle.get_corners()
        
        # Vehicle body as filled polygon
        vehicle_rect = plt.Polygon(
            corners, linewidth=2, edgecolor=vis_config['vehicle_heading_color'], 
            facecolor=vis_config['vehicle_color'], alpha=vis_config['vehicle_alpha'], zorder=10
        )
        self.ax.add_patch(vehicle_rect)
        
        # Vehicle heading indicator (arrow)
        cos_y = np.cos(vehicle.yaw)
        sin_y = np.sin(vehicle.yaw)
        arrow_length = vis_config['arrow_length']
        self.ax.arrow(
            vehicle.x, vehicle.y,
            arrow_length * cos_y, arrow_length * sin_y,
            head_width=0.3, head_length=0.3, 
            fc=vis_config['vehicle_heading_color'], ec=vis_config['vehicle_heading_color'], 
            zorder=11, length_includes_head=True
        )
    
    def _draw_trajectory(self, trajectory: List[Tuple[float, float]]) -> None:
        """Draw the vehicle trajectory path.
        
        Args:
            trajectory: List of (x, y) trajectory points
        """
        vis_config = self._visualization_config()
        
        if len(trajectory) > 1:
            traj = np.array(trajectory)
            self.ax.plot(traj[:, 0], traj[:, 1], color=vis_config['trajectory_color'],
                        alpha=vis_config['trajectory_alpha'], linewidth=1)

    def _draw_planned_path(self, path: List[Tuple[float, float]], width: float) -> None:
        """Draw the planned path with vehicle width visualization."""
        if len(path) < 2:
            return

        pts = np.array(path)
        self.ax.plot(pts[:, 0], pts[:, 1], color='deepskyblue', linestyle='--', linewidth=2, alpha=0.8, zorder=2)

        # Draw expanded band to represent vehicle width
        self.ax.plot(pts[:, 0], pts[:, 1], color='cyan', linewidth=max(3, width*4), alpha=0.25, zorder=1)

    
    def _draw_info(
        self,
        vehicle,
        goal_slot,
        step_count: int,
        goal_pose: Tuple[float, float, float] | None = None,
    ) -> None:
        """Draw information text overlay and goal visualization.
        
        Args:
            vehicle: Vehicle object
            goal_slot: Goal parking slot object or dict
            step_count: Current simulation step
        """
        vis_config = self._visualization_config()

        if goal_pose is not None:
            goal_x, goal_y, goal_heading = goal_pose
        elif isinstance(goal_slot, dict):
            goal_x = goal_slot.get('x', 0.0)
            goal_y = goal_slot.get('y', 0.0)
            goal_heading = goal_slot.get('heading', 0.0)
        elif all(hasattr(goal_slot, attr) for attr in ("x", "y", "heading")):
            goal_x = goal_slot.x
            goal_y = goal_slot.y
            goal_heading = goal_slot.heading
        else:
            goal_x, goal_y, goal_heading = 0.0, 0.0, 0.0

        # Calculate errors
        pos_error = np.sqrt((vehicle.x - goal_x)**2 + (vehicle.y - goal_y)**2)
        heading_error = normalize_angle(vehicle.yaw - goal_heading)
        
        # Draw goal position and heading indicator
        goal_cos = np.cos(goal_heading)
        goal_sin = np.sin(goal_heading)
        
        self.ax.scatter([goal_x], [goal_y], c=vis_config['goal_color'], s=150, 
                       marker='*', zorder=5)
        self.ax.arrow(
            goal_x, goal_y,
            0.8 * goal_cos, 0.8 * goal_sin,
            head_width=0.25, head_length=0.25, 
            fc=vis_config['goal_color'], ec=vis_config['goal_color'], zorder=4, 
            length_includes_head=True
        )
        
        # Create information text
        info = f"Step: {step_count}\n"
        info += f"Pos: ({vehicle.x:.2f}, {vehicle.y:.2f}) m\n"
        info += f"Vel: {vehicle.vx:.2f} m/s\n"
        info += f"Heading: {np.rad2deg(vehicle.yaw):.1f}°\n"
        info += f"Pos Error: {pos_error:.3f} m\n"
        info += f"Heading Error: {np.rad2deg(heading_error):.1f}°"
        
        # Draw info text box
        self.ax.text(0.02, 0.98, info, transform=self.ax.transAxes,
                    fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', 
                            alpha=vis_config['info_box_alpha']))
    
    def close(self) -> None:
        """Close the visualization and cleanup matplotlib resources."""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.ax = None
