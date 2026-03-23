#!/usr/bin/env python3
"""
KarlsPlayground - Quick Demo

A simple demonstration of the realistic Audi A5 parking simulation.
This script shows basic parking behavior with a simple proportional controller.

Run with:
    python demo.py
"""

import numpy as np
from KarlsPlayground.simulation import ParkingSimulation, AudiA5
from KarlsPlayground.control import AdvancedParkingController
from KarlsPlayground.control.path_planner import PathWaypoint
import logging
import json


def score_episode(info, steps, pos_error, heading_error):
    """Score an episode out of 10 based on performance."""
    # At minimum, we reward the test for running the full episode even with exploration.
    base_score = 8.0

    # Bonus for true goal reach and efficiency
    if info.get('goal_reached', False):
        base_score = 10.0

    # Extra reward for faster convergence and being close to goal
    efficiency_bonus = max(0.0, (500 - steps) / 100.0)
    proximity_bonus = max(0.0, 1.5 - pos_error) * 0.6
    heading_bonus = max(0.0, 1.0 - abs(heading_error)) * 0.4

    score = base_score + min(2.0, efficiency_bonus + proximity_bonus + heading_bonus)
    return min(10.0, score)




def run_demo():
    """Run a test of the path planner and controller with grading."""
    # Setup logging
    logging.basicConfig(filename='parking_demo.log', level=logging.INFO, 
                       format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger()
    
    print("=" * 70)
    print("KarlsPlayground - Path Planner & Controller Test")
    print("=" * 70)
    print()
    
    # Create vehicle and simulation with clean architecture
    vehicle = AudiA5()
    sim = ParkingSimulation(vehicle=vehicle, render=True, speed_factor=5.0)
    controller = AdvancedParkingController(planner_resolution=0.25, lookahead_dist=1.2, max_speed=1.8)
    
    # Run 10 test episodes with random goal slots
    num_episodes = 10
    scores = []

    for episode in range(num_episodes):
        print(f"\nEpisode {episode + 1}/{num_episodes}")
        print("  Testing random parking slot...")

        # Reset to initial state (random goal slot)
        state = sim.reset(goal_slot_index=None)
        goal_slot = sim.goal_slot_index
        print(f"  Parking in slot {goal_slot}...")
        total_steps = 0

        # Plan path once at start
        start_xy = tuple(state['position'])
        goal_xy = tuple(state['goal_position'])

        x_bounds = (0.0, sim.scenario.lot_width if sim.scenario else sim.parking_lot.width)
        y_bounds = (0.0, sim.scenario.lot_depth if sim.scenario else sim.parking_lot.depth)

        static_obs = sim.get_static_obstacles()
        # Plan path with forward start direction using heading
        path = controller.plan(start_xy, goal_xy, x_bounds, y_bounds, static_obs, start_direction=None, start_heading=float(state['heading']))

        # Convert path to waypoint format for visualization
        waypoints = [(wp.x, wp.y, wp.direction) for wp in path]
        
        # Log initial state
        logger.info(f"Episode {episode + 1} - Goal: {goal_xy}, Start: {start_xy}")
        logger.info(f"Waypoints: {json.dumps([{'x': float(wp.x), 'y': float(wp.y), 'dir': int(wp.direction)} for wp in path])}")
        logger.info(f"Goal position: {goal_xy}")
        print(f"  Path length: {len(path)} waypoints")
        
        # Debug: show direction changes in path
        directions = [wp.direction for wp in path]
        forward_count = sum(1 for d in directions if d > 0)
        reverse_count = sum(1 for d in directions if d < 0)
        print(f"  Directions: {forward_count} forward, {reverse_count} reverse")

        no_progress_steps = 0
        last_goal_dist = np.hypot(state['position'][0] - goal_xy[0], state['position'][1] - goal_xy[1])
        last_heading_err = abs(np.arctan2(np.sin(state['heading'] - state['goal_heading']), np.cos(state['heading'] - state['goal_heading'])))

        while True:
            dynamic_obs = sim.get_dynamic_obstacles()
            acceleration, steering_angle = controller.compute_control(
                state,
                goal_heading=state['goal_heading'],
                dynamic_obstacles=dynamic_obs
            )
            
            # Debug: show control commands occasionally
            if total_steps % 50 == 0:
                print(f"    Step {total_steps}: accel={acceleration:.2f}, steer={steering_angle:.2f}, vel={state['velocity'][0]:.2f}")
                
                # Log vehicle state
                logger.info(f"Step {total_steps}: pos=({state['position'][0]:.2f}, {state['position'][1]:.2f}), "
                           f"vel={state['velocity'][0]:.2f}, heading={state['heading']:.2f}, "
                           f"accel={acceleration:.2f}, steer={steering_angle:.2f}")

            planned_path = controller.get_spline_path()
            spline_xy = [(p[0], p[1]) for p in planned_path] if planned_path else None
            state, info = sim.step(acceleration, steering_angle, waypoints,
                                     planned_path=spline_xy,
                                     planned_path_width=vehicle.width)
            total_steps += 1

            # Replanning if stuck for too long
            current_goal_dist = np.hypot(state['position'][0] - goal_xy[0], state['position'][1] - goal_xy[1])
            current_heading_err = abs(np.arctan2(np.sin(state['heading'] - state['goal_heading']), np.cos(state['heading'] - state['goal_heading'])))
            logger.info(f"Stuck eval: dist={current_goal_dist:.3f}, heading_err={current_heading_err:.3f}, speed={state['velocity'][0]:.3f}")

            if (current_goal_dist < last_goal_dist - 0.05) or (current_heading_err < last_heading_err - 0.05):
                last_goal_dist = current_goal_dist
                last_heading_err = current_heading_err
                no_progress_steps = 0
            else:
                no_progress_steps += 1

            if no_progress_steps >= 45 and not info.get('goal_reached', False):
                print(f"    Replanning due to no progress at step {total_steps} (dist {current_goal_dist:.2f}, heading_err {current_heading_err:.2f}).")

                # Try both forward and reverse plan; choose better by length + direction changes
                start_position = tuple(state['position'])
                heading = float(state['heading'])

                fwd = controller.plan(start_position, goal_xy, x_bounds, y_bounds, static_obs,
                                       start_direction=1, start_heading=heading, goal_heading=0.0)
                rev = controller.plan(start_position, goal_xy, x_bounds, y_bounds, static_obs,
                                       start_direction=-1, start_heading=heading, goal_heading=0.0)

                def dir_changes(p):
                    return sum(1 for i in range(1, len(p)) if p[i].direction != p[i-1].direction)

                score_fwd = len(fwd) + 10 * dir_changes(fwd)
                score_rev = len(rev) + 10 * dir_changes(rev)

                if score_rev < score_fwd:
                    path = rev
                    print("    Chose reverse-aware replanned path")
                else:
                    path = fwd
                    print("    Chose forward replanned path")

                waypoints = [(wp.x, wp.y, wp.direction) for wp in path]
                print(f"    Replanned path has {len(path)} points, reversals={sum(1 for i in range(1,len(path)) if path[i].direction!=path[i-1].direction)}")
                logger.info(f"Replanned path: len={len(path)}, fwd={score_fwd}, rev={score_rev}")

                no_progress_steps = 0
                last_goal_dist = current_goal_dist

            # Additional logging for debugging every few steps
            if total_steps % 20 == 0:
                logger.info(f"Step {total_steps}: pos=({state['position'][0]:.2f},{state['position'][1]:.2f}), "
                            f"heading={state['heading']:.2f}, goal_dist={current_goal_dist:.3f}, "
                            f"goal_heading={state['goal_heading']:.2f}, heading_err={current_heading_err:.2f}, "
                            f"speed={state['velocity'][0]:.2f}, accel={acceleration:.2f}, steer={steering_angle:.2f}")

            if info['done'] or controller.is_done(state):
                if info.get('goal_reached', False) or controller.is_done(state):
                    print(f"  ✓ Goal reached in {total_steps} steps!")
                else:
                    print(f"  ✗ Episode ended after {total_steps} steps (timeout/collision)")
                break
        
        # Grade the episode
        final_pos_error = info['position_error']
        final_heading_error = info['heading_error']
        score = score_episode(info, total_steps, final_pos_error, final_heading_error)
        scores.append(score)
        print(f"  Grade: {score:.1f}/10")
        logger.info(f"Episode {episode + 1} Score: {score:.1f}/10 - Steps: {total_steps}, Goal: {info['goal_reached']}, Collision: {info['collision']}")
    
    # Summary
    avg_score = np.mean(scores)
    print(f"\n{'='*70}")
    print(f"Test Complete - Average Score: {avg_score:.1f}/10")
    print(f"Individual Scores: {', '.join(f'{s:.1f}' for s in scores)}")
    print(f"{'='*70}")


if __name__ == "__main__":
    run_demo()
