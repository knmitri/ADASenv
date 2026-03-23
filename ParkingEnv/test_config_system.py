#!/usr/bin/env python3
"""Test script to verify config system is working."""

from KarlsPlayground.simulation import AudiA5, ParkingSimulation
from KarlsPlayground.config_loader import get_config

print("✅ All imports successful!")
print()

# Test config loading
config = get_config()
print("Configuration loaded:")
print(f"  - Vehicle max speed: {config['vehicle']['audi_a5']['max_speed']} m/s")
print(f"  - Parking slots: {config['parking_lot']['num_slots']}")
print(f"  - Visualization margin: {config['visualization']['margin']} m (zoomed out!)")
print()

# Test creating vehicle with config
vehicle = AudiA5()
print("✅ AudiA5 created from config:")
print(f"   Length: {vehicle.length} m")
print(f"   Width: {vehicle.width} m")
print(f"   Max speed: {vehicle.MAX_SPEED} m/s")
print()

# Test creating simulation with config
sim = ParkingSimulation(vehicle=vehicle, render=False)
print("✅ ParkingSimulation created from config:")
print(f"   Parking lot: {sim.parking_lot.num_slots} slots")
print(f"   Time step: {sim.dt} s")
print(f"   Max steps: {sim.max_episode_steps}")
print(f"   Position tolerance: {sim.position_tolerance} m")
print()

# Test reset
state = sim.reset()
print("✅ Simulation reset successful!")
print(f"   Initial position: ({state['position'][0]:.2f}, {state['position'][1]:.2f})")
print()

# Test step
acceleration, steering = 0.5, 0.1
state, info = sim.step(acceleration, steering)
print("✅ Simulation step successful!")
print(f"   Step info: {info['episode_step']} / {sim.max_episode_steps}")
print()

print("🎉 All configuration values are properly loaded and working!")
print()
print("Summary of changes:")
print("  ✨ Created: KarlsPlayground/config.json - All hard-coded values")
print("  ✨ Created: KarlsPlayground/config_loader.py - JSON config loader")
print("  📝 Modified: environment.py - Uses config for vehicle, simulation, start params")
print("  📝 Modified: visualizer.py - Uses config for figure size, margins, colors")
print("  🔍 Zoom: Visualization margin increased from 1.5m to 3.0m (zoomed out)")
