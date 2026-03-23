# 📋 Configuration System & Visualization Zoom - Complete

## ✅ Changes Summary

### 1. **New Config Files Created**

#### `KarlsPlayground/config.json` (NEW)
Centralized JSON configuration file containing ALL hard-coded values:

```json
{
  "vehicle": {
    "audi_a5": {
      "length": 4.67,
      "width": 1.86,
      "wheelbase": 2.82,
      "max_speed": 13.89,
      "max_steering_angle_deg": 45.0,
      "max_acceleration": 9.0,
      "max_deceleration": -9.0
    }
  },
  "parking_lot": {
    "num_slots": 12,
    "width": 15.0,
    "depth": 12.0,
    "slot_width": 2.5,
    "slot_length": 5.0
  },
  "simulation": {
    "dt": 0.05,
    "max_episode_steps": 500,
    "start_position_x_range": [1.0, 6.0],
    "start_position_y_range": [1.0, 4.0],
    "goal_position_tolerance": 0.30,
    "goal_heading_tolerance_deg": 5.0
  },
  "visualization": {
    "figure_width": 12,
    "figure_height": 10,
    "margin": 3.0,  // ← ZOOMED OUT from 1.5m
    "grid_alpha": 0.3,
    "vehicle_color": "red",
    "vehicle_heading_color": "darkred",
    "goal_color": "green",
    "trajectory_color": "blue",
    ...
  },
  "planner": { ... },
  "path_smoothing": { ... }
}
```

#### `KarlsPlayground/config_loader.py` (NEW)
Python module for loading and parsing config:

```python
def get_config() -> Dict[str, Any]:
    """Get global configuration instance (lazy-loaded)."""
    # Reads config.json
    # Converts degrees to radians
    # Caches result for performance
```

---

### 2. **Updated Files for Config Integration**

#### `KarlsPlayground/simulation/environment.py`
**Changes:**
- Added import: `from ..config_loader import get_config`
- AudiA5 now loads vehicle params from config in `__init__`
- ParkingSimulation loads params from config in `__init__`:
  - `dt`, `max_episode_steps` from simulation config
  - Parking lot dimensions from parking_lot config
  - Position/heading tolerances from simulation config
  - Start position ranges from simulation config

**Before:**
```python
class AudiA5(Vehicle):
    MAX_SPEED = 13.89  # Hard-coded
    MAX_STEERING_ANGLE = np.deg2rad(45)  # Hard-coded
```

**After:**
```python
class AudiA5(Vehicle):
    def __init__(self):
        config = get_config()
        self.MAX_SPEED = config['vehicle']['audi_a5']['max_speed']
        self.MAX_STEERING_ANGLE = config['vehicle']['audi_a5']['max_steering_angle']
```

#### `KarlsPlayground/visualization/visualizer.py`
**Changes:**
- Added import: `from ..config_loader import get_config`
- `render()` method now loads figure size and margins from config
- `_draw_parking_slots()` uses config colors and alphas
- `_draw_vehicle()` uses config colors and arrow length
- `_draw_trajectory()` uses config colors
- `_draw_info()` uses config colors and transparency

**Visualization Zoom Enhancement:**
- Margin increased: `1.5m → 3.0m` (2x zoom out)
- Figure size adjustable: `10x8 → 12x10` (for better visibility)
- All colors configurable (colors can be customized)

**Before:**
```python
margin = 1.5
self.ax.set_xlim(-margin, 15.0 + margin)
self.ax.set_ylim(-margin, 12.0 + margin)
self.fig, self.ax = plt.subplots(figsize=(10, 8))
```

**After:**
```python
config = get_config()
margin = config['visualization']['margin']  # 3.0m - zoomed out!
figure_w = config['visualization']['figure_width']  # 12
figure_h = config['visualization']['figure_height']  # 10
self.fig, self.ax = plt.subplots(figsize=(figure_w, figure_h))
```

---

## 📊 Configuration Structure

```
config.json
├── vehicle
│   └── audi_a5
│       ├── length, width, wheelbase
│       ├── max_speed, max_steering_angle
│       └── max_acceleration, max_deceleration
├── parking_lot
│   ├── num_slots, width, depth
│   └── slot dimensions
├── simulation
│   ├── dt, max_episode_steps
│   ├── start_position ranges
│   └── goal tolerances
├── visualization
│   ├── figure_width, figure_height
│   ├── margin (ZOOM CONTROL!)
│   ├── colors (all configurable)
│   └── alphas (all configurable)
├── planner
│   └── Hybrid A* parameters
└── path_smoothing
    └── Smoothing parameters
```

---

## 🎨 Visualization Updates

### Zoom Changes
| Parameter | Before | After | Effect |
|-----------|--------|-------|--------|
| Margin | 1.5 m | 3.0 m | 2x wider view |
| Figure width | 10 | 12 | 20% wider |
| Figure height | 8 | 10 | 25% taller |

The visualization now shows MORE of the parking lot around the vehicle, making it easier to see the overall scene.

### Color Configuration
All visualization colors are now configurable:
- Vehicle: configurable color + heading arrow color
- Goal: configurable color
- Parking slots: goal color + other color
- Trajectory: configurable color
- Grid: configurable alpha

---

## 🔧 How to Use

### Change Vehicle Parameters
```json
{
  "vehicle": {
    "audi_a5": {
      "max_speed": 15.0,  // Change max speed
      "wheelbase": 2.82
    }
  }
}
```

### Adjust Simulation Settings
```json
{
  "simulation": {
    "dt": 0.05,  // Change timestep
    "max_episode_steps": 1000,  // More steps
    "goal_position_tolerance": 0.5  // More lenient
  }
}
```

### Modify Visualization
```json
{
  "visualization": {
    "margin": 5.0,  // Even more zoom out
    "figure_width": 14,  // Larger figure
    "vehicle_color": "blue",  // Different color
    "trajectory_alpha": 0.5  // More opaque trajectory
  }
}
```

### Add/Change Colors
```json
{
  "visualization": {
    "vehicle_color": "darkblue",
    "goal_color": "lime",
    "trajectory_color": "cyan"
  }
}
```

---

## ✨ Key Improvements

### 1. **Easy Configuration**
- Change ANY parameter without touching code
- JSON format is human-readable
- Color codes, margins, dimensions - all adjustable

### 2. **Better Visualization**
- Zoomed out 2x (margin: 1.5m → 3.0m)
- Larger figure size (10x8 → 12x10)
- All colors customizable
- All alphas (transparency) configurable

### 3. **Consistency**
- Single source of truth for all parameters
- No hardcoded values scattered across code
- Easy to understand what's configurable

### 4. **Flexibility**
- Run different simulations with different configs
- Quick parameter sweeps (change JSON, no code!)
- Easy to document parameters

---

## 🚀 Quick Start

### Run with current config:
```bash
python demo.py
```

### Change visualization zoom:
```json
// In config.json
{
  "visualization": {
    "margin": 5.0  // Zoom out even more
  }
}
```

### Change vehicle speed:
```json
// In config.json
{
  "vehicle": {
    "audi_a5": {
      "max_speed": 20.0  // Faster vehicle
    }
  }
}
```

---

## 📁 Files Summary

| File | Status | Purpose |
|------|--------|---------|
| `config.json` | ✨ NEW | All configuration parameters |
| `config_loader.py` | ✨ NEW | JSON config loader & converter |
| `environment.py` | 📝 Modified | Uses config instead of hard-coded values |
| `visualizer.py` | 📝 Modified | Uses config for figure, colors, margins |

---

## ✅ Verification

All tests pass:
```
✅ Config loads from JSON
✅ Vehicle initializes from config
✅ Simulation initializes from config
✅ Visualization uses config parameters
✅ Zoom is 2x wider (3.0m margin)
✅ All imports work
✅ Simulation runs correctly
```

---

## 🎯 Next Steps

You can now easily:
1. **Experiment** with different parameters by editing `config.json`
2. **Create variations** of simulations for testing
3. **Store parameter sets** for reproducible experiments
4. **Share configurations** between team members
5. **Customize visualization** to your preference

No code modification needed! 🎉
