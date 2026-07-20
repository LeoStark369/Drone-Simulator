# 🚁 Autonomous Drone Simulator

A modular Python framework for simulating **autonomous drone flight** — global path planning, reactive obstacle avoidance, an autonomous landing state machine, and live 3D sensor visualization — built with a clean hardware-abstraction layer so the same control stack can be pointed at a real MAVLink-based flight controller.

![Demo mission](assets/demo_mission.png)
*Random obstacle field, A\* global path (blue dashed), flown trajectory after reactive avoidance (green), and live LIDAR returns (orange).*

---

## Features

| Capability | How it works |
|---|---|
| 🧭 **Path planning** | 3D A\* search over a voxelized occupancy grid, with greedy line-of-sight path smoothing to remove unnecessary waypoints |
| 🛑 **Obstacle avoidance** | Reactive artificial potential field (APF) blending attraction to the next waypoint with repulsion from live LIDAR returns; automatically triggers a global replan when the forward path is blocked |
| 🎯 **Autonomous landing** | A staged state machine (`APPROACH → VERIFY → DESCEND → TOUCHDOWN`) that checks ground clearance before and during descent, aborting back to a hold if the landing zone becomes obstructed |
| 📡 **Sensor simulation** | Multi-layer rotating LIDAR model (ray-cast against sphere/box obstacles) with configurable range, resolution, vertical field of view, and gaussian range noise |
| 📊 **Visualization** | Live matplotlib 3D view of obstacles, planned path, flown trajectory, and current sensor returns |
| 🔌 **Real-world ready** | All algorithmic modules talk to a `DroneInterface` abstract class, not a concrete simulated drone — implement it against `pymavlink`/`DroneKit` to drive real hardware (see `examples/real_hardware_stub.py`) |

---

## Project structure

```
drone-simulator/
├── drone_sim/
│   ├── config.py               # all tunables in one place (speed, sensor range, PID-ish gains...)
│   ├── environment.py          # Obstacle geometry (sphere/box) + Environment collision queries
│   ├── sensors.py              # LidarSensor: multi-layer ray-cast scan model
│   ├── path_planner.py         # AStarPlanner: 3D grid A* + line-of-sight path smoothing
│   ├── obstacle_avoidance.py   # PotentialFieldAvoider: reactive local avoidance
│   ├── drone.py                # DroneState, DroneInterface (ABC), simulated Drone (point-mass model)
│   ├── landing.py               # AutoLander: staged autonomous landing state machine
│   ├── visualizer.py            # SimulatorVisualizer: live 3D matplotlib rendering
│   └── simulator.py             # Simulator: orchestrates sense → decide → act → land each tick
├── examples/
│   └── real_hardware_stub.py    # Template for wiring a real MAVLink vehicle into the same stack
├── tests/
│   ├── test_path_planner.py
│   ├── test_sensors.py
│   └── test_mission.py
├── main.py                      # CLI entry point
├── requirements.txt
├── setup.py
└── LICENSE
```

---

## Installation

```bash
git clone https://github.com/<your-username>/drone-simulator.git
cd drone-simulator
pip install -r requirements.txt
```

Requires Python 3.9+.

---

## Quick start

Run a full mission headlessly and print a summary:

```bash
python main.py --seed 42
```

```
==================================================
Mission success : True
Reason          : landed
Ticks executed  : 284
Trajectory pts  : 284
==================================================
```

Run with a live 3D visualization window:

```bash
python main.py --seed 42 --visualize
```

Run with a custom start/goal and obstacle density, saving the final frame:

```bash
python main.py --seed 7 --num-obstacles 25 \
    --start 5 5 8 --goal 90 90 0 \
    --visualize --save-fig mission_result.png
```

### Using it as a library

```python
from drone_sim import Environment, Simulator

env = Environment.random_scene(num_obstacles=20, seed=1)
sim = Simulator(env, seed=1)

result = sim.run_mission(start=(2, 2, 6), goal=(80, 80, 0))

print(result.success, result.reason, result.ticks)
```

---

## How the control loop works

Each simulation tick (`Simulator.run_tick`) does:

1. **Sense** — cast a full LIDAR scan (`LidarSensor.scan`) from the current pose against the `Environment`.
2. **Decide**
   - If the forward cone is blocked, or the drone has drifted far from the global path, trigger a fresh **A\*** replan.
   - Otherwise, compute a **reactive avoidance velocity**: pulled toward the next waypoint, pushed away from any obstacle inside the influence radius, with a hard-safety override if anything is inside the minimum safe distance.
3. **Act** — command that velocity to the `DroneInterface`, integrate the (bounded-acceleration) vehicle dynamics one timestep.
4. **Land** — once within range of the goal, control hands off to `AutoLander`, which holds, verifies ground clearance, descends, and touches down — aborting back to a hold if the pad becomes obstructed mid-descent.

This two-layer design (deliberate global planning + fast reactive avoidance) mirrors how real autonomy stacks are structured: a global planner that reasons about the whole map, and a fast local controller that reacts to what the sensors see *right now*, since the global map is never perfectly up to date.

---

## Taking this to real hardware

Every algorithmic module (`path_planner.py`, `obstacle_avoidance.py`, `landing.py`, `simulator.py`) depends only on:

- `Environment` — build this from live sensor data (e.g. cluster LIDAR/depth-camera point clouds into `Obstacle` spheres/boxes) instead of `Environment.random_scene(...)`.
- `DroneInterface` — implement `get_state`, `set_velocity_command`, `step`, and `land` against your flight controller's SDK (e.g. `pymavlink`).

See [`examples/real_hardware_stub.py`](examples/real_hardware_stub.py) for a `MavlinkDrone` skeleton. **Keep hard safety layers — geofencing, battery failsafes, RC override, obstacle-stop — in your autopilot firmware.** This library is a planning/avoidance/landing *algorithm* layer; it is not a certified flight-safety system, and any real-world flight should retain independent hardware safeguards.

---

## Running the tests

```bash
pip install pytest
pytest tests/ -v
```

The test suite covers:
- A\* planning correctness (finds direct routes, routes around obstacles, correctly reports failure when the goal is enclosed)
- LIDAR ray-casting accuracy against known geometry
- End-to-end missions: the drone lands successfully, and the flown trajectory never violates the minimum obstacle clearance

---

## Configuration

All tunables (max speed, sensor range/noise, avoidance gains, landing speeds, safety margins) live in `drone_sim/config.py` as a single `SimConfig` dataclass — override individual fields to match a real airframe's limits:

```python
from drone_sim.config import SimConfig

config = SimConfig(max_speed=4.0, lidar_max_range=25.0, min_safe_distance=2.0)
```

---

## Roadmap

- [ ] Swap point-mass dynamics for a full quadrotor model (thrust/torque, attitude control)
- [ ] Dynamic (moving) obstacle support in the LIDAR + avoidance layers
- [ ] RRT\* planner option for very large / high-resolution environments
- [ ] ROS 2 bridge for direct integration with real robotics stacks

---

## License

MIT — see [LICENSE](LICENSE).

## Author

**Surafel Gashaw (Leo Stark)**
