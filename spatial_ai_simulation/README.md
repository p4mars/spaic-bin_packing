# spatial_ai_simulation

Gazebo world package for the Spatial AI station.

## Usage

Build and source the workspace:

```bash
colcon build --packages-select spatial_ai_simulation
source install/setup.bash
```

Launch the standalone world:

```bash
ros2 launch spatial_ai_simulation spatial_ai_simulation.launch.py
```

Launch the world with a MIRTE Master robot:

```bash
ros2 launch spatial_ai_simulation spatial_ai_mirte_master.launch.py
```

The standalone world is self-contained and uses the STL models installed from this package's `gazebo_models` directory.
