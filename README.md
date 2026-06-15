# pairs_vins_kickoff_estimator_plugin

A state-estimator plugin for the PAIRS UAV stack that bridges the gap between takeoff and visual-inertial odometry. When a VINS/VIO estimator (for example OpenVINS) has not yet converged, this estimator "kicks off" the flight so the UAV can take off and hover, then automatically switches the estimation manager over to the real VINS estimator once it is initialized. It plugs into `pairs_uav_managers` through pluginlib and is loaded by the estimation manager alongside the other state estimators.

## Contents

- pluginlib plugin `vins_kickoff/VinsKickoffEstimatorPlugin` (`vins_kickoff::VinsKickoff`), a `pairs_uav_managers::StateEstimator` exported via `estimator_plugins.xml`
- library `PairsUavStateEstimators_VinsKickoff`
- ready-to-run tmux sessions under `tmux/` for a Gazebo simulation and for a real-world `bluefox_front` deployment

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)

```bash
sudo apt install ros-noetic-pairs-vins-kickoff-estimator-plugin
```

## Usage

The plugin is loaded automatically by the estimation manager; it is not launched on its own. To try it end to end, run one of the bundled tmux sessions:

```bash
# Gazebo simulation
cd tmux/simulation && ./simulation.sh

# Real-world bluefox_front deployment
cd tmux/realworld/bluefox_front && ./tmux.sh
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_vins_kickoff_estimator_plugin` package; the original
copyright is retained in [LICENSE](LICENSE).
