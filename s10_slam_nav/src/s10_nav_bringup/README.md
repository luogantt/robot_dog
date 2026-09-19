# S10 final-round navigation stack

This directory contains the competition-focused mapping, localization, route-following,
obstacle and safety bringup. It is fail-closed: launching does not enable motion.

## Required source packages

- `drdds`
- patched `rslidar_sdk` built as `XYZIRT` with transform and IMU parsing enabled
- patched `lightning-lm-deep-robotics`
- `s10_sensor_adapter`
- `s10_route_nav`
- `s10_nav_bringup`

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/nvidia/goai_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Before building, make sure the RoboSense and Lightning sources used by the workspace
contain the patches documented in the repository-level `UPSTREAM_VERSIONS.md`.

## Safety sequence

1. Keep the robot supported or in an open low-speed test area.
2. Start `rl_deploy`, stand the robot and enter RL mode with the gamepad.
3. Start the navigation launch. `/STEER` remains zero because autonomy is disabled.
4. Confirm `/s10_nav/healthy` is continuously `true`.
5. Enable motion explicitly:

```bash
ros2 service call /s10_nav/set_enabled std_srvs/srv/SetBool '{data: true}'
```

6. Stop and latch the output at any time:

```bash
ros2 service call /s10_nav/emergency_stop std_srvs/srv/Trigger '{}'
```

If health is lost, the mux disables autonomy. Recovery always requires an explicit reset
and re-enable; motion never resumes just because a topic starts publishing again.

## Mapping

Edit `config/s10_lightning_slam.yaml` so `system.map_path` is an absolute writable path.
Then run:

```bash
ros2 launch s10_nav_bringup mapping.launch.py
```

Drive manually, close the loop, save the map, stop mapping and restart in localization
mode before recording the final route.

## Route recording

```bash
ros2 launch s10_nav_bringup record_route.launch.py \
  output_file:=/home/nvidia/goai_data/routes/final_route_v01.yaml
ros2 service call /s10_route_recorder/set_recording std_srvs/srv/SetBool '{data: true}'
ros2 service call /s10_route_recorder/mark_checkpoint std_srvs/srv/Trigger '{}'
ros2 service call /s10_route_recorder/set_recording std_srvs/srv/SetBool '{data: false}'
ros2 service call /s10_route_recorder/save std_srvs/srv/Trigger '{}'
```

Mark each scoring point while the robot is physically inside its accepted region.

## Navigation

```bash
ros2 launch s10_nav_bringup navigation.launch.py \
  route_file:=/home/nvidia/goai_data/routes/final_route_v01.yaml
```

The installed `template_route.yaml` is only a low-speed interface test. Never use it as
the competition route.
