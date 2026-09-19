# S10 upstream versions and required competition patches

## Pinned sources

| Repository | Version checked during design |
|---|---|
| `DeepRoboticsLab/lightning-lm-deep-robotics` | `31ce5190bb2dc9c8be99a572fe2413380ad60cd6` |
| `RoboSense-LiDAR/rslidar_sdk` | tag `v1.5.19`, local commit beginning `78d2abb` |
| `RoboSense-LiDAR/rslidar_msg` | `fe8a95cb242bd294cc3d5e3422f2093fb49a56ee` |
| `DeepRoboticsLab/deep-robotics-msg` | repository snapshot downloaded with the project |

## Required RoboSense build settings

The stock SDK defaults are not suitable for Lightning's RoboSense point handler.

```cmake
set(POINT_TYPE "XYZIRT" CACHE STRING "Point type")
option(ENABLE_TRANSFORM "Enable transform functions" ON)
option(ENABLE_IMU_DATA_PARSE "Enable imu data parse" ON)
```

Verify the runtime `PointCloud2` schema is exactly `x/y/z/intensity FLOAT32`, `ring
UINT16`, and `timestamp FLOAT64`.

## Required Lightning changes

1. Guard empty RoboSense point clouds before reading `points[0]`.
2. Use the options loaded inside `LocSystem::Init()` when selecting the configured
   initial pose.
3. Set `system.use_imu_orient: false` for Airy IMU messages.
4. Remove or conditionalize unavailable, unused dependencies such as
   `scrubber_common` and `agibot_robot`.
5. Build and run headless on the AGX (`system.with_ui: false`).

The delivery copies under `src/lightning` and `src/rslidar_sdk` already contain these
patches. When creating a fresh workspace with `s10_final.repos`, reapply the patches
before building.
