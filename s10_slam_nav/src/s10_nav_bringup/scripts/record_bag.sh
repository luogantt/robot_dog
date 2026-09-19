#!/usr/bin/env bash
set -euo pipefail

bag_root="${BAG_ROOT:-/home/lg/.project/S10_final_delivery/data/bags}"
mkdir -p "${bag_root}"
bag_name="s10_$(date +%Y%m%d_%H%M%S)"

exec ros2 bag record -o "${bag_root}/${bag_name}" \
  /rslidar_front/points \
  /rslidar_front/imu \
  /rslidar_rear/points \
  /rslidar_rear/imu \
  /IMU \
  /lightning/odom \
  /lightning/nav_state \
  /s10_nav/cmd_tracking \
  /s10_nav/cmd_avoidance \
  /s10_nav/obstacle_state \
  /s10_nav/healthy \
  /s10_nav/state \
  /s10_nav/checkpoint_event \
  /STEER
