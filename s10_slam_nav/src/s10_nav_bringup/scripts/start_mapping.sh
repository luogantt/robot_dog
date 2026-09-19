#!/usr/bin/env bash
set -euo pipefail

driver_config="${DRIVER_CONFIG:-/home/lg/.project/S10_final_delivery/src/s10_nav_bringup/config/airy_dual.yaml}"
lightning_config="${LIGHTNING_CONFIG:-/home/lg/.project/S10_final_delivery/src/s10_nav_bringup/config/s10_lightning_slam.yaml}"

exec ros2 launch s10_nav_bringup mapping.launch.py \
  driver_config:="${driver_config}" \
  lightning_config:="${lightning_config}" \
  start_driver:="${START_DRIVER:-true}"
