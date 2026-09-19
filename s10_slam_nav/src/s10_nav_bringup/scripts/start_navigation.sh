#!/usr/bin/env bash
set -euo pipefail

driver_config="${DRIVER_CONFIG:-/home/lg/.project/S10_final_delivery/src/s10_nav_bringup/config/airy_dual.yaml}"
lightning_config="${LIGHTNING_CONFIG:-/home/lg/.project/S10_final_delivery/src/s10_nav_bringup/config/s10_lightning_loc.yaml}"
route_file="${ROUTE_FILE:-/home/lg/.project/S10_final_delivery/data/routes/final_route_v01.yaml}"

exec ros2 launch s10_nav_bringup navigation.launch.py \
  driver_config:="${driver_config}" \
  lightning_config:="${lightning_config}" \
  route_file:="${route_file}" \
  start_driver:="${START_DRIVER:-true}"
