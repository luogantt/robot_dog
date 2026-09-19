#!/usr/bin/env bash
set -euo pipefail

route_file="${1:-/home/lg/.project/S10_final_delivery/data/routes/final_route_v01.yaml}"
localization_config="${2:-/home/lg/.project/S10_final_delivery/src/s10_nav_bringup/config/s10_lightning_loc.yaml}"

failures=0
for command_name in ros2 python3; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "FAIL: missing command ${command_name}"
    failures=$((failures + 1))
  fi
done

for required_file in "${route_file}" "${localization_config}"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "FAIL: file is not readable: ${required_file}"
    failures=$((failures + 1))
  else
    echo "OK: ${required_file}"
  fi
done

if [[ -r "${localization_config}" ]]; then
  map_path="$(python3 - "${localization_config}" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    print((yaml.safe_load(stream) or {}).get('system', {}).get('map_path', ''))
PY
)"
  if [[ -z "${map_path}" || "${map_path}" != /* || ! -d "${map_path}" ]]; then
    echo "FAIL: Lightning map_path must be an existing absolute directory: ${map_path}"
    failures=$((failures + 1))
  else
    echo "OK: map ${map_path}"
  fi
fi

if command -v ros2 >/dev/null 2>&1; then
  for package_name in drdds rslidar_sdk lightning s10_sensor_adapter s10_route_nav s10_nav_bringup; do
    if ! ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
      echo "FAIL: ROS package not available: ${package_name}"
      failures=$((failures + 1))
    else
      echo "OK: package ${package_name}"
    fi
  done
fi

if [[ "${failures}" -ne 0 ]]; then
  echo "Preflight failed with ${failures} problem(s)."
  exit 1
fi

echo "Static preflight passed. Start the stack, then wait for /s10_nav/healthy=true before enabling."
