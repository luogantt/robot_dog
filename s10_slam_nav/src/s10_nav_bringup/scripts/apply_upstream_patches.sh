#!/usr/bin/env bash
set -euo pipefail

workspace_root="${WORKSPACE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
lightning_dir="${LIGHTNING_DIR:-${workspace_root}/src/lightning}"
rslidar_dir="${RSLIDAR_DIR:-${workspace_root}/src/rslidar_sdk}"

git -C "${rslidar_dir}" apply --check "${workspace_root}/patches/rslidar_s10.patch"
git -C "${lightning_dir}" apply --check "${workspace_root}/patches/lightning_s10.patch"
git -C "${rslidar_dir}" apply "${workspace_root}/patches/rslidar_s10.patch"
git -C "${lightning_dir}" apply "${workspace_root}/patches/lightning_s10.patch"

echo "S10 upstream patches applied. Build from a clean build directory."
