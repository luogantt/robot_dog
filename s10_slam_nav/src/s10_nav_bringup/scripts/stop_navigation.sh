#!/usr/bin/env bash
set -euo pipefail

# Fail closed first. The terminal running ros2 launch can then be stopped with Ctrl-C.
ros2 service call /s10_nav/emergency_stop std_srvs/srv/Trigger '{}'
echo "Emergency stop latched. Stop the launch process with Ctrl-C."
