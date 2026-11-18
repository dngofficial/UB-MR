#!/usr/bin/env bash
set -euo pipefail
lincoln_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
lincoln_python="${LINCOLN_PYTHON:-python3}"
export PYTHONPATH="$lincoln_root/ros2_ws/src/rotation_integration${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
case "${1:-offline}" in
  offline)
    "$lincoln_python" "$lincoln_root/tools/audit_unity_scene.py"
    exec "$lincoln_python" -m unittest discover -s "$lincoln_root/ros2_ws/src/rotation_integration/test" -v
    ;;
  ros)
    export RUN_ROS_INTEGRATION=1
    export ROS_DOMAIN_ID="${LINCOLN_TEST_DOMAIN_ID:-83}"
    export ROS_LOG_DIR="${ROS_LOG_DIR:-$lincoln_root/ros2_ws/log/ros_checks}"
    exec "$lincoln_python" -m unittest discover -s "$lincoln_root/ros2_ws/src/rotation_integration/test" -p 'test_ros*.py' -v
    ;;
  build)
    cd "$lincoln_root/ros2_ws"
    exec "$lincoln_python" -m colcon build --symlink-install --packages-select rotation_integration
    ;;
  *)
    echo 'Usage: bash tools/check.sh [offline|ros|build]; optional LINCOLN_PYTHON=/path/to/python' >&2
    exit 2
    ;;
esac
