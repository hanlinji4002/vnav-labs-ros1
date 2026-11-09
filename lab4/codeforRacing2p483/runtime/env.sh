#!/usr/bin/env bash
# Source only into this run's shell; never changes ~/.bashrc or system settings.
set -eo pipefail
lab_gpu_runtime_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export LAB4_GPU_ROOT="${LAB4_GPU_ROOT:-$(cd -- "$lab_gpu_runtime_dir/.." && pwd -P)}"
[[ -f /opt/ros/noetic/setup.bash ]] || { echo 'ROS Noetic is not installed.' >&2; return 2 2>/dev/null || exit 2; }
[[ -f "$LAB4_GPU_ROOT/ros_ws/devel/setup.bash" ]] || { echo 'Build the portable ros_ws first.' >&2; return 2 2>/dev/null || exit 2; }
if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" != 0 && "${LIBGL_ALWAYS_SOFTWARE:-}" != false ]]; then
  echo 'Refusing LIBGL_ALWAYS_SOFTWARE; hardware rendering is required.' >&2
  return 2 2>/dev/null || exit 2
fi
# Avoid carrying another ROS workspace or Python virtual environment into a run.
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH ROSLISP_PACKAGE_DIRECTORIES PYTHONPATH
unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME ROS_HOME VIRTUAL_ENV CONDA_PREFIX PYTHONHOME
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
source /opt/ros/noetic/setup.bash
source "$LAB4_GPU_ROOT/ros_ws/devel/setup.bash"
export PYTHONPATH="$LAB4_GPU_ROOT/ros_ws/devel/lib/python3/dist-packages:$LAB4_GPU_ROOT/ros_ws/src/tesse-ros-bridge/tesse-interface/src:/opt/ros/noetic/lib/python3/dist-packages"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export LAB4_GPU_ENV_READY="$LAB4_GPU_ROOT"
# DISPLAY/WSLg sockets and Mesa configuration are inherited, not replaced.
