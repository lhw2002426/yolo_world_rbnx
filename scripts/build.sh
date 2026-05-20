#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase. Two steps:
#   1. colcon build vendored graspnet_msgs so the rclpy background
#      thread can `from graspnet_msgs.srv import ObjectDetectionRequest`
#      (legacy ROS service surface) and `from graspnet_msgs.msg import
#      DetectedObject(s)` (legacy publisher surface).
#   2. rbnx codegen --mcp so:
#        * atlas_pb2 / atlas_pb2_grpc Python stubs exist (the Service.run()
#          path uses these).
#        * perception_mcp.py is generated from
#          capabilities/lib/perception/srv/DetectObject.srv with
#          DetectObject_Request / DetectObject_Response dataclasses.
#          (This is what main.py imports at the bottom.)
#
# Note: we do NOT vendor or build ultralytics / YOLOE — those come
# from `pip install ultralytics` at deploy time. Adding a system
# requirement instead of vendoring ~1GB of pytorch is the deliberate
# tradeoff (same as `apt install ros-humble-nav2-bringup` for nav2).
#
# Output layout:
#   rbnx-build/codegen/proto_gen/                atlas stubs
#   rbnx-build/codegen/robonix_mcp_types/        perception_mcp.py
#   rbnx-build/ws/install/graspnet_msgs/         msg/srv Python bindings
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[yolo_world/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Symlink vendored graspnet_msgs into colcon ws.
ln -snf "$PKG/src/graspnet_msgs" "$PKG/rbnx-build/ws/src/graspnet_msgs"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[yolo_world/build] colcon build (graspnet_msgs)"
cd "$PKG/rbnx-build/ws"
colcon build --symlink-install \
    --packages-select graspnet_msgs \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

# Sanity: warn if ultralytics isn't pip-installed yet. Don't fail the
# build (CI machines without GPU will skip pip install ultralytics).
if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "[yolo_world/build] NOTE: ultralytics not importable. Install with:"
    echo "                       pip install ultralytics"
    echo "                     (deploy will fail at on_init without it)."
fi

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[yolo_world/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[yolo_world/build] done."
