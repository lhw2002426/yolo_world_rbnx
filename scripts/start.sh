#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Start phase. Source ROS + colcon overlay + codegen PYTHONPATH, then
# exec the python module.
#
# See yolo_grasp_rbnx/scripts/start.sh for the rationale of the
# fallback overlay chain. Same trick here: if the vendored
# graspnet_msgs colcon-build didn't produce importable bindings,
# fall back to the operator's pre-existing graspnet workspace
# (env YOLO_WORLD_EXTRA_OVERLAYS or auto-discover).
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u
else
    echo "[yolo_world/start] ERR: colcon overlay missing — run scripts/build.sh" >&2
    exit 2
fi

# ── Direct PYTHONPATH / AMENT_PREFIX_PATH / LD_LIBRARY_PATH injection ──
# Why: when this start.sh runs inside an outer shell that already has
# ANOTHER colcon overlay sourced (e.g. operator's ~/.bashrc sources
# /home/.../tracing_ws/install/setup.bash), colcon's idempotent prefix
# markers can cause our `source $PKG/rbnx-build/ws/install/setup.bash`
# above to silently NOT add our overlay's paths to AMENT_PREFIX_PATH /
# PYTHONPATH / LD_LIBRARY_PATH. Symptoms in this order as you tighten
# the fix:
#
#   1. `import graspnet_msgs.srv` fails        ← needs PYTHONPATH
#   2. import OK but rclpy.create_publisher    ← needs LD_LIBRARY_PATH
#      raises "Could not load library
#      libgraspnet_msgs__rosidl_typesupport_introspection_c.so"
#      → rclpy thread CRASHES, RPC stays alive,
#        package looks ACTIVE but produces nothing
#   3. [optional] ament_index lookups fail     ← needs AMENT_PREFIX_PATH
#
# We inject all three from the `graspnet_msgs` install tree directly,
# bypassing colcon's idempotent guard. Idempotent — if colcon DID
# source correctly, the paths are merely duplicated, not corrupted.
GMSGS_PREFIX="$PKG/rbnx-build/ws/install/graspnet_msgs"
if [[ -d "$GMSGS_PREFIX" ]]; then
    # 1. AMENT_PREFIX_PATH — for share/ament_index resource lookups.
    case ":${AMENT_PREFIX_PATH:-}:" in
        *":${GMSGS_PREFIX}:"*) ;;
        *) export AMENT_PREFIX_PATH="${GMSGS_PREFIX}:${AMENT_PREFIX_PATH:-}" ;;
    esac
    # 2. PYTHONPATH — for `import graspnet_msgs.{msg,srv}`. Find the
    # actual python site-packages dir colcon emitted (Ubuntu uses
    # `local/lib/python3.X/dist-packages`; other layouts also work).
    for _site in \
        "$GMSGS_PREFIX"/local/lib/python*/dist-packages \
        "$GMSGS_PREFIX"/lib/python*/site-packages \
        "$GMSGS_PREFIX"/lib/python*/dist-packages
    do
        if [[ -d "$_site" ]]; then
            case ":${PYTHONPATH:-}:" in
                *":${_site}:"*) ;;
                *) export PYTHONPATH="${_site}:${PYTHONPATH:-}" ;;
            esac
        fi
    done
    unset _site
    # 3. LD_LIBRARY_PATH — rclpy dlopen()s typesupport .so files
    # (`libgraspnet_msgs__rosidl_typesupport_*.so`) at create_publisher /
    # create_subscription / create_service time. They live next to the
    # other ROS .so files in the install lib/ directory.
    for _libdir in \
        "$GMSGS_PREFIX"/lib \
        "$GMSGS_PREFIX"/local/lib
    do
        if [[ -d "$_libdir" ]]; then
            case ":${LD_LIBRARY_PATH:-}:" in
                *":${_libdir}:"*) ;;
                *) export LD_LIBRARY_PATH="${_libdir}:${LD_LIBRARY_PATH:-}" ;;
            esac
        fi
    done
    unset _libdir
fi
unset GMSGS_PREFIX

_source_overlay() {
    local f="$1"
    if [[ -f "$f" ]]; then
        echo "[yolo_world/start] sourcing extra overlay: $f" >&2
        # shellcheck disable=SC1090
        set +u; source "$f"; set -u
        return 0
    fi
    return 1
}

if [[ -n "${YOLO_WORLD_EXTRA_OVERLAYS:-}" ]]; then
    IFS=':' read -ra _extras <<< "$YOLO_WORLD_EXTRA_OVERLAYS"
    for f in "${_extras[@]}"; do _source_overlay "$f" || true; done
fi

if ! python3 -c "import graspnet_msgs.srv" 2>/dev/null; then
    echo "[yolo_world/start] WARN: graspnet_msgs not importable from \
own overlay — trying fallback paths" >&2
    for f in \
        "$HOME/lhw/grasp/driver/graspnet/install/setup.bash" \
        "$HOME/grasp/driver/graspnet/install/setup.bash" \
        "/home/syswonder/lhw/grasp/driver/graspnet/install/setup.bash"
    do
        _source_overlay "$f" && break || true
    done
fi

if ! python3 -c "import graspnet_msgs.srv" 2>&1 >/dev/null; then
    echo "[yolo_world/start] FATAL: cannot import graspnet_msgs.srv" >&2
    echo "[yolo_world/start] AMENT_PREFIX_PATH:" >&2
    printf '  %s\n' ${AMENT_PREFIX_PATH//:/ } >&2
    echo "[yolo_world/start] PYTHONPATH:" >&2
    printf '  %s\n' ${PYTHONPATH//:/ } >&2
    echo "[yolo_world/start] vendored install tree:" >&2
    find "$PKG/rbnx-build/ws/install/graspnet_msgs" -name '*.py' 2>&1 | head -10 >&2 || true
    exit 3
fi
echo "[yolo_world/start] graspnet_msgs OK: $(python3 -c \
'import graspnet_msgs.srv as s; print(s.__file__)')" >&2

CODEGEN_PROTO="$PKG/rbnx-build/codegen/proto_gen"
CODEGEN_MCP="$PKG/rbnx-build/codegen/robonix_mcp_types"
if [[ ! -d "$CODEGEN_PROTO" || ! -d "$CODEGEN_MCP" ]]; then
    echo "[yolo_world/start] ERR: codegen output missing — run scripts/build.sh" >&2
    exit 2
fi
export PYTHONPATH="$CODEGEN_PROTO:$CODEGEN_MCP:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

exec python3 -u -m yolo_world.main
