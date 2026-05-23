#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""yolo_world_rbnx — open-vocabulary object detection service.

Wraps ultralytics YOLOE (the prompt-free YOLO-World variant with 4585
predefined classes). Owns ``robonix/service/perception/object_detect/*``.

Two parallel surfaces, sharing one detection function:

    1. Atlas-routed MCP   (the new path, what Pilot's LLM sees)
       robonix/service/perception/object_detect/detect_object
       — input/output: DetectObject_Request / _Response (codegen'd from
         capabilities/lib/perception/srv/DetectObject.srv)

    2. Legacy ROS service (compat path, what pick.py + yolo_grasp.py
       still call)
       /yolo/detect_object  (graspnet_msgs/srv/ObjectDetectionRequest)
       — same field shape (renamed for namespace), same handler.

Both eventually return the highest-confidence YOLOE match (≥0.2) for
the requested name, with 2D bbox + 3D camera-frame centroid (median
depth in bbox back-projected through the camera_info K matrix).

Lifecycle (per Robonix developer guide §5):
    on_init      — heavy is OK here per nav2_wrapper precedent: load
                   YOLOE weights, resolve atlas camera contracts, spawn
                   rclpy background thread with subscribers + ROS
                   service + publishers. Returning Err here aborts boot
                   cleanly (rbnx-cli will print the cause).

We do NOT spawn an external `ros2 run` / `ros2 launch` subprocess.
The whole node lives in this Python process — same as
`mid360_imu_rbnx`.

Atlas-resolved deps (config_key, contract_id, fallback_topic):
    rgb           primitive/camera/rgb         /camera/color/image_raw
    depth         primitive/camera/depth       /camera/depth/image_raw
    camera_info   primitive/camera/camera_info /camera/color/camera_info

Each can be overridden by a hard-coded topic in cfg (e.g. for bench
testing without atlas) — see `_resolve_topic`.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robonix_api import ATLAS, Service, Ok, Err  # noqa: E402

logging.basicConfig(
    level=os.environ.get("YOLO_WORLD_LOG_LEVEL", "INFO"),
    format="[yolo_world] %(message)s",
)
log = logging.getLogger("yolo_world")

# Provider id MUST match the deploy manifest's `service: - name: ...`.
yolo_world = Service(
    id=os.environ.get("ROBONIX_CAPABILITY_ID", "yolo_world"),
    namespace="robonix/service/perception/object_detect",
)

# ── shared state (between rclpy thread + MCP handlers) ──────────────────────
_state_lock = threading.Lock()
_initialized = False

# Latest synchronized camera frame, captured by message_filters callback.
_latest_color_image = None       # numpy ndarray, RGB
_latest_depth_image = None       # numpy ndarray, depth in mm (uint16)
_latest_camera_info = None       # sensor_msgs/CameraInfo

# YOLOE model (loaded once in on_init, reused on every detect call).
_yolo_model = None
_bridge = None                   # cv_bridge.CvBridge

# ROS thread state.
_ros_node = None
_ros_thread: Optional[threading.Thread] = None
_ros_stop_evt = threading.Event()

# Synchronization between init() and _ros_thread_main:
#   - _ros_ready_evt is set exactly once after the rclpy node has
#     successfully created all of its publishers / subscribers /
#     services, OR immediately after a setup-time exception is caught.
#   - _ros_thread_error holds the exception (if any) so init() can
#     propagate it to atlas as Err(...). Without this fail-fast path,
#     a thread crash inside create_publisher (e.g. typesupport .so
#     missing → "type_support is null") leaves the package looking
#     ACTIVE on atlas while none of the ROS surfaces are actually up.
_ros_ready_evt = threading.Event()
_ros_thread_error: Optional[BaseException] = None
_ROS_READY_TIMEOUT_S = 15.0


# ── upstream-resolution helpers ─────────────────────────────────────────────
_DEFAULT_TOPICS = {
    "rgb":         "/camera/color/image_raw",
    "depth":       "/camera/depth/image_raw",
    "camera_info": "/camera/color/camera_info",
}

_DEP_CONTRACTS = {
    "rgb":         "robonix/primitive/camera/rgb",
    "depth":       "robonix/primitive/camera/depth",
    "camera_info": "robonix/primitive/camera/camera_info",
}


def _resolve_topic(key: str, cfg: dict) -> str:
    """Resolve the ROS topic name to subscribe for `key`.

    Priority:
      1. cfg[f'{key}_topic'] — explicit override (debug / bench testing)
      2. atlas find_capability(<contract>, transport=ros2) → endpoint
      3. _DEFAULT_TOPICS[key] (matches OrbbecSDK_rbnx defaults)
    """
    explicit = (cfg.get(f"{key}_topic") or "").strip()
    if explicit:
        log.info("topic[%s] explicit cfg override: %s", key, explicit)
        return explicit

    contract_id = _DEP_CONTRACTS[key]
    try:
        caps = ATLAS.find_capability(contract_id=contract_id, transport="ros2")
    except Exception as e:  # noqa: BLE001
        log.warning("atlas query %s failed: %s — falling back to default",
                    contract_id, e)
        caps = []

    if caps:
        try:
            ch = yolo_world.connect_capability(caps[0], contract_id, "ros2")
            ep = ch.endpoint
            try:
                ch.close()
            except Exception:  # noqa: BLE001
                pass
            if ep:
                log.info("topic[%s] resolved via atlas: %s (provider=%s)",
                         key, ep, caps[0].provider_id)
                return ep
        except Exception as e:  # noqa: BLE001
            log.warning("atlas connect %s failed: %s", contract_id, e)

    fallback = _DEFAULT_TOPICS[key]
    log.warning("topic[%s] no atlas provider; using default %s", key, fallback)
    return fallback


# ── detection core (shared between MCP + ROS service) ───────────────────────
def _detect_object(object_name: str) -> dict:
    """Detection core. Returns a dict with the same keys both surfaces fill.

    Returns:
      {
        "success":          bool,
        "message":          str,
        "bbox_2d":          [x_min, y_min, x_max, y_max] (pixels) or [],
        "object_center_3d": [x, y, z] (meters, camera optical) or [],
        "confidence":       float,
      }
    """
    global _latest_color_image, _latest_depth_image, _latest_camera_info

    with _state_lock:
        if (_latest_color_image is None or _latest_depth_image is None
                or _latest_camera_info is None):
            return {
                "success": False,
                "message": ("camera data not available "
                            "(waiting for synchronized RGB+depth+camera_info)"),
                "bbox_2d": [],
                "object_center_3d": [],
                "confidence": 0.0,
            }
        color_img = _latest_color_image.copy()
        depth_img = _latest_depth_image.copy()
        cam_info = _latest_camera_info

    # 1. YOLOE inference.
    try:
        results = _yolo_model.predict(  # type: ignore[union-attr]
            source=color_img, device="cuda:0", verbose=False)
        detection = results[0]
    except Exception as e:  # noqa: BLE001
        return {
            "success": False, "message": f"YOLOE inference failed: {e}",
            "bbox_2d": [], "object_center_3d": [], "confidence": 0.0,
        }

    if detection is None or len(detection.boxes) == 0:
        return {
            "success": False, "message": "no objects detected in image",
            "bbox_2d": [], "object_center_3d": [], "confidence": 0.0,
        }

    # 2. Filter by confidence ≥ 0.2 (matches upstream threshold).
    boxes = detection.boxes.xyxy.cpu().numpy()
    confs = detection.boxes.conf.cpu().numpy()
    cls   = detection.boxes.cls.cpu().numpy()

    name_lower = object_name.lower().strip()
    matches: list[tuple[int, float]] = []  # (idx, conf)
    for i in range(len(boxes)):
        c = float(confs[i])
        if c < 0.2:
            continue
        det_name = detection.names[int(cls[i])].lower()
        if (name_lower == det_name or name_lower in det_name
                or det_name in name_lower):
            matches.append((i, c))

    if not matches:
        return {
            "success": False,
            "message": f"object '{object_name}' not found at confidence ≥ 0.2",
            "bbox_2d": [], "object_center_3d": [], "confidence": 0.0,
        }

    best_idx, best_conf = max(matches, key=lambda t: t[1])
    x1, y1, x2, y2 = boxes[best_idx]
    bbox = [float(int(x1)), float(int(y1)), float(int(x2)), float(int(y2))]

    # 3. 3D back-projection from depth + intrinsics.
    center_3d = _back_project_3d(bbox, depth_img, cam_info)

    return {
        "success":          True,
        "message":          f"detected '{object_name}' at conf {best_conf:.3f}",
        "bbox_2d":          bbox,
        "object_center_3d": center_3d if center_3d is not None else [],
        "confidence":       float(best_conf),
    }


def _back_project_3d(bbox_2d, depth_img, cam_info):
    """Median-depth back-projection. Returns [x, y, z] meters, or None.

    Median (not mean) on the bbox depth ROI to reject background/zero
    pixels that median-tolerates. Depth in mm → meters.
    """
    try:
        import numpy as np
        x_min, y_min, x_max, y_max = [int(v) for v in bbox_2d]
        roi = depth_img[y_min:y_max, x_min:x_max]
        valid = roi[(roi > 0) & (roi < 3000)]   # mm, max 3m
        if len(valid) == 0:
            log.warning("back-project: no valid depth in bbox %s",
                        (x_min, y_min, x_max, y_max))
            return None
        z = float(np.median(valid)) / 1000.0    # mm → m

        cx_pix = (x_min + x_max) / 2.0
        cy_pix = (y_min + y_max) / 2.0
        K = cam_info.k
        fx, fy = float(K[0]), float(K[4])
        cx, cy = float(K[2]), float(K[5])

        return [
            (cx_pix - cx) * z / fx,
            (cy_pix - cy) * z / fy,
            z,
        ]
    except Exception as e:  # noqa: BLE001
        log.error("back-project failed: %s", e)
        return None


# ── ROS bring-up (background thread) ────────────────────────────────────────
def _ros_thread_main(rgb_topic: str, depth_topic: str, info_topic: str) -> None:
    """Subscribe + ROS service host + topic publishers, all in one rclpy
    node. Stays alive for the lifetime of the package.

    Setup phase wrapped in try/except: any failure (typesupport .so
    not loadable, intra-thread import error, rclpy.init failure,
    etc.) is captured into _ros_thread_error so init() can return
    Err(...) instead of falsely reporting Ok and leaving us "ACTIVE
    but mute" on atlas. _ros_ready_evt is set in BOTH the success
    and failure paths so init()'s wait() always returns within
    _ROS_READY_TIMEOUT_S (or sooner).
    """
    global _ros_node, _bridge
    global _latest_color_image, _latest_depth_image, _latest_camera_info
    global _ros_thread_error

    node = None
    try:
        import rclpy                              # noqa: E402
        from rclpy.node import Node               # noqa: E402
        from sensor_msgs.msg import Image, CameraInfo  # noqa: E402
        from cv_bridge import CvBridge            # noqa: E402
        import message_filters                    # noqa: E402
        # graspnet_msgs is vendored in src/ and built by colcon — overlay is
        # sourced by start.sh before this module imports.
        from graspnet_msgs.srv import ObjectDetectionRequest  # noqa: E402
        from graspnet_msgs.msg import DetectedObject, DetectedObjects  # noqa: E402

        rclpy.init(args=None)
        _bridge = CvBridge()
        node = Node("yolo_world_node")
        _ros_node = node

        # Synchronized RGB + depth + camera_info subscribers via message_filters.
        sub_rgb   = message_filters.Subscriber(node, Image,      rgb_topic)
        sub_depth = message_filters.Subscriber(node, Image,      depth_topic)
        sub_info  = message_filters.Subscriber(node, CameraInfo, info_topic)
        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=0.1)

        def _camera_cb(rgb_msg, depth_msg, info_msg):
            global _latest_color_image, _latest_depth_image, _latest_camera_info
            try:
                # `passthrough` keeps RGB as-is (Orbbec publishes rgb8 / 16UC1).
                rgb   = _bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding="passthrough")
                depth = _bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            except Exception as e:  # noqa: BLE001
                node.get_logger().error(f"camera_cb cv_bridge: {e}")
                return
            with _state_lock:
                _latest_color_image = rgb
                _latest_depth_image = depth
                _latest_camera_info = info_msg
        sync.registerCallback(_camera_cb)
        log.info("subscribed: rgb=%s  depth=%s  info=%s",
                 rgb_topic, depth_topic, info_topic)

        # Compat ROS service (pick.py, yolo_grasp.py both call this).
        def _ros_service_handler(request, response):
            result = _detect_object(request.object_name)
            response.success           = result["success"]
            response.message           = result["message"]
            response.bbox_2d           = list(result["bbox_2d"])
            response.object_center_3d  = list(result["object_center_3d"])
            response.confidence        = float(result["confidence"])
            return response
        node.create_service(ObjectDetectionRequest, "/yolo/detect_object",
                            _ros_service_handler)
        log.info("ROS service up: /yolo/detect_object")

        # Compat publishers (kept for whoever subscribes — currently nobody
        # critical, but the upstream node had them and removing changes the
        # observable shape).
        detection_image_pub = node.create_publisher(Image, "/yolo/detection_image", 10)
        detected_objects_pub = node.create_publisher(
            DetectedObjects, "/yolo/detect_objects", 10)
        # Stash in module scope so periodic timer can reach them.
        globals()["_detection_image_pub"]  = detection_image_pub
        globals()["_detected_objects_pub"] = detected_objects_pub

        # Periodic broadcast of all detected objects, 1 Hz (matches upstream).
        node.create_timer(1.0, _periodic_broadcast)
    except BaseException as e:  # noqa: BLE001 — must include SystemExit/KeyboardInterrupt etc.
        # Setup-time failure. Most common cause in practice: graspnet_msgs
        # typesupport .so files not on LD_LIBRARY_PATH, so create_publisher
        # raises "type_support is null". Capture and signal init().
        _ros_thread_error = e
        log.error("rclpy thread setup failed: %s: %s",
                  type(e).__name__, e, exc_info=True)
        try:
            if node is not None:
                node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            import rclpy as _rclpy_for_shutdown
            if _rclpy_for_shutdown.ok():
                _rclpy_for_shutdown.shutdown()
        except Exception:  # noqa: BLE001
            pass
        _ros_ready_evt.set()
        return

    # Setup OK — let init() proceed.
    _ros_ready_evt.set()

    # Spin until told to stop. We don't use rclpy.spin() because we want
    # to honor _ros_stop_evt for clean shutdown.
    import rclpy  # noqa: E402  (re-import for the spin loop scope)
    while not _ros_stop_evt.is_set():
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as e:  # noqa: BLE001
            # Per-iteration errors should NOT bring the whole thread down
            # — that would silently re-introduce the "alive but mute"
            # failure mode. Log and continue.
            log.warning("rclpy.spin_once raised: %s", e)
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    log.info("rclpy thread exited")


def _periodic_broadcast() -> None:
    """Publish all detected objects on /yolo/detect_objects (best-effort).

    Mirrors the upstream node's 1 Hz timer. Useful for visualisation
    tools subscribed to that topic; nothing in the grasp pipeline
    consumes it directly."""
    pub = globals().get("_detected_objects_pub")
    if pub is None or _yolo_model is None:
        return
    with _state_lock:
        if _latest_color_image is None:
            return
        rgb = _latest_color_image.copy()
        depth = _latest_depth_image.copy() if _latest_depth_image is not None else None
        info = _latest_camera_info
    try:
        from graspnet_msgs.msg import DetectedObject, DetectedObjects  # noqa: E402
        results = _yolo_model.predict(source=rgb, device="cuda:0", verbose=False)
        det = results[0]
        msg = DetectedObjects()
        if _ros_node is not None:
            msg.header.stamp = _ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"
        if det is not None and len(det.boxes) > 0:
            for i in range(len(det.boxes)):
                conf = float(det.boxes.conf[i].cpu().numpy())
                if conf < 0.2:
                    continue
                x1, y1, x2, y2 = det.boxes.xyxy[i].cpu().numpy()
                bbox = [float(int(x1)), float(int(y1)),
                        float(int(x2)), float(int(y2))]
                obj = DetectedObject()
                obj.object_name = det.names[int(det.boxes.cls[i].cpu().numpy())]
                obj.bbox_2d = bbox
                obj.confidence = conf
                if depth is not None and info is not None:
                    c3 = _back_project_3d(bbox, depth, info)
                    obj.object_center_3d = c3 if c3 is not None else []
                else:
                    obj.object_center_3d = []
                msg.objects.append(obj)
        pub.publish(msg)
    except Exception as e:  # noqa: BLE001
        log.debug("periodic broadcast skipped: %s", e)


# ── lifecycle ───────────────────────────────────────────────────────────────
@yolo_world.on_init
def init(cfg):
    """Driver(CMD_INIT). Heavy:
      1. parse cfg + load YOLOE weights
      2. resolve atlas camera contracts → topic names
      3. spawn rclpy thread (subscribers + service + publishers)
    """
    global _initialized, _yolo_model
    with _state_lock:
        if _initialized:
            return Ok()

    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as e:
            return Err(f"bad config_json: {e}")

    # 1. Load YOLOE weights.
    #
    # We VENDOR the primary YOLOE-11l prompt-free checkpoint
    # (yoloe-11l-seg-pf.pt, ~71 MB) under yolo_world/weights/. The
    # operator can override via config.model_path to point at a
    # different / larger checkpoint they dropped in there. If the
    # vendored file is missing AND no override is given, fail loud:
    # the alternative (silently letting ultralytics download from
    # the internet at first inference) makes deployment behaviour
    # depend on whether the deploy host has GitHub access, which is
    # exactly the kind of magic vendor-by-default is meant to kill.
    pkg_root = Path(os.environ.get(
        "RBNX_PACKAGE_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
    ))
    weights_default = pkg_root / "yolo_world" / "weights" / "yoloe-11l-seg-pf.pt"
    model_path = cfg.get("model_path") or str(weights_default)
    if not Path(model_path).is_file():
        return Err(
            f"YOLOE weights not found at {model_path}. "
            f"Either (a) the vendored checkpoint at {weights_default} "
            f"was stripped from the clone (check `git status` — large "
            f"files sometimes get filtered by sparse-checkout or "
            f"workspace dedup) and a fresh `git pull` will fix it, or "
            f"(b) you set config.model_path to a non-existent path."
        )
    log.info("loading YOLOE: %s", model_path)
    try:
        from ultralytics import YOLOE  # noqa: E402
        _yolo_model = YOLOE(model_path)
    except Exception as e:  # noqa: BLE001
        return Err(f"YOLOE load failed: {e}")
    log.info("YOLOE loaded; supports prompt-free detection (4585 classes)")

    # 2. Resolve atlas camera contracts.
    rgb_topic   = _resolve_topic("rgb",         cfg)
    depth_topic = _resolve_topic("depth",       cfg)
    info_topic  = _resolve_topic("camera_info", cfg)

    # 3. Spawn rclpy thread. We then BLOCK on _ros_ready_evt — the
    # thread signals it after either successful setup of every
    # publisher / subscriber / service, OR a setup-time exception
    # (captured into _ros_thread_error). This makes init() fail-fast
    # and propagate the error to atlas via Err(...), instead of
    # returning Ok and leaving us "ACTIVE but mute" (the typesupport.so
    # / LD_LIBRARY_PATH failure mode that was previously silent).
    global _ros_thread, _ros_thread_error
    _ros_stop_evt.clear()
    _ros_ready_evt.clear()
    _ros_thread_error = None
    _ros_thread = threading.Thread(
        target=_ros_thread_main,
        args=(rgb_topic, depth_topic, info_topic),
        name="yolo_world-ros",
        daemon=True,
    )
    _ros_thread.start()

    if not _ros_ready_evt.wait(timeout=_ROS_READY_TIMEOUT_S):
        _ros_stop_evt.set()
        _ros_thread.join(timeout=2.0)
        return Err(
            f"rclpy thread did not become ready within "
            f"{_ROS_READY_TIMEOUT_S}s (likely blocked in rclpy.init or "
            f"create_publisher; check `ps -T` and the log just above)"
        )

    if _ros_thread_error is not None:
        err = _ros_thread_error
        _ros_stop_evt.set()
        _ros_thread.join(timeout=2.0)
        return Err(
            f"rclpy thread setup failed: {type(err).__name__}: {err} — "
            f"if message mentions 'libgraspnet_msgs__rosidl_typesupport_*.so', "
            f"the vendored graspnet_msgs lib/ is not on LD_LIBRARY_PATH "
            f"(check scripts/start.sh's graspnet_msgs path injection)"
        )

    with _state_lock:
        _initialized = True
    log.info("init complete: object_detect MCP + /yolo/detect_object live")
    return Ok()


@yolo_world.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Stop the rclpy thread; keep the model loaded
    in case we get re-ACTIVATEd (avoid the multi-second weights reload).
    """
    log.info("CMD_DEACTIVATE: stopping rclpy thread")
    _ros_stop_evt.set()
    if _ros_thread is not None:
        _ros_thread.join(timeout=5.0)
    with _state_lock:
        global _initialized
        _initialized = False
    return Ok()


# ── atlas-routed MCP handler (Pilot's view) ─────────────────────────────────
# Imported AFTER on_init body so the codegen module is loadable from the
# overlay setup.bash sourced by start.sh.
from perception_mcp import (  # noqa: E402  pylint: disable=wrong-import-position
    DetectObject_Request, DetectObject_Response,
)


@yolo_world.mcp("robonix/service/perception/object_detect/detect_object")
def detect_object(req: DetectObject_Request) -> DetectObject_Response:
    """Detect a named object. Open-vocab over YOLOE's 4585 classes."""
    result = _detect_object(req.object_name)
    return DetectObject_Response(
        bbox_2d          = list(result["bbox_2d"]),
        object_center_3d = list(result["object_center_3d"]),
        confidence       = float(result["confidence"]),
        success          = bool(result["success"]),
        message          = str(result["message"]),
    )


def main() -> int:
    import signal
    def _on_signal(sig, _frame):
        log.info("signal %d — shutting down", sig)
        _ros_stop_evt.set()
        if _ros_thread is not None:
            _ros_thread.join(timeout=3.0)
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT,  _on_signal)
    try:
        yolo_world.run()
    finally:
        _ros_stop_evt.set()
        if _ros_thread is not None:
            _ros_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
