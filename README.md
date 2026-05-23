# yolo_world_rbnx

Robonix package for open-vocabulary object detection on the Piper +
Orbbec Dabai DCW grasp pipeline. Stage 4A of the migration.

## What it does

Wraps **ultralytics YOLOE** (the prompt-free, open-vocabulary YOLO-World
variant — 4585 predefined classes, no prompt needed). Owns
`robonix/service/perception/object_detect/*`. Synchronously subscribes
RGB + depth + camera_info, runs YOLOE on demand, returns the
highest-confidence match for the requested class with both 2D bbox
and 3D camera-frame centroid (back-projected via median depth).

This is **the only GPU/ML package** in the piper grasp pipeline. Everything
else is pure CPU. Inference latency (RTX 3060 / Jetson Orin AGX) is
~150–300 ms per call; weights are ~50 MB.

## Two surfaces, one detection function

```
        Pilot LLM            pick.py / yolo_grasp.py
            │                         │
            ▼                         ▼
   atlas-routed MCP            ROS service
   detect_object              /yolo/detect_object
            │                         │
            └─────────► _detect_object() ◄────────┘
                              │
                              ▼
                        YOLOE inference
                        + 3D back-project
```

| surface | contract / topic | who calls it | when to remove |
|---|---|---|---|
| **MCP** | `robonix/service/perception/object_detect/detect_object` | Pilot LLM (the new path) | never |
| **ROS service** | `/yolo/detect_object` (graspnet_msgs/srv/ObjectDetectionRequest) | pick.py + yolo_grasp.py (legacy) | Stage 6 cutover |

The two surfaces share one `_detect_object()` function. The MCP and
ROS request/response shapes are byte-equivalent (we ship a renamed
copy `DetectObject.srv` for the MCP path; same fields). Removing
the ROS surface is a single delete in Stage 6 once pick_skill_rbnx
moves to the MCP one.

## Atlas-resolved upstream deps

Three camera streams, resolved via atlas at `on_init`:

| key | contract | fallback if atlas can't resolve |
|---|---|---|
| `rgb` | `robonix/primitive/camera/rgb` | `/camera/color/image_raw` |
| `depth` | `robonix/primitive/camera/depth` | `/camera/depth/image_raw` |
| `camera_info` | `robonix/primitive/camera/camera_info` | `/camera/color/camera_info` |

OrbbecSDK_rbnx (Stage 1) provides all three with these exact fallback
endpoints, so the resolved value normally equals the fallback. Atlas
indirection only matters when a different camera package (e.g.
realsense_camera_rbnx for ranger deploys) provides them.

You can override resolution from the deploy manifest's `config:`:

```yaml
service:
  - name: yolo_world
    config:
      rgb_topic: /alt/color/image_raw   # bypass atlas, hardcode
```

## Architecture

```
yolo_world_rbnx/
├── package_manifest.yaml
├── capabilities/
│   ├── service/perception/object_detect/
│   │   ├── driver.v1.toml         # rpc, lifecycle/srv/Driver.srv
│   │   └── detect_object.v1.toml  # rpc/MCP, perception/srv/DetectObject.srv
│   └── lib/perception/srv/
│       └── DetectObject.srv       # codegen → DetectObject_Request/_Response
├── yolo_world/
│   ├── __init__.py
│   ├── main.py                    # robonix Service + rclpy thread
│   └── _upstream/
│       └── object_detection_node.py  # original upstream node (reference)
├── scripts/
│   ├── build.sh                   # colcon graspnet_msgs + rbnx codegen --mcp
│   └── start.sh                   # source overlays, exec yolo_world.main
└── src/
    └── graspnet_msgs/             # vendored msg + srv (32 KB)
        ├── msg/{DetectedObject,DetectedObjects,GraspPose,PiperStatusMsg}.msg
        └── srv/{ObjectDetectionRequest,GraspRequest}.srv
```

## Lifecycle

```
on_init  ── load YOLOE weights ──► resolve atlas camera contracts
                                   ──► spawn rclpy thread
                                       (subscribers + ROS service host
                                        + publishers + 1Hz periodic
                                        broadcast on /yolo/detect_objects)

on_deactivate ── stop rclpy thread; keep model in memory for fast
                  re-ACTIVATE (avoids ~3-5s torch reload).
```

## Build / run

```bash
# Standalone build (rbnx boot does this automatically on first start).
cd /Users/howenliu/lab/packages/yolo_world_rbnx
bash scripts/build.sh

# rbnx boot path (recommended).
cd /Users/howenliu/lab/piper_grasp_deploy
rbnx boot
```

## Manual single-package debugging

`scripts/dev_source.sh` is a helper for running this package by hand
outside of `rbnx boot` — useful when you need pdb / fast iter / a
clean stdout. It sources the same overlays / PYTHONPATH that
`scripts/start.sh` would, and verifies that the vendored
`graspnet_msgs` is the one importable in this shell.

```bash
# In a shell that will run python3 -m yolo_world.main:
cd /Users/howenliu/lab/packages/yolo_world_rbnx
source scripts/dev_source.sh
python3 -u -m yolo_world.main
```

**Do NOT add `source dev_source.sh` to `~/.bashrc`.** The script does
`$(rbnx path …)` which spawns a child bash; if `.bashrc` re-sources
the script, every child bash recurses, which manifests as N copies of
`[yolo_world-source] package root: …` in the rbnx-boot log followed
by a 60s registration timeout (the real `python3 -m yolo_world.main`
never actually runs). The script has a reentrancy guard against this,
but the right place for it is `source` it on demand from a single
debugging shell, not your shell init.

`rbnx boot` itself uses `scripts/start.sh`, which has its own
sourcing chain and doesn't need this helper.

## Verification (in order)

```bash
# 1. atlas-side: provider + capabilities visible
rbnx caps | grep yolo_world
# expect:
#   yolo_world  com.robonix.piper_grasp.yolo_world  ACTIVE
#     robonix/service/perception/object_detect/driver         (rpc/grpc)
#     robonix/service/perception/object_detect/detect_object  (rpc/mcp)

# 2. MCP path (new): Pilot would invoke this; manually:
rbnx ask "is there a comb in the camera view?"
# pilot calls detect_object("comb") → expects success=true + bbox + 3D center

# 3. ROS path (legacy): pick.py / yolo_grasp.py still uses this
ros2 service call /yolo/detect_object \
    graspnet_msgs/srv/ObjectDetectionRequest "{object_name: 'cup'}"
# expect: success=true, bbox_2d=[...], object_center_3d=[x,y,z]

# 4. periodic broadcast (visualisation, low priority)
ros2 topic echo /yolo/detect_objects --once
# expect: array of DetectedObject with bbox + 3D center per detection
```

## YOLOE weights

Default model: `yoloe-11l-seg-pf.pt`. Resolution priority:

1. `config: { model_path: /abs/path/to/custom.pt }` in the deploy manifest
2. `<pkg>/yolo_world/weights/yoloe-11l-seg-pf.pt` (committed to git? no — see `.gitignore`)
3. ultralytics auto-download to `~/.cache/torch/hub/` on first inference

**First-run latency is dominated by the weights download** (~50 MB
over network) on a fresh deploy machine. Pre-warm by running the
ultralytics CLI once: `yolo predict model=yoloe-11l-seg-pf.pt source=...`.

## Vendored vs system-installed

| component | mode | rationale |
|---|---|---|
| `graspnet_msgs` (msg + srv) | **vendored** | upstream graspnet repo is huge + we only need the IDL |
| `ultralytics` | system pip install | ~1 GB w/ pytorch — vendoring costs more than it saves |
| YOLOE weights | system download (lazy) | ultralytics handles it |
| `cv_bridge` / `message_filters` | system apt (humble) | ROS bindings, must match host's libstdc++ |

## Coupling with Stage 4B (yolo_grasp_rbnx)

yolo_grasp_rbnx (the grasp-pose estimator) **calls this package** as
a client. Two paths possible:

* **Today (legacy)**: yolo_grasp.py uses `rclpy.Client` against
  `/yolo/detect_object`. No atlas indirection. We MUST keep this
  ROS service alive until Stage 6 cutover.
* **Tomorrow (atlas)**: yolo_grasp_rbnx atlas-resolves the MCP
  contract and calls the typed handler. Stage 4B already wires
  this path; whether yolo_grasp.py uses it depends on the
  `prefer_atlas` config flag in yolo_grasp_rbnx.

Both paths land in the same `_detect_object()` function here.

## Failure modes

| symptom | cause | fix |
|---|---|---|
| `init: YOLOE load failed` | `pip install ultralytics` missing or wrong torch | `pip install ultralytics torch` |
| `camera data not available` on every call | RGB / depth / camera_info topics not publishing | check OrbbecSDK_rbnx is ACTIVE; `ros2 topic hz /camera/color/image_raw` |
| MCP works but ROS service hangs | mismatched `graspnet_msgs` between yolo_world's overlay and pick.py's overlay | rebuild both; or move pick.py to MCP path |
| `back-project: no valid depth in bbox` | depth sensor not aligned to RGB, or object beyond 3 m | enable `depth_registration:=true` in OrbbecSDK_rbnx (Stage 1 default); check object distance |
