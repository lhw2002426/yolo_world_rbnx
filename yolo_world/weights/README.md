# YOLO weights — vendored

This directory ships the model checkpoints `yolo_world_rbnx`
loads at `on_init` time, so deployment doesn't require the host
to have internet access (or the right GitHub TLS roots, or the
right ultralytics version, …).

## What's here

| file | size | source | role |
|------|------|--------|------|
| `yoloe-11l-seg-pf.pt` | ~71 MB | upstream `grasp/driver/yoloe/yoloe-11l-seg-pf.pt` (commit before 2026-05) | primary YOLOE 11-large prompt-free model. Loaded by default. |

The hash + provenance match the upstream `yoloe/` package the
migration imported from; if you need bit-for-bit reproducibility
of the original `object_detection_node.py` behaviour, this is
the file to use.

## What's NOT here

- **`yolov8s-world.pt`** (~26 MB) — upstream had it as a comparison
  baseline, never the production model. Not vendored. Drop it in
  yourself and point `config.model_path` at it if you need it.
- **TensorRT `.engine` caches** — generated at runtime, host-specific
  (CUDA arch + driver version), so committing them helps no one.
  `.gitignore` excludes `*.engine` / `*.onnx`.
- **YOLOE 11x / xl / m variants** — significantly larger or slower,
  not used by the current Stage 4A pipeline. `.gitignore` excludes
  `yoloe-11x-*.pt` patterns so they don't accidentally get committed.

## Overriding from config

`yolo_world_rbnx`'s `package_manifest.yaml` exposes
`config.model_path`. In `piper_grasp_deploy/robonix_manifest.yaml`:

```yaml
- name: yolo_world
  url: https://github.com/lhw2002426/yolo_world_rbnx
  branch: main
  config:
    # Empty → uses vendored yoloe-11l-seg-pf.pt
    model_path: ""
    # Absolute path → uses that file instead
    # model_path: /opt/yolo_weights/my-finetuned.pt
```

If the path you give doesn't exist, `on_init` fails LOUD with
a clear message — we removed the "auto-download from ultralytics"
fallback because silent network access during deployment is the
exact behaviour vendor-by-default is meant to prevent.

## Updating

When upstream YOLOE ships a new checkpoint:
```bash
cp /path/to/new/yoloe-11l-seg-pf.pt \
   packages/yolo_world_rbnx/yolo_world/weights/
cd packages/yolo_world_rbnx
git add yolo_world/weights/yoloe-11l-seg-pf.pt
git commit -m "weights: bump YOLOE checkpoint to <upstream version>"
git push
```
The 71 MB push will be slow on a flaky connection; consider
`git -c http.postBuffer=524288000 push` if it stalls.
