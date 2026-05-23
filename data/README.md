# yolo_world_rbnx — runtime data

This directory holds **runtime output**, not source files:

- `last_detection.jpg` — overwritten every 1 s by the package's
  `_periodic_broadcast` timer with the latest RGB frame plus all
  detected bounding boxes (≥0.2 confidence) drawn on top, plus a
  `HH:MM:SS` timestamp in the upper-left corner so you can tell
  the file is fresh.

The path is configurable via `cfg.debug_overlay_path`. Set it to
the empty string `""` to disable writes entirely.

The contents of this directory are not tracked by git (see
`.gitignore`). The directory itself is kept in the repo so the
default output path resolves to a writable location even on a
fresh checkout.
