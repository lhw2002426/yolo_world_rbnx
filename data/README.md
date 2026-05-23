# yolo_world_rbnx — runtime data

This directory holds **runtime output**, not source files:

- `last_detection.<ext>` — overwritten every 1 s by the package's
  `_periodic_broadcast` timer with the latest RGB frame plus all
  detected bounding boxes (≥0.2 confidence) drawn on top, plus a
  `HH:MM:SS` timestamp in the upper-left corner so you can tell
  the file is fresh.

The path is configurable via `cfg.debug_overlay_path` (default
`<pkg>/data/last_detection.jpg`). Set it to the empty string `""`
to disable writes entirely.

**Note on file extension**: the actual extension on disk depends on
what your OpenCV build can encode. If the requested extension fails
(`opencv-python-headless` and the apt-installed `python3-opencv`
on Ubuntu often ship without libjpeg, so `.jpg` writes fail with
"could not find a writer for the specified extension"), the package
silently falls back through:

  1. requested extension (`.jpg` by default)
  2. `.png` — almost always works, included in every cv2 build
  3. PIL.PNG / PIL.JPEG — if Pillow is installed
  4. raw PPM — last-ditch, only depends on numpy

Whichever ends up working becomes the actual filename. If you
asked for `.jpg` but ended up with `last_detection.png`, the
package logs ONE warning (then stays quiet); just open the `.png`
the deploy actually wrote.

The contents of this directory are not tracked by git (see
`.gitignore`). The directory itself is kept in the repo so the
default output path resolves to a writable location even on a
fresh checkout.
