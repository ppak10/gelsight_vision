"""Stitch the per-touch GelSight captures of a session into one grid view."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Union

import cv2
import numpy as np

from gelsight_vision.gelsight.height_map import save_height_preview

_NAME_RE = re.compile(r"^gelsight_(\d{4})\.png$")


def _grid_position(capture_index: int, touch_x: int, touch_y: int) -> tuple[int, int]:
    """Map 1-indexed capture number to (row, col) in the stitched grid.

    Grid is `touch_x` rows tall by `touch_y` columns wide. Captures fill the
    current row left-to-right before moving one row down. With touch_x=3 and
    touch_y=3 this gives [[1,2,3],[4,5,6],[7,8,9]] -- capture 1 at top-left,
    final capture at bottom-right.
    """
    k = capture_index - 1
    row = k // touch_y
    col = k % touch_y
    return row, col


def stitch_session(
    session_dir: Union[str, Path],
    touch_x: int,
    touch_y: int,
) -> None:
    """Read all `gelsight_NNNN.png` + `_depth.npy` pairs from `session_dir` and
    write three sibling files: `gelsight_stitched.png`,
    `gelsight_stitched_depth.npy`, `gelsight_stitched_depth_preview.png`."""
    session_dir = Path(session_dir)
    img_paths = sorted(
        p for p in session_dir.glob("gelsight_*.png") if _NAME_RE.match(p.name)
    )
    if not img_paths:
        print(f"stitch: no captures found in {session_dir}")
        return

    expected = touch_x * touch_y
    if len(img_paths) != expected:
        print(
            f"stitch: expected {expected} captures, found {len(img_paths)}; "
            "stitching available frames into their grid slots"
        )

    first_img = cv2.rotate(cv2.imread(str(img_paths[0])), cv2.ROTATE_90_COUNTERCLOCKWISE)
    first_depth = np.rot90(
        np.load(img_paths[0].with_name(img_paths[0].stem + "_depth.npy")), k=1,
    )
    h, w = first_img.shape[:2]
    dh, dw = first_depth.shape

    stitched_img = np.zeros((touch_x * h, touch_y * w, 3), dtype=first_img.dtype)
    stitched_depth = np.full(
        (touch_x * dh, touch_y * dw), np.nan, dtype=first_depth.dtype
    )

    for path in img_paths:
        match = _NAME_RE.match(path.name)
        if not match:
            continue
        idx = int(match.group(1))
        row, col = _grid_position(idx, touch_x, touch_y)
        if not (0 <= row < touch_x and 0 <= col < touch_y):
            print(f"stitch: capture {idx} falls outside the {touch_x}x{touch_y} grid; skipping")
            continue
        img = cv2.rotate(cv2.imread(str(path)), cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth = np.rot90(np.load(path.with_name(path.stem + "_depth.npy")), k=1)
        stitched_img[row * h:(row + 1) * h, col * w:(col + 1) * w] = img
        stitched_depth[row * dh:(row + 1) * dh, col * dw:(col + 1) * dw] = depth

    img_out = session_dir / "gelsight_stitched.png"
    depth_out = session_dir / "gelsight_stitched_depth.npy"
    preview_out = session_dir / "gelsight_stitched_depth_preview.png"
    cv2.imwrite(str(img_out), stitched_img)
    np.save(depth_out, stitched_depth)
    # Replace NaNs (missing tiles) with the per-tile median for the preview only.
    preview_depth = np.where(
        np.isnan(stitched_depth), np.nanmedian(stitched_depth), stitched_depth
    )
    save_height_preview(preview_depth, preview_out)
    print(f"stitched image -> {img_out}")
    print(f"stitched depth -> {depth_out}")
    print(f"stitched preview -> {preview_out}")
