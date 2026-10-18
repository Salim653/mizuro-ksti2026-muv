---
name: reference-ksti-arena3d-dataset
description: KSTI 2026 arena-3d synthetic dataset exports (D:\00 DOWNLOADS\ksti-arena3d-race-260f-1280x720.zip and its (1) variant) already ship pixel-perfect ground truth — labels/*.png masks, annotations.jsonl polylines, telemetry, track geometry; the labeling task is format conversion, not annotation
metadata:
  type: project
---

The KSTI arena-3d dataset exports live in **`D:\00 DOWNLOADS\`**: `ksti-arena3d-race-260f-1280x720.zip` and a duplicate-named `ksti-arena3d-race-260f-1280x720 (1).zip`. Inspected 2026-10-16.

**They are the SAME 19.4 m lap, exported twice with different camera mounts** (frames differ between the two):
- z1: mount height 0.16 m, forward from anchor +0.015 m.
- z2: mount height 0.21 m, forward from anchor −0.165 m (set back).
- Both: 260 frames @ 20 fps, 1280×720, `mode: race`, `anchor: mid`.
- **The 20 fps is an EXPORT SAMPLING choice in the capture code, NOT a hardware or perception limit** (Salim corrected this 2026-10-17). Do not treat 20 fps — or the ~1.5 m/s lap speed it implies — as the car's real capture rate or its stable-speed ceiling; the Jetson/IMX219 can run far faster. See the corrected speed-ceiling note in [[project-ksti-2026-mav]].

**Each zip already contains full ground truth — do not build an annotation pipeline for these:**
- `labels/*.png` — single-channel segmentation maps, pixel = class id. Class ids spaced 20 apart, **read by nearest-id, not equality** (antialiased boundaries blend): background 0, surface 20, infield 40, **line 60 (painted edge line)**, **dash 80 (painted centre dash)**, ego 100, obstacle 120, gate 140, finish 160.
- `annotations.jsonl` — one object per frame: `intrinsics`, `pose`, `truth`, and `lines[]` = projected polylines with `cls ∈ {outer_edge, inner_edge, centre_dash}` and `pts` of `[u, v, depth]`; plus `points[]` for obstacle/checkpoint/start_finish.
- `telemetry.csv` — flat per-frame numeric truth (lane_offset_m + = left, heading_err_deg, steer_deg, cover_pct, clearances, score…).
- `dataset.json` (schema/intrinsics/class map), `track/track_data.json` (raw loop geometry; outer/inner are independently resampled — match neighbours by distance, not index), `code/baseline_line_follower.py` (loader + lane fit + Stanley + scoring), `README_DATASET.md`.

**Why (his actual goal):** Salim wanted to auto-label "the mid line and the edge line of the road" and had started segmenting in X-AnyLabeling. Both target classes already exist in the shipped ground truth — edge = mask class 60 / annotations `outer_edge`+`inner_edge`; mid = mask class 80 / `centre_dash`. So the work collapses to a **format conversion** (masks or polylines → whatever the training pipeline needs, e.g. YOLO-seg), not manual/SAM labeling. Checked 2026-10-16 and told him to stop labeling.

**How to apply:** Before proposing any annotation effort on a KSTI dataset, check for `labels/` + `annotations.jsonl` inside the zip first — the sim exports are self-labeling. Note also that the JSONL splits edges into `outer_edge`/`inner_edge` while the mask merges both sides into class 60, so "left/right as separate classes" requires the JSONL. README caveats to keep honest: synthetic frames have no motion blur/rolling shutter/glare/shadow, so a detector tuned here will look better than on the real hall floor.

Related: [[project-ksti-2026-mav]], [[reference-x-anylabeling]]
