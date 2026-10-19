# KSTI 2026 — Mini Autonomous Vehicle · 3D arena rehearsal

A browser view of the official *lintasan* (track map from the participant guidebook), rebuilt as
real 3D geometry, with a 1:10 mock car driving it in both competition modes.

---

## ▶ Open it

Unzip, then **double-click `index.html`**. No install, no server, no internet — Three.js is
bundled in the folder. Chrome or Edge.

**If a control or the map texture seems missing, check the build stamp** in the grey line under the
title (currently 16). Two separate things can cause it, and they are not the same problem:

- *Cached sub-resources.* Chrome caches `file://` scripts, so the bundled ones carry `?v=` tags and
  a red banner names any that fail to load. `Ctrl+Shift+R` clears it.
- *Chrome blocking file:// images.* A double-clicked page gets its own opaque origin, and Chrome
  then refuses to load `track_floor.jpg` outright (`net::ERR_FAILED`). That is why the map texture
  ships inlined as a `data:` URL in `track_floor_data.js` - without it the overlay option silently
  disappears from the panel, which is exactly what it looked like before. Regenerate it after any
  change to the source map with `python tools/inline_texture.py`.

If the stamp shows an older number than the one above, you are looking at a stale copy: re-extract
the zip.

| Key | Action |
|---|---|
| `Space` | run / pause the attempt |
| `R` | reset to the start line |
| `M` | switch Autonomous Race ↔ Race Challenge |
| `C` | cycle camera (orbit → chase → top → side → Cam POV) |
| `V` | jump straight to Cam POV |
| `P` | toggle the picture-in-picture camera inset |
| drag / wheel | orbit / zoom |

---

## 🗺 What the map actually is

Measured off the committee's PDF, not estimated. Scale anchored to the guidebook's own
"lebar lintasan 50 cm ±10%" rule.

| Property | Value |
|---|---|
| Centreline loop length | **19.41 m** (one lap = the race) |
| Track width | **0.50 m, constant** (0.499 – 0.503 m measured) |
| Arena footprint (outer line) | **5.21 × 6.58 m** |
| Infield | 4.17 × 5.54 m |
| Tightest centreline radius | **0.46 m** |
| Radius median / 10th pct | 1.97 m / 0.77 m |
| Straight running (>20 m radius) | **83 of 1440 samples ≈ 6 % of the lap** |
| Markings | 2 solid edge lines + 1 dashed centre line, ~16.7 mm painted width |

⚠ Two things worth raising with the committee:

1. The guidebook states a **minimum turning radius of 1.5 m**, but this sample map's tightest
   line radius is **0.46 m**. The drawing is labelled *contoh* (illustrative) — real corner speed
   is unknown until the test day.
2. The map shows a **constant** 0.50 m width, while the rules say width "does not have to be the
   same along the track". Expect variation the drawing does not model.

---

## 🚗 The mock car

Built to the guidebook envelope (BAB III.B), 2WD + Ackermann:

| | |
|---|---|
| L × W × H | 440 × 200 × 120 mm (limits: 600 / 400 / 450) |
| Wheelbase | 250 mm (limit 160 – 550) |
| Track width | 160 mm (limit 130 – 350) |
| Visible detail | steering knuckles, driven wheels, red emergency-stop button, front camera module |

Steering angle is geometric, not cosmetic: `atan(wheelbase / radius)`, with the inner wheel
turning tighter than the outer when Ackermann is on. Toggle Ackermann off in the panel and the
+0.25 STP multiplier drops out of the score.

---

## 🏁 The two modes

**Autonomous Race** — the car follows the painted midline through every checkpoint, in order,
against the stage clock. Off-track is judged at **30 % of the car's width** beyond the line.

**Race Challenge** — the checkpoints become cones sitting on the line. The route is recomputed
as a lateral detour that keeps `cone radius + half car width + 12 mm` of clearance, alternating
sides. Off-track is judged at **50 %**.

> 📐 Reality check from the geometry: on a 0.50 m track with a 0.20 m car, the detour needs
> **0.145 m of the 0.150 m lateral room available**. The challenge is at the edge of physically
> possible on this map — decide whether to enter it before you spend time on it.

Shared rule model: 3 attempts, 5-minute stage cap, 10-minute prep, best score kept, a violation
stops the car and relocates it to the last checkpoint with no clock stop, and a 4th violation
ends the attempt.

---

## 🧮 Scoring (guidebook BAB III.F)

```
S_AR  = (1 + max((T_stage − T_total)/T_stage, 0)) · cp
S_ARC = ((1 + max((T_stage − T_total)/T_stage, 0)) · cp) / (2 · Tcp)
Total = S_AR · (1 + STP + S_ARC)          STP = 0.25 for Ackermann

penalty seconds = coef · T_stage / Tcp     0.8 off-track · 0.5 missed checkpoint · 0.2 part detached
```

`T_total` already includes penalties, so a sloppy fast lap and a clean slow lap compete directly.
The HUD shows the live breakdown and the best score of the session.

⚠ The penalty formulas render broken in the PDF. They are read here as `coef × T_stage ÷ Tcp`.
With 8 checkpoints and a 300 s stage, one off-track costs **30 s** — that single assumption
drives the whole speed-versus-safety tradeoff. Confirm it at the Technical Meeting (8 Nov).

---

## 📷 Cam POV and the axle-referenced mount

`Cam POV` renders from a camera bolted to the chassis, so it is the view your IMX219 actually
gets — not a chase camera pretending to be one. The **Camera mount** panel places it relative to
an axle reference point:

| Anchor | Where it sits |
|---|---|
| Rear axle | local x = −wheelbase/2 (−125 mm) |
| Centre | between the axles |
| Front axle | local x = +wheelbase/2 (+125 mm) |

`Forward from anchor`, `Height above floor` and `Lateral` are metres relative to that point
(lateral `+` = the car's left); `Pitch down` and `Horizontal FOV` orient it. Defaults are
0.16 m high, 12° down, 62° hFOV — a Pi camera module v2 on the bulkhead. The anchor also drives
the **Orbit / chase pivot = axle anchor point** checkbox, so you can watch the orbit camera swing
round the front axle instead of the loop centre, and the `R` / `C` / `F` markers on the deck show
which point is which.

Why it matters for perception: at 0.16 m and 12° down, the ground enters the frame only about
**0.21 m** ahead and the full 0.50 m lane width only fits the frame from **~0.45 m** ahead.
Everything closer exits through the bottom or the sides. Raising the mount or its pitch buys
look-ahead; that is a real tradeoff on this car, not a simulation quirk.

---

## 🧪 Dataset capture — frames + ground truth for your own code

`Data capture → Record lap` drives a whole attempt headlessly at a fixed 1/fps in **sim time**
(so frames are evenly spaced regardless of your GPU or sim speed) and keeps, per frame:

| Output | Contents |
|---|---|
| `frames/000001.jpg` | what the mounted camera sees — physical paint, gates, cones and floor only. Racing line, trail, alternate path and label sprites are switched off during capture |
| `labels/000001.png` | segmentation map, pixel value = class id (0 background, 20 surface, 40 infield, 60 edge line, 80 centre dash, 120 cone, 140 gate, 160 start/finish) |
| `telemetry.csv` | one flat row per frame: pose, speed, commanded steer, radius, lane offset, heading error, distance to each painted line, clearance at each axle, checkpoint and cone state, running score |
| `annotations.jsonl` | per frame: camera intrinsics, projected lane polylines and object pixels, plus ego ground truth |
| `dataset.json` | mount, intrinsics, class map, sim settings, track metadata, coordinate conventions |
| `track/track_data.json` | the raw loop geometry |
| `code/baseline_line_follower.py` | loader + lane recovery + Stanley control + scoring, runs straight from the zip |
| `README_DATASET.md` | conventions, column glossary, and what the frames lie about |

`Export ZIP` writes it all as one archive — built in the page, no server, no dependencies.

The loader recovers the lane twice: from the label map (oracle) and from RGB with a brightness
threshold (a detector you should replace with your own). Verified against controlled synthetic
frames with known geometry: **0.4 mm** lateral and **0.03°** heading error at the fitted station,
which is what makes the residual on real captures a perception number rather than a math number.

```bash
python code/baseline_line_follower.py my-dataset.zip --detector label   # oracle
python code/baseline_line_follower.py my-dataset.zip --detector rgb     # your detector
```

Needs `numpy` and `opencv-python-headless`.

### What the real frames measured (120-frame race lap, 640x480, default mount)

Run through the loader with the geometry-matched truth from `track/track_data.json`:

| Metric | median | p90 | worst |
|---|---|---|---|
| lateral error vs the painted centreline | 0.023 m | 0.088 m | 1.85 m |
| heading error at the fit station | 7.9 deg | 24 deg | 63 deg |
| brightness detector vs label oracle | 0.0242 m | - | identical to within 1 mm |

Three conclusions, in order of how much they should change your plan:

1. **Segmentation is not the bottleneck.** A plain brightness threshold recovers the lane to
   within a millimetre of the oracle label map on these frames. Effort belongs in the control law
   and the mount, not in a cleverer line detector - at least until real floor glare is in play.
2. **Heading from a 0.16 m mount is a lookahead quantity.** The fit can only see the lane from
   about 0.6 m ahead, and on this map's 0.5-2 m corners the lane genuinely points 8-25 degrees off
   the car's heading there. A Stanley-style controller that wants heading *at the axle* will not
   get it: raise the mount, pitch it down more, or steer off a lookahead point.
3. **The lateral number is good but has tails.** Median 2.3 cm is comfortable against the 10 cm
   that costs a violation, but a handful of frames land metres out where the visible paint geometry
   becomes ambiguous mid-corner. That is why the loader prints median, p90 and a frame count over
   threshold rather than an MAE - a 2 % failure rate is twice a lap of trouble.

---

## 🎛 Panel controls

| Slider | Models | Default |
|---|---|---|
| Top speed | motor / gearing ceiling | 1.60 m/s |
| Lateral grip limit | tyre + floor friction; caps corner speed at `√(a·R)` | 6.0 m/s² |
| Line-follow error | vision noise, pushes the car toward the line | 0.020 m |
| Sim speed | wall-clock multiplier | 2.0× |
| Checkpoints | `Tcp`, also sets the penalty divisor | 8 |
| Obstacle avoidance width | how early the car starts its detour | 0.45 m |
| Stage limit | the 5-minute cap | 300 s |

Overlays: racing line, the *other* mode's path (for comparison), painted centre dashes, the
original map texture draped on the floor, the motion trail, the axle centre markers, and whether
your own car body appears in POV / capture frames.

The map overlay is blended **additively** on purpose: the committee's lintasan export is a
black-background CAD drawing (90 % of its pixels are near black), so painting it normally over the
dark arena looks like nothing happened. Additive blending lets the black paper add nothing and the
drawn lines glow through — and where their lines and our rebuilt ribbons coincide, you are seeing
the extraction confirmed by eye.

Capture settings: frame size (320×240 → 1280×720), rate 5–60 fps, frame cap, JPEG quality, and
whether to include label maps and annotations. 260 frames at 640×480 with labels lands around
20 MB in memory — trim the cap or the size for a longer run.

---

## 📁 Files

| File | Role |
|---|---|
| `index.html` | the app — all logic and styling |
| `three.min.js` | bundled renderer (r147, MIT) |
| `track_data.js` | geometry the app loads |
| `zipStore.js` | dependency-free store-method ZIP writer used by the exporter (same file runs under Node for its self-test) |
| `dataset_assets.js` | the README and Python loader embedded in every export |
| `track_floor_data.js` | the map texture inlined as a data: URL (see below) |
| `tools/inline_texture.py` | regenerates that file from `track_floor.jpg` |
| `track_floor.jpg` | optional floor texture |
| `track_data.json` | same geometry, machine-readable |
| `sanity_overlay.png` | the fit, drawn over the source map |
| `extract_track.py` | rebuilds the data from the map |

### Rebuilding after a map revision

```bash
# 1. render the committee PDF to PNG (200 dpi) one folder up
python -c "import pymupdf;d=pymupdf.open('LINTASAN.pdf');d[0].get_pixmap(dpi=200).save('../track_map_200dpi.png')"
# 2. re-extract, then regenerate the JS wrapper
python extract_track.py
python -c "import json;d=json.load(open('track_data.json'));open('track_data.js','w').write('window.TRACK = '+json.dumps(d)+';\n')"
```

Needs `opencv-python`, `numpy`, `pymupdf`. The scale is re-anchored to 0.50 m width automatically.

---

## 🔬 Assumptions, stated plainly

- Checkpoint positions are **not on the map** (the guidebook says a checkpoint may carry no mark
  at all). They are placed evenly along the centreline and are adjustable.
- The start/finish line is assumed at the top of the loop; the map carries no marker.
- Car dimensions are a representative 1:10 chassis, not your chosen platform — swap the `CAR`
  object at the top of the script block in `index.html` to model the real one.
- Motion is a kinematic path follower with a friction-limited speed profile. It is not a tyre or
  suspension model, and it does not simulate the sensor stack.
- The camera mount defaults (0.16 m, 12° down, 62° hFOV) are a *guess* at where a Pi camera
  module v2 ends up on this chassis. The projection maths is exact, so once you measure your real
  height, pitch and pixel rows, typing them into the panel makes the frames match your camera —
  and until then, treat the capture geometry as a template, not a calibration.
- Frames are perfect renders: no motion blur, rolling shutter, exposure drift, glare, vibration or
  tyre slip. A detector tuned here will look better than it behaves on the hall floor.
- The map is a *sample*. Treat every number here as provisional until the Technical Meeting.

*Built from `GUIDEBOOK PESERTA MINI AUTONOMOUS VEHICLE KSTI 2026.pdf` and the lintasan PDF linked
from its page 6. Robotics Competition at KSTI 2026, Institut Teknologi Sepuluh Nopember,
Hall Robotika ITS, 12 November 2026.*
