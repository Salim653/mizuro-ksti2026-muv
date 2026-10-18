# 📋 KSTI 2026 — Mini Autonomous Vehicle · Guidebook Recap

> ⚡ **TL;DR** — Closed-loop line track, 50 cm wide, 5-minute runs, 3 attempts, best score kept. IR line followers are **banned**, so lane detection must be camera-based. Score is `(1 + time_remaining/300) × checkpoints`, and the final total multiplies that by `(1.25 if Ackerman steering) × (1 + challenge_score)`. **Checkpoints are the dominant term** — a lap that skips a 'point costs far more than a slow lap that hits all of them.
>
>  **Next action** — Confirm the sensor stack against the ban list (see §8), then measure the actual track length from `track_map_200dpi.png` to set the speed target.

---

## 📅 1 · Event facts

| Item | Value |
|---|---|
| Name | Mini Autonomous Vehicle — Robotics Competition at KSTI 2026 |
| Organizer | Institut Teknologi Sepuluh Nopember (ITS), Surabaya |
| Date & venue | **12 November 2026**, Hall Robotika ITS (offline) |
| Technical Meeting | **8 November 2026** — registration confirmation + group split announced here |
| Eligibility | Mahasiswa (KTM required); team max **5 people**, 1 team leader |
| Team/institution limit | Not limited — multiple teams per campus allowed |
| Entry fee | Rp150,000 (batch 1) / Rp175,000 (batch 2) · **non-refundable** |
| Registration | https://its-robotics.com/ → KSTI 2026 → Mini Autonomous Vehicle |
| Verification SLA | Max 2×24 h after submission |
| Prize | 1st gold + Rp4.000.000 · 2nd silver + Rp3.000.000 · 3rd bronze + Rp2.000.000 |
| Contact | Nindita +62 896-4920-8124 · Yuke +62 857-5577-3985 |

⚠️ The prize amounts are written as "Rp4.00.000,00" in the PDF. Read as Rp4.000.000 — verify at the technical meeting.

---

## 🤖 2 · Robot rules

**Platform**
- ⛔ Fuel-powered cars banned → **electric only**, scale 1:10.
- ✅ Must have at least one steering axle.
- 🔴 **Physical `emergency stop` button** (easily reachable) **and a separate `start` button** that launches navigation. Both are checked rules.
- One car / one platform per team. Platform swap = banned; component repair/modification = allowed.

**Power** — No limit on battery count or capacity.

**Drive & steering**
- 2WD or 4WD allowed.
- ⛔ No omni, mecanum, or any wheel config that lets the robot move in all directions.
- ✅ Differential drive system is **required**.
- Steering system otherwise free.
- 💎 **Ackerman steering earns STP = 0.25** (single-axle or two-axle). See §5 — it is a *multiplier*, not a bonus.

**Dimensions (mm)**

| Dimension | Min | Max |
|---|---|---|
| Track width (lebar jejak roda) | 130 | 350 |
| Wheel base | 160 | 550 |
| Height | — | 450 |
| Width (incl. all mounted parts) | — | 400 |
| Length (incl. all mounted parts) | — | 600 |

**Sensors**
- ⛔ **Infrared and/or line-follower sensors are forbidden.**
- ✅ Any other sensor that is *not* a direct line-tracing sensor is allowed.
- 📡 **All communication banned during the run** — wireless *and* wired. No RC fallback, no telemetry tether, no laptop link. Fully standalone once the start button is pressed.

---

## 🛣️ 3 · Track

- Width **50 cm ± 10%**, and **not constant** along the lap.
- Minimum turning radius **1.5 m ± 10%** (outer radius).
- Marking: **two solid straight edge lines** (white **or** black) + **one dashed centre line** in the same colour as the edges.
- Mark width ≥ 1 cm · gap between dashes ≥ 5 cm.
- Mark colour is chosen to suit the arena floor → **be ready for both white-on-dark and black-on-light**.
- Checkpoints placed along the track, shown **green**; start/finish in a colour different from the road markings (illustrated as red, **not guaranteed**). A checkpoint may carry **no marking at all** — the map is demonstration only.
- 📎 Local assets in this folder: `track_map_200dpi.png` (also `_small.png`, `track_map_raw.bin`) — a closed loop with an S-curved long-straight layout, dark floor, white edges, dashed centreline, "KSTI 26" logo in the infield. No start line or checkpoints drawn. See §7.

---

## 🏁 4 · Format & run flow

Two stages: **Eliminasi** (must clear a minimum score to advance) → **Final**. The **Challenge** is optional and only run in the final.

Per attempt, both formats share this flow:

1. Be at the start area **5 min before** your slot — late = attempt forfeited (`gugur`).
2. Called → **10 min preparation**: small adjustments, sensor calibration, strategy review, repairs.
3. Prep ends → **5 min attempt window**. Place the car on the start line, press start.
4. 🚫 **After pressing start, nobody may touch the robot.** Any contact ends the attempt.
5. On a violation: referee calls it → press `emergency`, car is moved back to the **last checkpoint**, press `start` again, **no time pause**.
6. Max **3 violations per attempt** — the 4th ends the attempt.
7. Part detaching/falling off = violation **and the run ends immediately**.
8. Not finished in 5 min → stop the car; score counts only the progress reached.
9. **Best of 3 attempts** is the final result.

| | Autonomous Race | AR Challenge |
|---|---|---|
| Checkpoint | Green mark on track, pass in order | The checkpoint **is an obstacle** to avoid; avoiding it = passing it |
| Out-of-track threshold | **30%** of the car outside | **50%** of the car outside |
| Availability | All rounds | Final only, optional |

---

## 🧮 5 · Scoring

**Autonomous Race**

```
S_AR = (1 + max{(T_stage − T_total)/T_stage, 0}) × cp
```
`T_stage` = 300 s · `T_total` = elapsed time **including penalties** · `cp` = checkpoints passed.

**AR Challenge** — same shape, then normalised:

```
S_ARC = [(1 + max{(T_stage − T_total)/T_stage, 0}) × cp] / (2 × Tcp)
```
`Tcp` = total checkpoints on track. Max contribution ≈ **0.5**.

**Final total**

```
Total = S_AR × (1 + STP + S_ARC)          STP = 0.25 if Ackerman
```

**Penalty time added to `T_total`** (all expressed as a fraction of the stage time, divided by the checkpoint count):

| Violation | Added time |
|---|---|
| Missed checkpoint | `0.5 × T_stage / N_all_checkpoints` |
| Part detached | `0.2 × T_stage / N_all_checkpoints` |
| Out of track | `0.8 × T_stage / N_all_checkpoints` |

📐 **Why the maths matters** — with, say, 10 checkpoints, `T_stage/N_cp` = 30 s, so one off-track event costs **+24 s** and one missed checkpoint costs **+15 s** *and* drops `cp` by 1. Working the numbers:

| Scenario | cp | T_total | S_AR |
|---|---|---|---|
| Clean fast lap | 10 | 150 s | 15.0 |
| Clean slow lap | 10 | 250 s | 11.7 |
| Fast but one off-track | 10 | 174 s | 14.2 |
| Fast but one checkpoint missed | 9 | 165 s | 12.5 |
| Fast but 3 off-track (limit) | 10 | 222 s | 13.1 |

➡️ **Speed is worth ~1.3 points per 30 s saved; a single missed checkpoint is worth ~2.5 points lost.** Reliability at moderate speed beats raw speed. That is the design brief.

💎 Ackerman is the cheapest win available: it multiplies the whole score by 1.25 with no per-run effort.

---

## 🚩 6 · Judges, protests, fine print

- Judges/referees appointed by committee; decisions **final and absolute**.
- Protests: only from a team member, politely, **immediately after** the match and before the next one starts; never during a run. ⛔ Protests about violation calls or scoring are **not accepted** — only suspected breaches of robot rules or procedure.
- Committee may change these rules; changes announced via the registration platform and official socials. Registering = accepting the guidebook.
- Committee not liable for lost/damaged participant property.

---

## 🐞 7 · Guidebook inconsistencies worth raising at the Technical Meeting

1. 🏷️ Prize section header says **"Humanoid Penalty Kick"** — copy-paste from another category.
2. 📅 §II.A.7 says participants must obey the rules of **"KSTI 2025"**.
3. 💰 Prize figures written "Rp4.00.000,00" (ambiguous digit grouping).
4. 📏 Out-of-track threshold differs between formats (30% vs 50%) with no stated reason — confirm which applies in the final.
5. ❓ The **minimum qualifying score** to pass elimination is referenced but never stated.
6. ❓ `N_all_checkpoints` and `Tcp` are undefined numerically — the whole penalty scale depends on them.
7. 🎨 Checkpoint/start-line colours are "demonstration only" → ask whether the final palette is fixed before the event.
8. 🖥️ "Tombol lain untuk memulai navigasi" + "no touching after start" implies the start button must be reachable without opening the chassis — ask how they check it.

---

## 👁️ 8 · Perception implications (why the camera question matters)

The IR/line-follower ban removes the cheap solution entirely: **lane keeping must come from an RGB camera + software**, since only non-line-tracing sensors are permitted.

- 🎯 **Lane keeping needs no depth.** A downward camera gives the centreline and edges as geometry in the image plane; fit the lane, steer. Stereo/depth buys nothing here.
- 📐 **Resolution budget:** the marks are ≥ 1 cm wide on a 50 cm track. Fitting ~70 cm across the lane onto a 640-px sensor means ≈ 0.9 mm/px — the 1 cm mark spans ~11 px, which is plenty. A 1/4" lens at 20–40 cm mounting height covers it comfortably.
- 🚧 **Where depth *does* earn its place:** the Challenge, where the checkpoint is a physical obstacle to avoid. A flat green tape mark and a solid object on the lane look identical to a single top-down camera. Distinguishing them needs either height above the floor, a side/front view, or two viewpoints.
- 📷 **The practical two-camera split** is not stereo — it is **one forward-tilted camera** for long-range path and obstacle discovery, plus **one downward camera** for precise close-range lane centring where the forward view loses resolution. Fixed focus is fine for both, because each has a fixed working distance and a deep DOF at that scale.
- 🗺️ **The map is known.** `track_map_200dpi.png` + `arena-3d/track_data.json` mean the course geometry can be pre-loaded, so odometry/IMU can fuse with vision for checkpoint counting and for recovering after being reset to the last checkpoint.
- 💡 **Cheapest obstacle discriminator:** a forward camera plus a known track model answers "is this blob on my lane, and does it sit above the floor plane?" without needing metric stereo at all.

---

## 📎 9 · Files in this directory

| File | What it is |
|---|---|
| `GUIDEBOOK PESERTA MINI AUTONOMOUS VEHICLE KSTI 2026.pdf` | Source document, 11 pages (Bab I–IV). Page 1 = cover, no text layer. |
| `track_map_200dpi.png` / `track_map_small.png` / `track_map_raw.bin` | Example track map at three resolutions. |
| `arena-3d/` | `extract_track.py`, `track_data.json` / `.js`, `track_floor.jpg`, `sanity_overlay.png`, `three.min.js` — 3D arena reconstruction from the map. |

---

## ✅ 10 · Action checklist

| # | Task | Owner | Due |
|---|---|---|---|
| 1 | Register on its-robotics.com, upload KTM + payment proof | Team leader | before batch 2 closes |
| 2 | Confirm Ackerman chassis decision (worth ×1.25) | Mech | before build freeze |
| 3 | Bench-test the vision stack against the IR/line-follower ban wording | SW | before 8 Nov |
| 4 | Bring the §7 question list to the Technical Meeting | Team leader | 8 Nov 2026 |
| 5 | Calibrate for both white-on-dark and black-on-light marking | SW | before Test Day |
| 6 | Set a lap-time target from measured track length ÷ 5 min minus margin | Controls | after map measurement |
