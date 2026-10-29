#!/usr/bin/env python3
"""Replay kontrol Fase 3 di sim: Stanley + speed profile against track geometry.

    .venv/bin/python tools/replay_control.py [--track PATH] [--dt 0.05]

Membaca track_data.json (geometri loop dari arena-3d), mensimulasikan
robot yang mengikuti centreline dengan Stanley controller, dan melaporkan:
- max |lateral error|, max |heading error| sepanjang lap
- max akselerasi lateral (harus <= max_lateral_accel)
- waktu lap total + kecepatan rata-rata

Model: arc-length based. State = (s, d, psi) — posisi arc, offset
lateral, heading error relatif tangent centreline. Dinamika bicycle:
    ds/dt = v * cos(psi)
    dd/dt = v * sin(psi)
    dpsi/dt = v * tan(delta) / L

TIDAK menyentuh motor, TIDAK menyentuh serial. Murni matematika.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def load_track(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    for key in ("centre", "radius", "s"):
        if key not in data:
            raise SystemExit(f"track_data.json tidak punya '{key}'")
    return data


def _interp(sorted_s: list[float], values: list[float], query: float) -> float:
    """Interpolasi linear dari array (s, value) yang sudah terurut."""
    n = len(sorted_s)
    if query <= sorted_s[0]:
        return values[0]
    if query >= sorted_s[-1]:
        return values[-1]
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if sorted_s[mid] <= query:
            lo = mid
        else:
            hi = mid
    span = sorted_s[hi] - sorted_s[lo]
    if span <= 0:
        return values[lo]
    t = (query - sorted_s[lo]) / span
    return values[lo] + t * (values[hi] - values[lo])


def simulate(track: dict, *, dt: float = 0.05,
             wheelbase_m: float = 0.15,
             max_lateral_accel: float = 3.0,
             max_speed: float = 0.8, min_speed: float = 0.15,
             heading_gain: float = 0.8, lateral_gain: float = 1.2,
             softening: float = 0.1,
             max_accel: float = 0.5, max_decel: float = 1.0,
             speed_lookahead_m: float = 1.0,
             init_offset_m: float = 0.0,
             init_heading_deg: float = 0.0) -> dict:
    """Simulasi closed-loop Stanley against centreline (arc-length model)."""
    s_arr = track["s"]
    radius = track["radius"]
    track_len = s_arr[-1]

    # State awal.
    s = 0.0
    d = init_offset_m
    psi = math.radians(init_heading_deg)
    speed = min_speed

    max_lat_err = abs(d)
    max_head_err = abs(psi)
    max_lat_accel = 0.0
    steps = 0
    max_steps = int(300.0 / dt)
    warmup_steps = int(2.0 / dt)           # skip 2s awal (transient)
    ss_lat_err = 0.0                        # steady-state (setelah warmup)
    ss_head_err = 0.0

    while steps < max_steps:
        # Stanley steering. Konvensi controller: lane_error > 0 = lajur di
        # KANAN robot. Simulasi: d > 0 = robot di KANAN centreline (lajur
        # di KIRI robot). Jadi lane_error = -d, heading_error = -psi.
        cross = math.atan(lateral_gain * (-d) / (speed + softening))
        steer = heading_gain * (-psi) + cross
        steer = max(-0.55, min(0.55, steer))

        # Speed dari kurvatur di DEPAN (lookahead arc-length).
        s_ahead = s + speed * speed_lookahead_m
        if s_ahead >= track_len:
            s_ahead -= track_len          # loop tertutup
        r_ahead = _interp(s_arr, radius, s_ahead)
        kappa = 1.0 / max(r_ahead, 1.5) if r_ahead > 0 else 0.0
        if kappa > 1e-6:
            v_target = math.sqrt(max_lateral_accel / kappa)
        else:
            v_target = max_speed
        v_target = max(min_speed, min(max_speed, v_target))

        # Smooth akselerasi.
        dv = v_target - speed
        max_dv = (max_accel if dv >= 0 else max_decel) * dt
        speed += max(-max_dv, min(max_dv, dv))
        speed = max(0.0, min(max_speed, speed))

        # Akselerasi lateral aktual.
        lat_accel = speed * speed * abs(kappa)
        max_lat_accel = max(max_lat_accel, lat_accel)

        # Update state (bicycle model).
        ds = speed * math.cos(psi) * dt
        dd = speed * math.sin(psi) * dt
        dpsi = speed * math.tan(steer) / wheelbase_m * dt
        s += ds
        d += dd
        psi += dpsi

        max_lat_err = max(max_lat_err, abs(d))
        max_head_err = max(max_head_err, abs(psi))
        if steps >= warmup_steps:
            ss_lat_err = max(ss_lat_err, abs(d))
            ss_head_err = max(ss_head_err, abs(psi))
        steps += 1

        # Selesai setelah 1 lap penuh.
        if s >= track_len:
            break

    return {
        "steps": steps,
        "time_s": steps * dt,
        "distance_m": s,
        "avg_speed_mps": s / (steps * dt) if steps else 0.0,
        "max_lateral_error_mm": max_lat_err * 1000.0,
        "max_heading_error_deg": math.degrees(max_head_err),
        "ss_lateral_error_mm": ss_lat_err * 1000.0,
        "ss_heading_error_deg": math.degrees(ss_head_err),
        "max_lateral_accel": max_lat_accel,
        "completed": s >= track_len,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay Stanley controller against track geometry")
    parser.add_argument("--track", type=Path,
                        default=Path("../arena-3d/track_data.json"))
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--offset", type=float, default=0.0,
                        help="offset lateral awal (m)")
    parser.add_argument("--heading", type=float, default=0.0,
                        help="heading error awal (derajat)")
    args = parser.parse_args()

    if not args.track.is_file():
        raise SystemExit(f"track tidak ada: {args.track}")

    track = load_track(args.track)
    print(f"track: {args.track} ({len(track['centre'])} titik, "
          f"{track['s'][-1]:.1f} m)")

    # Skenario 1: start centred, aligned.
    r = simulate(track, dt=args.dt)
    _report("start centred", r)

    # Skenario 2: start dengan offset + heading error.
    r2 = simulate(track, dt=args.dt, init_offset_m=args.offset,
                  init_heading_deg=args.heading)
    _report(f"start offset={args.offset}m heading={args.heading}deg", r2)


def _report(label: str, r: dict) -> None:
    print(f"\n--- {label} ---")
    print(f"waktu simulasi   : {r['time_s']:.1f} s"
          f"{' (LAP SELESAI)' if r['completed'] else ' (TIMEOUT)'}")
    print(f"jarak tempuh     : {r['distance_m']:.1f} m")
    print(f"kecepatan rata   : {r['avg_speed_mps']:.2f} m/s")
    print(f"max lat error    : {r['max_lateral_error_mm']:.1f} mm "
          f"(transient, termasuk kondisi awal)")
    print(f"max heading err  : {r['max_heading_error_deg']:.2f} deg "
          f"(transient)")
    print(f"SS lat error     : {r['ss_lateral_error_mm']:.1f} mm "
          f"(steady-state, setelah 2s)")
    print(f"SS heading err   : {r['ss_heading_error_deg']:.2f} deg "
          f"(steady-state)")
    print(f"max lat accel    : {r['max_lateral_accel']:.2f} m/s^2")
    # Lolos = lap selesai + steady-state converge + a_lat dalam batas.
    ok = (r["completed"]
          and r["ss_lateral_error_mm"] < 100.0
          and r["ss_heading_error_deg"] < 5.0
          and r["max_lateral_accel"] <= 3.0 + 0.01)
    print(f"-> {'LOLOS' if ok else 'PERLU TUNING'}")


if __name__ == "__main__":
    main()
