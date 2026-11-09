# 2.483 s run — verified evidence

Run `MODPHYS14_VIDEO_t2491_20261106_01`, 2026-11-06, 960×600 / 60 fps recording.
The video itself is not stored in this repository.

| Field | Value |
| --- | --- |
| First gate → last gate (sim) | 2.483194 s |
| Wall-clock estimate | 2.481984 s |
| Gates | 12/12 ordered forward crossings |
| Collision messages in the timed interval | 0 |
| Minimum centre margin | 0.319 m (gate 4) |
| Physics | `official_physics: false`, max angular speed 14 rad/s |
| Valid measured run | `true` |

Files:

- `gate_race_result.json` — final judgement produced by `run_race.py`.
- `gate_timer.json` — live timer state at freeze (gate numbers, start/finish sim
  time, credited crossings, collision counts).
- `summary.json` — per-gate crossing time and horizontal/vertical offsets,
  wall-clock telemetry, terminal hold and data-quality checks.

The per-gate offsets can be recomputed with the timer rule: margin =
0.75 − max(|horizontal|, |vertical|); the minimum over the 12 gates is 0.319 m.

The recording carries a red `MODIFIED PHYSICS: max rate 14 rad/s` banner. Results
from this configuration are not comparable with official-physics runs; see the
top-level README.
