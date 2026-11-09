# LAB4 racing — 2.483 s round (source snapshot)

This folder holds the ROS 1 code behind the LAB4 race round that reached a
gate-to-gate time of **2.483 s** in simulation. It is the source snapshot of the
run `MODPHYS14_VIDEO_t2491_20261106_01`, not a standalone catkin workspace.

> **Physics disclosure.** The 2.483 s result was flown with one simulator field
> changed: Unity `PhysicsManager.m_DefaultMaxAngularSpeed`, from 7.0 to
> 14.0 rad/s (4 bytes, file offset 25008 in `lab4_Data/globalgamemanagers`).
> This is a change to the official simulator physics. The result is **not
> comparable** with official-physics runs and must be labelled as such wherever
> it is reported. The official-physics best in this project is 3.195 s. The
> patched asset is not committed; `race_20261029/speed_optimization_20261105/
> modified_physics/apply_maxang14_patch.py` rebuilds it from your own copy of
> the official file and verifies both SHA-256 values.

## Results

All times are first gate to last gate, simulation seconds, measured from the
actual `/current_state` crossings (not scheduled trajectory time).

| Round | Physics | Time (s) | Min centre margin (m) |
| --- | --- | --- | --- |
| Prehover baseline | official (7 rad/s) | 3.408 | 0.053 |
| 20 ms horizontal attitude preview | official | 3.260 | 0.072 |
| Closed-loop surrogate search | official | 3.195 | 0.289 |
| Surrogate + 14 rad/s, timing 1.431 | 14 rad/s | 2.604 | 0.202 |
| Surrogate + 14 rad/s, timing 1.455 | 14 rad/s | **2.483** | 0.319 |

Every listed run passed the same checks: 12 ordered forward crossings inside the
±0.75 m transparent square, zero collision messages between the first and last
gate, measured stable hover release, and terminal hold.

## Layout

```
race_20261029/                 runner, timer, capture, prehover, planner and search
  run_race.py                  main entry: starts ROS, simulator, bridge, reference,
                               controller, timer and capture; writes gate_race_result.json
  gate_timer.py                first-gate-to-last-gate judge (+ gate_geometry_evidence.json)
  capture_with_timer.py        HUD overlay and frame-accurate recording
  prehover_reference.py        5 s climb + hover, release only after measured stability
  attitude_optimization_20261030/  20 ms attitude preview publisher and profiles
  speed_optimization_20261105/     closed-loop surrogate, ES search, candidates
  route_optimization_20261030/     C4 re-solve and crossing audit tools
  controller_prehover/         ROS C++ geometric controller (source)
runtime/                       X11 capture and GPU trial adapters
竞速冲刺5秒/                    original 14-waypoint route and reference planner base
results_2p483/                 verified evidence of the 2.483 s run
```

## How it was run

The code expects the deployed workspace root used for this project
(`/home/labuser/lab4_gpu_20261023`) with ROS Noetic, the official TESSE
simulator, `ros_ws/` built and `scripts/plan_reference.py` in place. Overlay
`race_20261029/` and `runtime/` onto that root, rebuild the controller package,
then:

```bash
cd /home/labuser/lab4_gpu_20261023 && MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
python3 -B race_20261029/run_race.py demo2483_$(date +%H%M%S) \
  --root /home/labuser/lab4_gpu_20261023 \
  --simulator-zip /mnt/c/Users/<user>/lab4_gpu_transfer_20261023/lab4_simulator.zip --glcore33 \
  --plan-family hover --candidate-set speed_optimization_20261105/surrogate_modphys14c_candidates \
  --speed-factor 1.455 --omega-limit 10 --ff-tau 0.01 --control-profile modphys14_kr90 \
  --modified-physics 14 --timer-window --capture --wall-timeout 110
```

One run takes about two minutes and writes `runs/<name>/` with `scene.mp4`,
`gate_race_result.json` and the full evidence set. `--modified-physics 14`
verifies the deployed asset is the official copy, swaps in the patched copy for
the run, and restores and re-verifies the official file afterwards.

Controller rebuild:

```bash
cp -r race_20261029/controller_prehover/* ros_ws/src/controller_prehover/ \
  && source /opt/ros/noetic/setup.bash && cd ros_ws \
  && catkin_make -j8 -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
       --only-pkg-with-deps controller_prehover
```

## Not included

- Race videos and rosbags (large binaries); the verified run evidence is in `results_2p483/`.
- The patched `globalgamemanagers` asset — use the patch script described above.
- Local remote-deployment helpers (SSH/IP specific), which are machine-local by nature.
- The full offline test suite of the working tree; `runtime/test_run_trial_gpu.py`
  and `race_20261029/controller_prehover/tests/` are included.

## Provenance

Derived from `lab4/code/` (the D3 starter implementation) and `lab4/codeforRacing/`
(the earlier fast trajectory package) in this repository. Route offsets and segment
timings come from `speed_optimization_20261105/es_search_20261105_MODPHYS_cap14_v4.json`;
the run itself is `surrogate_modphys14c_candidates/modphys14c_t2491.{csv,json}`.
Absolute paths in the evidence files were rewritten to repository-relative paths
and host addresses were removed before publication.
