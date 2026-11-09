Native WSL2 / WSLg runtime adapter (no algorithm or simulator modifications)
=========================================================================

Place this runtime/ directory under /home/labuser/lab4_gpu_20261023/ alongside
the already-extracted portable payload (MANIFEST.json, ros_ws/, scripts/,
optimized/, original_parameters/). Build that ros_ws against ROS Noetic first.
Keep the checksummed official ZIP; extract its lab4/ into simulator/lab4/.
The ELF player must have its executable permission. Do not change asset bytes.

The adapter is Python 3.8-compatible and invokes its own env.sh automatically:

  python3 -B /home/labuser/lab4_gpu_20261023/runtime/run_trial_gpu.py gpu_take1 \
    --root /home/labuser/lab4_gpu_20261023 \
    --simulator-zip /mnt/c/Users/<user>/lab4_gpu_transfer_20261023/lab4_simulator.zip

Default: 960x600, Very Low, native x86_64, -force-glcore, -noaudio, 12 simulated
seconds after reference begins, 105-second total run watchdog (max configurable
110). --sim-seconds 18, --rviz and --capture are optional; without --capture,
the core trial still produces no video. DISPLAY and the validated WSLg
Mesa environment must already be available to this SSH/local session. No Xvfb,
BOX64, Linux NVIDIA driver installation or software renderer is used.

Optional platform compatibility switches (neither changes the defaults):
--glcore33 requests Unity -force-glcore33 instead of automatic -force-glcore.
--single-threaded-rendering appends -force-gfx-direct as a rendering-thread
diagnostic only; it is NOT a confirmed successful workaround or a claim that
recording/racing succeeds. These options do not select a different scene,
alter physics, or replace the renderer and same-run video checks. A platform
probe or a passing unit test is not a completed recorded trial.

env.sh affects only the child shell/process; it never edits shell startup or
system files. It resets inherited ROS/Python workspace paths, sources only
/opt/ros/noetic and this root's devel, and adds the copied TESSE interface.

Isolation: refuse existing TCP11311, TESSE TCP/UDP9000/9001/9002/9004/9005/9006/
9007, or an existing lab4.x86_64. Never stop those foreign processes. A fresh
loopback master uses the original absolute topics. Cleanup signals only the
process groups created by this invocation; Ctrl-C and hard wall timeout clean
up scoped processes. If a prior process remains, stop and inspect rather than
using broad kill commands. Every run name must be fresh.

The temporary IPv4/IPv6 TCP port probes use SO_REUSEADDR to tolerate eligible
closed connections in TIME_WAIT. UDP probes do not use it; no probe uses
SO_REUSEPORT. Active TCP listeners and bound UDP sockets still cause failure.
This is a point-in-time check, not a port reservation or permission to share
services; some TIME_WAIT states may still need to expire naturally. Roslaunch
uses --sigint-timeout=0.5 and --sigterm-timeout=0.5 so its own separately grouped
bridge nodes can be reaped within the adapter's 3-second SIGINT grace, without
expanding cleanup to foreign processes.

After its own master becomes ready, the adapter sets /use_sim_time=true through
a logged, bounded rosparam subprocess BEFORE rospy.init_node. It rejects an
observer that nevertheless initializes in wall-clock mode, then waits for the
bridge's first real /clock message with a finite timeout. This prevents mixing
a wall-clock reference-start observation with later simulator timestamps; it
does not rescale or replace the simulator clock.

Integrity: fixed CSV e1883baf87bba068a60445275db3e70d00999303eac137dcc0b0d83486c7dad2;
official ZIP 27e4fe0d2905eb82730dfb11b6eacb8a6adb12d4741ede3ee4e6f65615504915.
All payload regular-file hashes and every expanded ZIP asset are checked.
Source/CSV hashes are rechecked after the run. Built native-node and adapter
hashes, command argv/PIDs, original profile/params, initial metadata/odometry,
initial and final ROS params, clock-wall samples, bag, renderer and per-process
logs are recorded in runs/<name>/. Original algorithms are not patched.

Only final_balanced494 is accepted: gains36/12/60/4, J=.1/.2/.1, signed and free
heading, pure-reference rate feedforward, tau=.02, omega limit6, no preview.
The original Unity-coordinate Respawn/Reposition sequence is preserved. Bag
and clock recording start before that sequence, so reset/start contacts are
retained rather than filtered. Bridge collision publishing is enabled; its
enable_collision=true retains the fresh official player's default collision
setting (the legacy bridge sends a collider command only when disabling it).
Clock, scene, mass, forces, gate input, collision criteria and timescale are
not altered.

Optional same-run real-window video
----------------------------------
Add --capture to the command above, using a fresh run name, for example:

  python3 -B /home/labuser/lab4_gpu_20261023/runtime/run_trial_gpu.py gpu_video1 \
    --root /home/labuser/lab4_gpu_20261023 \
    --simulator-zip /mnt/c/Users/<user>/lab4_gpu_transfer_20261023/lab4_simulator.zip --capture

This invokes runtime/capture_x11_window.py against the unique mapped Unity
X11 window titled tesse_vnav_scenes, not the whole desktop or an RViz/trajectory
reconstruction. Capture requires the window to stay mapped and unchanged in
size; dimensions must be even and no larger than 960x600. X11/XWayland access,
libX11, ffmpeg and ffprobe must be available. Hardware GPU rendering is still
checked independently from Unity's Renderer log, not inferred from the video.

The reference starts only after the first real XGetImage frame is saved and
capture.ready.json is validated. The target sampling rate is 60 FPS; actual
monotonic timestamps, measured rate and missed schedule slots are retained.
The requested capture duration is sim-seconds+3 WALL seconds (15 or 21), capped
at 30; this is not a replacement simulation clock or a guarantee that a slow
run fits in the clip. After collection the runner keeps reference/controller,
bag and clock alive while waiting up to 25 wall seconds for normal capture,
encoding and validation, within the existing total watchdog. An insufficient
startup/encoding budget or any capture error is recorded as a real failure;
do not disable the watchdog or relabel an unverified clip as a successful run.

Frames are real PPM samples. VFR encoding uses their monotonic sample times
mapped to the mtimes of these NEW frame files, then only shifts the video PTS
origin to zero. It does not compress gaps, speed up flight, interpolate motion
or change system/ROS time. Exactly one disclosed duplicate of the last frame
marks the measured capture-end boundary. frames.ffconcat is diagnostic only;
Focal's actual encoding path uses image2 -ts_from_file 2.
For MP4, -use_editlist 0 prevents Focal ffmpeg's edit-list endpoint from causing
the final boundary sentinel to be excluded by default decoding. This preserves
the existing samples and timestamps; it does not add a new measured frame,
extend the captured flight, or relax the strict frame-count/PTS/duration checks.

Artifacts: runs/<name>/scene.mp4, capture.log, video_capture.json and
scene.mp4.capture/{capture_report.json,frames.jsonl,ffprobe.json,ffmpeg.log,
frames.ffconcat,frames/*.ppm}. Raw PPM frames can occupy gigabytes and stay in
WSL: DO NOT copy them back to the Mac or bulk-sync the .capture directory.
The helper checks both the WSL destination and /mnt/c backing-drive free space
before/during capture. Transfer only the MP4 and selected small evidence files
when needed; retaining raw frames is intentional and no automatic deletion is
performed here.

run_status.json.video_recorded becomes true only after helper exit0, a complete
report, a nonempty MP4, duration_verified=true and frame_timestamps_verified=true.
Validation requires encoded duration within 0.10s of the real monotonic span,
decoded frame count matching samples plus the one boundary frame, and maximum
frame timestamp error <=0.010s. Use the report's actual rate/span, not a player
or container FPS label. A video is same-run visual evidence, not an official
race timer, proof of every collision, or a whole-body clearance certificate.

Results: the unchanged sprint_evaluate.py and audit_race.py create summary.json.
run_status.json preserves failures even if collection/audit fails or times out.
Exit0 means the OBSERVED screen had 12 ordered planes, <5 simulated seconds,
sampled terminal hold, no data errors and zero reported collision messages
between the first and last gate. Start/reset contacts remain in the report.
Exit1 means an incomplete/not-met screen; exit2 operational/audit failure;
exit124 hard wall timeout; exit130 user interruption. Raw audit_exit is also
retained (its original exit0 only means complete valid plane sequence).
Zero collision messages do not prove reporting completeness or collision-free
flight. Neither a screen nor GPU renderer verification is an official race
score or a whole-body swept-clearance certificate; competition_pass stays null.

Standard-library tests on Mac/Linux, from the payload root containing runtime/
(no ROS import, network connection or process launch):
  python3 -B -m unittest discover -s runtime -p 'test_run_trial_gpu.py' -v
  python3 -B -m unittest discover -s runtime/tests -p 'test_*.py' -v
The current core and runtime/tests suites total 100 tests. These are contract
and pure-function tests, not evidence that any pending GPU/video run succeeded.
