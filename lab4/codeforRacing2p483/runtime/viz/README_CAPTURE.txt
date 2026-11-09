Optional original LAB4 RViz source/configuration, exact-byte copies.

Files and SHA256:
  lab4_viz.py (2832 bytes)
  a0f6cdbfead8ebba87b6ae00aca0e7ebb9dcf9d84f5ca0975652211f11939833
  lab4_sbs.rviz (5524 bytes)
  7034e671b41c1dcc46b2f1e7f40d00290078913b9db25e056698accf5823d37e

Sources relative to the project root:
  GitHub仓库整理/vnav-labs-ros1/lab4/code/scripts/lab4_viz.py
  GitHub仓库整理/vnav-labs-ros1/lab4/code/rviz/lab4_sbs.rviz
No algorithm, marker geometry or RViz configuration is rewritten.

Runtime requirements in addition to the core portable payload:
  ros-noetic-rviz ros-noetic-visualization-msgs
  rospy, geometry_msgs, nav_msgs, trajectory_msgs (already core dependencies).
  A valid running roscore, /use_sim_time=true, a live /clock, and the same
  current_state / desired_state / desired_traj_vertices as the real trial.
  TF displays also require the bridge's TF publishers. Use the same ROS master.
The launch intentions, NOT executed by this package, are:
  python3 /actual/path/runtime/viz/lab4_viz.py
  rviz -d /actual/path/runtime/viz/lab4_sbs.rviz
Default marker square size is 2.0 m; it is a diagram, NOT measured gate aperture.
The unmodified RViz configuration explicitly limits its own frame rate to 15.
This setting is distinct from Unity FPS, recording FPS, /clock and physics Hz.

WSLg capture: researched guidance, target-machine compatibility untested here.
Do not start Xvfb or force LIBGL_ALWAYS_SOFTWARE merely to obtain a recording.
Keep actual Unity and RViz windows on the existing GPU-backed WSLg display.
Ensure those processes inherit the actual WSLg DISPLAY/socket environment.

1. Check the installed recorder before choosing a command:
     ffmpeg -hide_banner -h demuxer=x11grab
   The current FFmpeg documentation describes -window_id, but upstream n4.2
   and n4.3 xcbgrab implementations have NO window_id option; n4.4 has it.
   The old local container uses 4.2.7, so that command cannot be presumed valid
   on a newly installed Focal ffmpeg either. Inspect the actual build's help.

2. Identify real X11 client windows on the SAME display:
     xwininfo -root -tree
   Use the confirmed Unity/RViz client window ID, not the root, a guessed ID,
   window position or a stale ID from another run. Wayland-native clients do
   not have an X11 client window ID and cannot use x11grab -window_id directly.
   Qt RViz generally uses X11/xcb here; verify rather than assume.

3. If this FFmpeg exposes window_id, a short per-window proof can be attempted:
     ffmpeg -nostdin -n -f x11grab -framerate 30 -draw_mouse 0 \
       -window_id 0xCONFIRMED_ID -i "$DISPLAY" -t 3 \
       -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p probe.mkv
   Replace the literal ID before running. Keep each window mapped and fixed
   size; closing, minimizing or shrinking the window can stop capture. Inspect
   decoded frames for real, moving scene content and the Unity FPS overlay.
   Neither a successful exit nor a 30-fps file tag proves correct scene capture.
   GPU/Xwayland readback correctness remains a machine/driver test, not a
   promise from the mere existence of the option.

4. WSLg's RAIL shell remotes individual application windows to Windows; it is
   not a conventional Linux desktop compositor with the Windows desktop as its
   X11 root drawable. Therefore root x11grab/import -window root must not be
   treated as a capture of the composed Windows desktop. Blank/incomplete root
   captures are a risk inferred from that architecture, not a universal tested
   failure claim. Prefer per-window capture. If that fails, capture the real
   WSLg application window(s) on the Windows side, e.g. Windows Graphics Capture
   through a compatible recorder, without moving Unity to a software display.

5. Capture proof and timing are separate:
   Retain Unity Player.log renderer/vendor/version and glxinfo renderer output;
   record the unmodified Unity in-scene FPS overlay. Keep actual /clock and
   explicit monotonic-wall telemetry for RTF, and the original race rosbag.
   Sample/update FPS, recorder FPS, render FPS and simulated physics Hz are not
   interchangeable. Extra capture/readback/encoding load can affect callbacks,
   so compare a recorded run to a no-recording baseline on the same plan.
   A GPU-backed renderer must not be claimed based only on CUDA/nvidia-smi.

Primary references checked:
  FFmpeg devices, x11grab/window_id (window lifecycle and geometry limitations):
  https://ffmpeg.org/ffmpeg-devices.html#x11grab
  https://raw.githubusercontent.com/FFmpeg/FFmpeg/n4.2/libavdevice/xcbgrab.c
  https://raw.githubusercontent.com/FFmpeg/FFmpeg/n4.3/libavdevice/xcbgrab.c
  https://raw.githubusercontent.com/FFmpeg/FFmpeg/n4.4/libavdevice/xcbgrab.c
  Microsoft WSLg architecture, RAIL-Shell and accelerated OpenGL:
  https://github.com/microsoft/wslg#rail-shell
  https://github.com/microsoft/wslg#opengl-accelerated-rendering-in-wslg

No remote connection, installation, simulator/container process or GUI action
was performed to prepare this optional source package. Core v1 tar is unchanged.
