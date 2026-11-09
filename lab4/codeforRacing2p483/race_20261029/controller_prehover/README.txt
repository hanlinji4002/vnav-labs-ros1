Isolated prehover attitude-gain scheduling
========================================

The original controller_candidate package is unchanged. This package copies its
controller and changes only effective position/velocity and attitude/rate
feedback gains plus schedule tracing. Allocation, signed thrust, reference
feedforward, inertia, simulator constants, gates, clock and physical settings
remain unchanged. The original high-gain position loop is not retained during
low-gain hover: both loops use the SAME quintic transition weight.

Default parameters:
  prehover_kx=4, prehover_kv=3, prehover_kr=12, prehover_komega=1, hover_release_t=5,
  gain_transition_s=.3, race_end_plan_s=60 (MUST pass actual candidate CSV end).
  Existing kx/kv/kr/komega remain race gains (current runner passes 36/12/60/4).

At plan time <=5, gains stay at 4/3/12/1 even while the reference is waiting for
measured hover stability. During 5..5.3 they rise with quintic smoothstep to
race gains. During end-.3..end they return smoothly to 4/3/12/1 for terminal hold.
The transition has zero endpoint first/second derivatives. The scheduler never
modifies desired time or the reference; it merely reads time_from_start.

The position small-angle natural frequency decreases from sqrt(36)=6 to
sqrt(4)=2 rad/s, with hover damping ratio3/(2*sqrt(4))=.75. At J=.1,
the attitude small-angle continuous natural frequency decreases from
sqrt(60/.1)=24.49 to sqrt(12/.1)=10.95 rad/s. With approximately20ms actual
feedback, this lowers omega*dt from .49 to .22, leaving more margin for delay
and sample/hold. This motivates a test; it is not a proof of real stability.
Actual hover position/speed/tilt/angular-rate thresholds are NOT relaxed.

Package: controller_prehover
Executable: controller_prehover_node
ROS node name stays controller_sprint, retaining the runner's private parameters.
Topic: /controller_sprint/gain_schedule (geometry_msgs/Vector3Stamped):
  x=effective kr, y=effective komega, z=race weight; stamp=actual simulator time.
One sample is published for each distinct simulator timestamp. Add this topic
to the bag, alongside original odometry/commands/clock/collision evidence.

Build: mirror this package into ros_ws/src/controller_prehover, source ROS Noetic,
then run catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
from the existing native ros_ws. Never overwrite controller_sprint.

Pure scheduler check (no ROS/Eigen required):
  c++ -std=c++11 -Wall -Wextra -pedantic -Iinclude tests/test_gain_schedule.cpp -o /tmp/lab4_prehover_schedule_test
  /tmp/lab4_prehover_schedule_test

Additional topic: /controller_sprint/pos_gain_schedule (Vector3Stamped):
  x=effective kx, y=effective kv, z=the SAME race weight. Record both topics.

Source audit: SOURCE_COMPARISON.json, declared_replacements.json and the unified
gain_schedule_changes.diff. Reversing the declared source-line replacements
exactly reconstructs the original source bytes. This check does not replace compilation
or real simulator measurements.
