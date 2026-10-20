#!/usr/bin/env python3
"""Open-loop thrust check: equal prop speeds w for T s, fit vertical acceleration, compare with m=1, cf=1e-3."""
import sys, time, numpy as np, rospy
from nav_msgs.msg import Odometry
from tesse.env import Env
from tesse.msgs import PropSpeeds, Respawn
import os; PERIOD = 1.0 / float(os.environ.get("RATE", "100"))
rospy.init_node('openloop_thrust', anonymous=True)
env, buf = Env(), []
rospy.Subscriber('/current_state', Odometry, lambda m: buf.append((m.header.stamp.to_sec(), m.pose.pose.position.z, m.twist.twist.linear.z)))
for w in [float(s) for s in sys.argv[1:]] or [52.0, 55.0, 60.0]:
    env.send(PropSpeeds(0, 0, 0, 0)); env.send(Respawn()); time.sleep(1.5)
    del buf[:]
    t_end = time.time() + 1.5
    while time.time() < t_end:
        env.send(PropSpeeds(w, w, w, w)); time.sleep(PERIOD)
    env.send(PropSpeeds(0, 0, 0, 0))
    a = np.array(buf); a = a[a[:, 1] > a[0, 1] + 0.01]       # airborne part
    acc = np.polyfit(a[:, 0] - a[0, 0], a[:, 2], 1)[0]      # slope of vz
    T_model = 4e-3 * w * w
    print('w=%.1f  thrust_model=%.3f N  a_model=%.3f  a_meas=%.3f  -> T/m_sim = %.3f  hover needs %.3f N (model units)'
          % (w, T_model, T_model - 9.81, acc, acc + 9.81, T_model * 9.81 / (acc + 9.81)))
env.send(PropSpeeds(0, 0, 0, 0)); env.send(Respawn())
