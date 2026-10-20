#!/usr/bin/env python3
"""Measure how violent the take-off is: tilt, body rates and rotor speeds over the first N seconds.
usage: takeoff_check.py [seconds]"""
import math, sys, time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rows, cmds = [], []


def on_state(m):
    q, w = m.pose.pose.orientation, m.twist.twist.angular
    r33 = 1 - 2 * (q.x * q.x + q.y * q.y)                  # body z . world z
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, r33))))
    p = m.pose.pose.position
    rows.append((m.header.stamp.to_sec(), tilt, math.degrees(math.sqrt(w.x ** 2 + w.y ** 2 + w.z ** 2)),
                 math.degrees(abs(w.z)), p.x, p.y, p.z))


rospy.init_node('takeoff_check', anonymous=True)
rospy.Subscriber('/current_state', Odometry, on_state, queue_size=100)
rospy.Subscriber('/rotor_speed_cmds', Actuators, lambda m: cmds.append(max(abs(v) for v in m.angular_velocities)), queue_size=1000)
t0 = time.time()
while not rospy.is_shutdown() and time.time() - t0 < dur:
    time.sleep(0.1)
a = np.array(rows)
t = a[:, 0] - a[0, 0]
moving = np.where(np.hypot(a[:, 4] - a[0, 4], a[:, 6] - a[0, 6]) > 0.02)[0]
k = moving[0] if len(moving) else 0
print('max tilt %.1f deg, max body rate %.0f deg/s, max yaw rate %.0f deg/s, max rotor cmd %.0f rad/s'
      % (a[:, 1].max(), a[:, 2].max(), a[:, 3].max(), max(cmds) if cmds else float('nan')))
tl = t[k:] - t[k]
for T in (1, 2, 4, 8):
    sel = tl <= T
    print('  first %d s after lift-off: max tilt %.1f deg, max body rate %.0f deg/s' % (T, a[k:][sel, 1].max(), a[k:][sel, 2].max()))
