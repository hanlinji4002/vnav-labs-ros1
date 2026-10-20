#!/usr/bin/env python3
"""How violent / shaky is the flight: yaw change, tilt, body rates, rotor speeds, and roll/pitch shake
(RMS after removing a 0.5 s moving average) during the take-off and in steady flight.
usage: takeoff_check.py [seconds] [csv]"""
import math, sys, time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rows, cmds = [], []


def on_state(m):
    q, w, p = m.pose.pose.orientation, m.twist.twist.angular, m.pose.pose.position
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))
    rows.append((m.header.stamp.to_sec(), p.x, p.y, p.z, roll, pitch, yaw, tilt,
                 math.sqrt(w.x ** 2 + w.y ** 2 + w.z ** 2), abs(w.z)))


rospy.init_node('takeoff_check', anonymous=True)
rospy.Subscriber('/current_state', Odometry, on_state, queue_size=100)
rospy.Subscriber('/rotor_speed_cmds', Actuators, lambda m: cmds.append(max(abs(v) for v in m.angular_velocities)), queue_size=1000)
t0 = time.time()
while not rospy.is_shutdown() and time.time() - t0 < dur:
    time.sleep(0.1)
a = np.array(rows)
if len(sys.argv) > 2:
    np.savetxt(sys.argv[2], a, delimiter=',', header='t,x,y,z,roll,pitch,yaw,tilt,body_rate,yaw_rate', comments='')
t = a[:, 0] - a[0, 0]
moving = np.where(np.abs(a[:, 3] - a[0, 3]) > 0.02)[0]
k = moving[0] if len(moving) else 0
tl = t - t[k]
deg = np.degrees
yaw = np.unwrap(a[:, 6])
print('yaw at start %.1f deg; yaw change in the first 3 s after lift-off %.1f deg'
      % (deg(a[k, 6]), deg(np.ptp(yaw[(tl >= 0) & (tl <= 3)]))))
print('max tilt %.1f deg, max body rate %.0f deg/s, max yaw rate %.0f deg/s, max rotor cmd %.0f rad/s'
      % (deg(a[:, 7].max()), deg(a[:, 8].max()), deg(a[:, 9].max()), max(cmds) if cmds else float('nan')))
n = 25  # 0.5 s at 50 Hz
def shake(sel):
    out = []
    for c in (4, 5):
        x = np.unwrap(a[:, c])
        hp = x - np.convolve(x, np.ones(n) / n, mode='same')
        out.append(deg(np.sqrt(np.mean(hp[sel] ** 2))))
    return out
valid = (np.arange(len(a)) >= n) & (np.arange(len(a)) < len(a) - n)
for name, sel in [('take-off 0-10 s', (tl >= 0) & (tl <= 10)), ('steady  >12 s', tl > 12)]:
    sel = sel & valid
    if sel.sum() > n:
        r, p = shake(sel)
        print('shake (roll/pitch RMS above 2 Hz) %s: %.2f / %.2f deg' % (name, r, p))
