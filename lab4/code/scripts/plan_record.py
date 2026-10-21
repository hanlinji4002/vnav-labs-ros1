#!/usr/bin/env python3
"""记录离线规划的 /desired_state 直到轨迹结束，写 <out>_plan.csv 并打印离墙距离等。
用法：plan_record.py <输出前缀>"""
import math, sys, time
import numpy as np
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

WALL_X = -50.89          # drone centre x at contact with the west wall (gate 9 -> 10 turn)
GATES = None
rows = []


def cb(m):
    tr, a = m.transforms[0].translation, m.accelerations[0].linear
    rows.append((m.time_from_start.to_sec(), tr.x, tr.y, tr.z, a.x, a.y, a.z))


rospy.init_node('plan_record', anonymous=True)
rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, cb, queue_size=1000)
t_wall = time.time()
while time.time() - t_wall < 120:
    time.sleep(0.2)
    if len(rows) > 5 and rows[-1][0] == rows[-5][0]:
        break
a = np.array(rows)
np.savetxt(sys.argv[1] + '_plan.csv', a, delimiter=',', header='t,x,y,z,ax,ay,az', comments='', fmt='%.4f')
reg = (a[:, 2] > -1) & (a[:, 2] < 10) & (a[:, 1] < -44)
k = np.argmin(a[reg, 1])
f = np.c_[a[:, 4], a[:, 5], a[:, 6] + 9.81]
tilt = np.degrees(np.arccos(f[:, 2] / np.linalg.norm(f, axis=1)))
print('samples %d, duration %.2f s' % (len(a), a[:, 0].max()))
print('gate 9->10 turn: min x %.2f at y %.2f -> %.2f m before wall contact'
      % (a[reg, 1][k], a[reg, 2][k], a[reg, 1][k] - WALL_X))
print('peak tilt %.1f deg, peak tilt in the 9->10 region %.1f deg' % (tilt.max(), tilt[reg].max()))
