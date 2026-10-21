#!/usr/bin/env python3
"""Part 0 记录：期望点 vs 实际位置。用法：hover_log.py --out name --duration 20"""
import argparse, math, time
import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--duration', type=float, default=20.0)
a = ap.parse_args()
rospy.init_node('hover_log', anonymous=True)
des, rows = [None], []


def on_des(m):
    des[0] = m.transforms[0]


def on_cur(m):
    if des[0] is None:
        return
    p, q, d = m.pose.pose.position, m.pose.pose.orientation, des[0]
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    rows.append((m.header.stamp.to_sec(), p.x, p.y, p.z, d.translation.x, d.translation.y, d.translation.z, yaw))


rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, on_des, queue_size=10)
rospy.Subscriber('/current_state', Odometry, on_cur, queue_size=50)
t_end = time.time() + a.duration
while time.time() < t_end and not rospy.is_shutdown():
    time.sleep(0.05)
with open(a.out + '.csv', 'w') as f:
    f.write('t,x,y,z,xd,yd,zd,yaw\n')
    for r in rows:
        f.write(','.join('%.6f' % v for v in r) + '\n')
if not rows:
    print('no data'); raise SystemExit(1)
t0 = rows[0][0]
err = [math.dist(r[1:4], r[4:7]) for r in rows]
d0 = err[0]
t5 = next((r[0] - t0 for r, e in zip(rows, err) if e < 0.05), float('nan'))
lines = ['target gate 0: (%.2f, %.2f, %.2f)' % rows[-1][4:7],
         'start (%.2f, %.2f, %.2f), distance %.2f m' % (rows[0][1], rows[0][2], rows[0][3], d0),
         'first time within 5 cm: %.2f s' % t5,
         'final error (mean of last 2 s): %.4f m' % (sum(e for r, e in zip(rows, err) if r[0] > rows[-1][0] - 2) /
                                                    sum(1 for r in rows if r[0] > rows[-1][0] - 2)),
         'final yaw %.1f deg' % math.degrees(rows[-1][7])]
open(a.out + '_summary.txt', 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines))
