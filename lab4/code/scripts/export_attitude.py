#!/usr/bin/env python3
"""Export t, roll, pitch (deg) of /current_state from bags (run on the guest):
export_attitude.py <out_dir> <bag>..."""
import os, sys, numpy as np, rosbag
out = sys.argv[1]
for b in sys.argv[2:]:
    rows = []
    for _, m, _ in rosbag.Bag(b).read_messages(['/current_state']):
        q = m.pose.pose.orientation
        r = np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        p = np.arcsin(np.clip(2 * (q.w * q.y - q.z * q.x), -1, 1))
        rows.append((m.header.stamp.to_sec(), np.degrees(r), np.degrees(p)))
    name = os.path.basename(b)[:-4]
    np.savetxt(os.path.join(out, name + '_att.csv'), rows, delimiter=',', header='t,roll,pitch', comments='', fmt='%.5f')
    print(name, len(rows))
