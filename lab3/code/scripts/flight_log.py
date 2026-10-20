#!/usr/bin/env python3
"""Record /desired_state vs /current_state for a while, save CSV and print tracking metrics.
usage: flight_log.py --mode circle|hover --duration 40 --warmup 10 --out ~/Desktop/LAB3/results/flight/name
"""
import argparse, csv, math, os, sys, time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from mav_msgs.msg import Actuators


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Logger:
    def __init__(self):
        self.des = None
        self.cmd = [float('nan')] * 4
        self.rows = []
        self.des_log = []   # (receive sim time, xd, yd) to recover the circle phase
        rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, self.on_des, queue_size=10)
        rospy.Subscriber('/rotor_speed_cmds', Actuators, self.on_cmd, queue_size=10)
        rospy.Subscriber('/current_state', Odometry, self.on_cur, queue_size=50)

    def on_des(self, m):
        self.des = m
        tr = m.transforms[0].translation
        self.des_log.append((rospy.Time.now().to_sec(), tr.x, tr.y))

    def on_cmd(self, m):
        if len(m.angular_velocities) == 4:
            self.cmd = list(m.angular_velocities)

    def on_cur(self, m):
        if self.des is None:
            return
        d = self.des
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        pd, vd, qd = d.transforms[0].translation, d.velocities[0].linear, d.transforms[0].rotation
        self.rows.append([m.header.stamp.to_sec(), p.x, p.y, p.z, pd.x, pd.y, pd.z,
                          v.x, v.y, v.z, vd.x, vd.y, vd.z, yaw_of(q), yaw_of(qd)] + self.cmd)


COLS = ['t', 'x', 'y', 'z', 'xd', 'yd', 'zd', 'vx', 'vy', 'vz', 'vxd', 'vyd', 'vzd',
        'yaw', 'yawd', 'w1', 'w2', 'w3', 'w4']


def analytic_desired(a, des_log):
    """Replace the latest-received desired state by the circle evaluated at each state stamp.
    traj_publisher: p=(5 sin(s/2), 5 cos(s/2), 2), yaw=-s/2 with s=t-t0; t0 from the phase of the desired messages."""
    d = np.array(des_log)
    d = d[d[:, 0] > 1.0]                      # rospy time is 0 until the first /clock
    # each message gives t0 = t_recv - 2*phase (mod 4*pi); ~97 % are received in the clock tick
    # they were sent in, so the median is t0 (min would pick up the few with a lagging clock)
    c = d[:, 0] - 2 * np.arctan2(d[:, 1], d[:, 2])
    med = 2 * np.angle(np.mean(np.exp(1j * c / 2)))
    r = 2 * np.angle(np.exp(1j * (c - med) / 2))
    t0 = med + np.median(r)
    s = a[:, 0] - t0
    b = a.copy()
    b[:, 4], b[:, 5], b[:, 6] = 5 * np.sin(s / 2), 5 * np.cos(s / 2), 2.0
    b[:, 10], b[:, 11], b[:, 12] = 2.5 * np.cos(s / 2), -2.5 * np.sin(s / 2), 0.0
    b[:, 14] = [wrap(y) for y in -s / 2]
    return b


def metrics(a, mode, warmup):
    t = a[:, 0] - a[0, 0]
    e = a[:, 1:4] - a[:, 4:7]
    en = np.linalg.norm(e, axis=1)
    out = {'samples': len(a), 'span_s': t[-1], 'rate_hz': (len(a) - 1) / t[-1]}
    ss = t >= warmup
    out['ss_pos_err_mean'] = en[ss].mean()
    out['ss_pos_err_max'] = en[ss].max()
    yaw_err = np.array([wrap(y) for y in a[:, 13] - a[:, 14]])
    out['ss_yaw_err_mean_deg'] = math.degrees(yaw_err[ss].mean())
    out['ss_yaw_err_maxabs_deg'] = math.degrees(np.abs(yaw_err[ss]).max())
    within = en < 0.1
    # first time after which the error stays below 0.1 m
    bad = np.where(~within)[0]
    out['settle_0.1m_s'] = 0.0 if len(bad) == 0 else (t[bad[-1] + 1] if bad[-1] + 1 < len(t) else float('nan'))
    if mode == 'hover':
        z0, zd = a[0, 3], a[-1, 6]
        out['z_overshoot_pct'] = 100 * max(0.0, (a[:, 3].max() - zd) / (zd - z0)) if zd != z0 else 0.0
        out['final_pos'] = a[-1, 1:4].round(4).tolist()
    else:
        r = np.hypot(a[:, 1], a[:, 2]) - np.hypot(a[:, 4], a[:, 5])
        vd = a[:, 10:13]
        tang = vd / np.maximum(np.linalg.norm(vd, axis=1, keepdims=True), 1e-9)
        along = np.sum(e * tang, axis=1)
        out['ss_radial_err_mean'] = r[ss].mean()
        out['ss_radial_err_std'] = r[ss].std()
        out['ss_height_err_mean'] = e[ss, 2].mean()
        out['ss_along_err_mean'] = along[ss].mean()
        out['ss_speed_mean'] = np.linalg.norm(a[ss, 7:10], axis=1).mean()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='circle', choices=['circle', 'hover'])
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--warmup', type=float, default=10.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('flight_log', anonymous=True)
    lg = Logger()
    wall0 = time.time()   # watchdog: a crashed simulator stops /clock and would hang rospy.sleep
    while not rospy.is_shutdown() and not lg.rows:
        time.sleep(0.05)
        if time.time() - wall0 > 20:
            sys.exit("no data from the simulator")
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < args.duration:
        time.sleep(0.1)
        if time.time() - wall0 > args.duration + 30:
            sys.exit("simulator clock stopped")
    a = np.array(lg.rows)
    if args.mode == 'circle':
        a = analytic_desired(a, lg.des_log)
    os.makedirs(os.path.dirname(os.path.expanduser(args.out)), exist_ok=True)
    with open(os.path.expanduser(args.out) + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLS)
        w.writerows(a.tolist())
    np.savetxt(os.path.expanduser(args.out) + '_des.csv', np.array(lg.des_log), delimiter=',',
               header='t_recv,xd,yd', comments='')
    m = metrics(a, args.mode, args.warmup)
    lines = ['%s: %s' % (k, ('%.4f' % v) if isinstance(v, float) else v) for k, v in m.items()]
    with open(os.path.expanduser(args.out) + '_metrics.txt', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
