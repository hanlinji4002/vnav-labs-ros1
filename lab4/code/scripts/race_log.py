#!/usr/bin/env python3
"""记录一次 LAB4 飞行（门、期望状态、实际状态）并打印指标。
用法：race_log.py --out results/flight/name [--duration 120] [--stop-after-end 3]
输出 <out>_des.csv、<out>_cur.csv、<out>_gates.csv、<out>_summary.txt
"""
import argparse, math, time
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
try:
    from tesse_ros_bridge.msg import CollisionStats
except ImportError:
    CollisionStats = None


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tilt_of(q):
    # angle between body z and world z
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))


class Recorder:
    def __init__(self):
        self.des, self.cur, self.gates, self.coll = [], [], [], []
        if CollisionStats is not None:   # bridge needs publish_collisions:=true
            rospy.Subscriber('/tesse/collision', CollisionStats, self.on_coll, queue_size=50)
        rospy.Subscriber('/desired_traj_vertices', PoseArray, self.on_gates, queue_size=1)
        rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, self.on_des, queue_size=50)
        rospy.Subscriber('/current_state', Odometry, self.on_cur, queue_size=50)

    def on_gates(self, m):
        if not self.gates:
            self.gates = [(p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
                          for p in m.poses]

    def on_coll(self, m):
        if m.is_collision:
            self.coll.append((rospy.Time.now().to_sec(), m.object_name))

    def on_des(self, m):
        tr, v, a = m.transforms[0], m.velocities[0], m.accelerations[0]
        self.des.append((rospy.Time.now().to_sec(), m.time_from_start.to_sec(),
                         tr.translation.x, tr.translation.y, tr.translation.z, yaw_of(tr.rotation),
                         v.linear.x, v.linear.y, v.linear.z, a.linear.x, a.linear.y, a.linear.z,
                         v.angular.z))

    def on_cur(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        self.cur.append((m.header.stamp.to_sec(), p.x, p.y, p.z, yaw_of(q), v.x, v.y, v.z, tilt_of(q)))


def write_csv(path, header, rows):
    with open(path, 'w') as f:
        f.write(header + '\n')
        for r in rows:
            f.write(','.join('%.6f' % v for v in r) + '\n')


def nearest(points, gate):
    """polyline (t,x,y,z) 离门中心最近的距离和时刻"""
    best = (float('inf'), 0.0)
    for (t0, *p0), (t1, *p1) in zip(points, points[1:]):
        d = [b - a for a, b in zip(p0, p1)]
        w = [g - a for a, g in zip(p0, gate[:3])]
        dd = sum(c * c for c in d)
        s = 0.0 if dd == 0 else max(0.0, min(1.0, sum(a * b for a, b in zip(d, w)) / dd))
        dist = math.dist([a + s * c for a, c in zip(p0, d)], gate[:3])
        if dist < best[0]:
            best = (dist, t0 + s * (t1 - t0))
    return best


def summarize(rec):
    des, gates, cur = rec.des, rec.gates, rec.cur
    if not des:
        return ['no /desired_state received']
    out = []
    t_start = des[0][0] - des[0][1]          # sim time when the trajectory started
    t_traj = max(r[1] for r in des)
    speed = [math.hypot(r[6], r[7], r[8]) for r in des]
    dts = [b[0] - a[0] for a, b in zip(des, des[1:])]
    out.append('desired_state msgs: %d, mean period %.3f s' % (len(des), sum(dts) / max(1, len(dts))))
    out.append('trajectory duration: %.2f s' % t_traj)
    out.append('desired speed max %.2f m/s' % max(speed))
    out.append('NaN rows in desired_state: %d' % sum(any(math.isnan(v) for v in r) for r in des))
    cur_pts = [(c[0], c[1], c[2], c[3]) for c in cur if c[0] >= t_start - 0.5]
    des_pts = [(r[0], r[2], r[3], r[4]) for r in des]
    if gates and len(cur_pts) > 1:
        out.append('gates: %d' % len(gates))
        out.append('gate   des_miss[m]  t_des[s]   act_miss[m]  t_act[s]')
        misses = []
        for i, g in enumerate(gates):
            d_des, t_des = nearest(des_pts, g)
            d_act, t_act = nearest(cur_pts, g)
            misses.append((d_act, t_act - t_start))
            out.append('%4d   %10.3f  %8.2f   %11.3f  %8.2f' % (i, d_des, t_des - t_start, d_act, t_act - t_start))
        out.append('max gate miss (actual): %.3f m' % max(m[0] for m in misses))
        # finish = first time within 10 cm of the last gate (it is a stop point)
        g = gates[-1]
        t_fin = next((c[0] - t_start for c in cur_pts if math.dist(c[1:4], g[:3]) < 0.10), float('nan'))
        out.append('race time (start -> within 10 cm of last gate): %.2f s' % t_fin)
        out.append('last gate passed (closest approach) at %.2f s, %.3f m from its centre' % (misses[-1][1], misses[-1][0]))
    flying = [c for c in rec.coll if c[0] >= t_start + 0.5]
    names = sorted(set(n for _, n in flying))
    out.append('collisions after take-off: %d messages %s' % (len(flying), names))
    for n in names:
        ts = [t - t_start for t, m in flying if m == n]
        out.append('  %s: t = %.2f .. %.2f s' % (n, min(ts), max(ts)))
    if cur_pts:
        errs, j = [], 0
        for c in cur_pts:
            while j + 1 < len(des) and des[j + 1][0] <= c[0]:
                j += 1
            if des[j][0] <= c[0]:
                errs.append(math.dist(c[1:4], des[j][2:5]))
        if errs:
            rms = math.sqrt(sum(e * e for e in errs) / len(errs))
            out.append('position error vs latest desired: rms %.3f m, max %.3f m' % (rms, max(errs)))
        sel = [c for c in cur if c[0] >= t_start - 0.5]
        out.append('actual speed max %.2f m/s, tilt max %.1f deg, min height %.2f m' % (
            max(math.hypot(c[5], c[6], c[7]) for c in sel), math.degrees(max(c[8] for c in sel)),
            min(c[3] for c in sel)))
        tail = [math.dist(c[1:4], des[-1][2:5]) for c in cur if c[0] > cur[-1][0] - 2.0]
        out.append('final hover error (mean of last 2 s): %.3f m' % (sum(tail) / len(tail)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--stop-after-end', type=float, default=3.0,
                    help='stop this long (sim s) after the trajectory reaches its end')
    args = ap.parse_args()
    rospy.init_node('race_log', anonymous=True)
    rec = Recorder()
    while rospy.Time.now().to_sec() == 0 and not rospy.is_shutdown():
        time.sleep(0.01)
    w0, s0 = time.time(), rospy.Time.now().to_sec()
    t_stop = time.time() + args.duration
    while not rospy.is_shutdown() and time.time() < t_stop:
        time.sleep(0.05)
        d = rec.des
        if len(d) > 2 and d[-1][1] == d[-2][1]:
            t_end = next(r[0] for r in d if r[1] == d[-1][1])
            if d[-1][0] - t_end > args.stop_after_end:
                break
    write_csv(args.out + '_des.csv', 't,time_from_start,x,y,z,yaw,vx,vy,vz,ax,ay,az,yaw_rate', list(rec.des))
    write_csv(args.out + '_cur.csv', 't,x,y,z,yaw,vx,vy,vz,tilt', list(rec.cur))
    write_csv(args.out + '_gates.csv', 'x,y,z,yaw', rec.gates)
    lines = summarize(rec)
    lines.append('real-time factor (sim/wall): %.3f' % ((rospy.Time.now().to_sec() - s0) / (time.time() - w0)))
    with open(args.out + '_summary.txt', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
