#!/usr/bin/env python3
"""roll/pitch 高频振荡的频谱：roll_spectrum.py <bag>..."""
import sys, numpy as np, rosbag
for b in sys.argv[1:]:
    t, r, p, w = [], [], [], []
    for _, m, _ in rosbag.Bag(b).read_messages(['/current_state']):
        q = m.pose.pose.orientation
        t.append(m.header.stamp.to_sec())
        r.append(np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        p.append(np.arcsin(np.clip(2 * (q.w * q.y - q.z * q.x), -1, 1)))
        w.append(np.hypot(m.twist.twist.angular.x, m.twist.twist.angular.y))
    t, r, p, w = map(np.array, (t, r, p, w))
    moving = np.r_[False, np.abs(np.diff(r)) > 0]
    i0, i1 = np.argmax(moving), len(t) - np.argmax(moving[::-1])
    t, r, p, w = t[i0:i1], np.degrees(r[i0:i1]), np.degrees(p[i0:i1]), np.degrees(w[i0:i1])
    n = 25
    hp = r - np.convolve(r, np.ones(n) / n, mode='same')
    hp = hp[n:-n]
    f = np.fft.rfftfreq(len(hp), 0.02)
    A = np.abs(np.fft.rfft(hp * np.hanning(len(hp))))
    top = np.argsort(A)[-3:][::-1]
    bands = [(0, 2), (2, 5), (5, 10), (10, 25)]
    pw = [np.sum(A[(f >= a) & (f < c)] ** 2) for a, c in bands]
    tot = sum(pw)
    def band(x, lo=2.0, hi=10.0):
        X = np.fft.rfft(x - x.mean())
        fx = np.fft.rfftfreq(len(x), 0.02)
        X[(fx < lo) | (fx > hi)] = 0
        return np.fft.irfft(X, len(x)).std()
    print('%-22s SHAKE 2-10Hz roll %.2f pitch %.2f deg | tilt max %.0f deg' % (b.split('/')[-1], band(r), band(p), np.degrees(np.arccos(np.clip(np.cos(np.radians(r)) * np.cos(np.radians(p)), -1, 1))).max()))
    continue
    print('%-22s roll hf rms %.2f deg | peaks %s Hz | power 0-2/2-5/5-10/10-25 Hz: %s | body rate p95 %.0f deg/s' % (
        b.split('/')[-1], hp.std(), ', '.join('%.1f' % f[k] for k in top),
        '/'.join('%.0f%%' % (100 * x / tot) for x in pw), np.percentile(w, 95)))
