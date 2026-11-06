#!/usr/bin/env python3
"""Builds results/stats/tables.md from the logs of run_pair.sh and run_video.sh."""
import os, re
L = os.path.expanduser('~/Desktop/LAB5')
G = L + '/results/logs/'
KEYS = [('# of Keypoints in Img 1', 'Keypoints 1 Size'), ('# of Keypoints in Img 2', 'Keypoints 2 Size'),
        ('# of Matches', 'Number of matches'), ('# of Good Matches', 'Number of good matches'),
        ('# of Inliers', 'Number of Inliers'), ('Inlier Ratio', 'Inliers ratio'),
        ('Time per pair (ms)', 'Time per pair'), ('# of samples', 'samples')]

def read(name):
    p = G + name + '.log'
    if not os.path.exists(p):
        return None
    s = open(p, errors='replace').read()
    out = {}
    for _, k in KEYS:
        m = re.search(re.escape(k) + r'[^:\n]*: *([-\d.eE+a-z]+)', s)
        if m:
            out[k] = m.group(1)
    return out

def cell(st, k):
    if st is None or k not in st:
        return '-'
    v = float(st[k])
    if k == 'Inliers ratio':
        return '%.1f%%' % (100 * v)
    if k == 'samples':
        return '%d' % v
    return '%d' % round(v) if float(v).is_integer() else '%.1f' % v

def table(prefix, tags, names=None):
    names = names or tags
    rows = ['| Statistics | ' + ' | '.join(names) + ' |', '|---' * (len(tags) + 1) + '|']
    stats = [read(prefix + '_' + t) for t in tags]
    for title, k in KEYS:
        rows.append('| ' + title + ' | ' + ' | '.join(cell(st, k) for st in stats) + ' |')
    return '\n'.join(rows)

M = ['SIFT', 'SURF', 'ORB', 'FAST', 'LK']
N = ['SIFT', 'SURF', 'ORB', 'FAST+BRIEF', 'Harris+LK']
SWEEP = [('SIFT', 'SIFT contrast 0.04 (default)'), ('SIFT_c002', 'SIFT contrast 0.02'),
         ('SURF', 'SURF hessian 100 (default)'), ('SURF_h50', 'SURF hessian 50'), ('SURF_h400', 'SURF hessian 400'),
         ('ORB_n500', 'ORB 500 features, 8 levels'), ('ORB_L1', 'ORB 500 features, 1 level (skeleton)'),
         ('ORB_flann', 'ORB 500 features, FLANN-LSH'), ('ORB_n2000', 'ORB 2000 features'), ('ORB', 'ORB 5000 features (used)'),
         ('FAST', 'FAST threshold 10 (default)'), ('FAST_t5', 'FAST threshold 5'), ('FAST_t30', 'FAST threshold 30'),
         ('FAST_flann', 'FAST threshold 10, FLANN-LSH')]
out = ['# LAB5 result tables', '',
       '## D5/D6.a box.png -> box_in_scene.png', '', table('box', M[:4], N[:4]), '',
       '## D6.a parameters tried on the pair of images', '',
       '| Variant | Kp 1 | Kp 2 | Good | Inliers | Ratio | Time (ms) |', '|---|---|---|---|---|---|---|']
for tag, label in SWEEP:
    st = read('box_' + tag)
    out.append('| %s | %s |' % (label, ' | '.join(cell(st, k) for k in
               ['Keypoints 1 Size', 'Keypoints 2 Size', 'Number of good matches', 'Number of Inliers', 'Inliers ratio', 'Time per pair'])))
out += ['', '## D6.b/D7 30fps_424x240_2018-10-01-18-35-06.bag (average over frame pairs)', '', table('30fps', M, N), '',
        '## D6.b/D7 vnav-lab5-smooth-trajectory.bag (average over frame pairs)', '', table('smooth', M, N), '']
open(L + '/results/stats/tables.md', 'w').write('\n'.join(out))
print('\n'.join(out))
