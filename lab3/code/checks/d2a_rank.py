#!/usr/bin/env python3
"""D2-A: rank of the wrench map F for quadrotor (a) and (b) of the handout figure.
Top view, x_B right, y_B up, z_B up: prop 1 (-,+), 2 (-,-), 3 (+,-), 4 (+,+), arm length d.
Drag torque of a prop spinning CW (seen from top) is +cd*u (CCW), of a CCW prop -cd*u."""
import numpy as np
cf, cd, d = 1e-3, 1e-5, 0.3
l = d / np.sqrt(2)
pos = np.array([[-1, 1], [-1, -1], [1, -1], [1, 1]]) * l
spin = {'a': ['ccw', 'cw', 'ccw', 'cw'], 'b': ['ccw', 'cw', 'cw', 'ccw']}
for k, s in spin.items():
    F = np.zeros((4, 4))
    for i in range(4):
        rho = np.array([pos[i, 0], pos[i, 1], 0.0])
        tau = np.cross(rho, [0, 0, cf])               # thrust moment
        F[:, i] = [cf, tau[0], tau[1], cd if s[i] == 'cw' else -cd]
    print('(%s) spins %s' % (k, s))
    print(np.array2string(F / np.array([[cf], [cf * l], [cf * l], [cd]]), precision=0))
    print('    rank %d, det %.3e, singular values %s' % (np.linalg.matrix_rank(F), np.linalg.det(F),
                                                       np.array2string(np.linalg.svd(F)[1], precision=2)))
    if np.linalg.matrix_rank(F) < 4:
        # pure yaw torque at hover thrust is not reachable
        target = np.array([9.81, 0, 0, 1e-3])
        u, res, *_ = np.linalg.lstsq(F, target, rcond=None)
        print('    best wrench for [mg,0,0,1e-3]:', np.array2string(F @ u, precision=6), '(yaw torque lost)')
