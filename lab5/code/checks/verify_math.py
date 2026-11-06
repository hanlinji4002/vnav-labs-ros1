#!/usr/bin/env python3
"""Numeric check of the D1 (sphere projection) and D2 (vanishing point) answers."""
import numpy as np

# D1 on axis: the projection is a circle of radius r / sqrt(d^2 - r^2)
for r, d in [(1, 3), (0.5, 2), (2, 3.5)]:
    th = np.linspace(0, np.pi, 200001)
    x = r * np.sin(th)
    z = d + r * np.cos(th)
    print('D1 on axis r=%g d=%g: max radius %.10f, formula %.10f' % (r, d, (x / z).max(), r / np.sqrt(d * d - r * r)))

# D1 off axis: tangent rays satisfy (c.m)^2 = (m.m)(|c|^2 - r^2), an ellipse when c_z > r
for c, r in [(np.array([1., 0.5, 4.]), 1.), (np.array([2., -1., 3.]), 1.5)]:
    k = c @ c - r * r
    t = c / np.linalg.norm(c)
    e1 = np.cross(t, [0, 0, 1.])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(t, e1)
    # circle where the tangent rays touch the sphere
    centre = c * k / (c @ c)
    rc = r * np.sqrt(k) / np.linalg.norm(c)
    a = np.linspace(0, 2 * np.pi, 721)
    P = centre + rc * (np.outer(np.cos(a), e1) + np.outer(np.sin(a), e2))
    m = P / P[:, 2:3]
    residual = (m @ c) ** 2 - (m * m).sum(1) * k
    Q = np.outer(c, c) - k * np.eye(3)
    ellipse_centre = -np.linalg.solve(Q[:2, :2], Q[:2, 2])
    C = c[2]
    print('D1 c=%s r=%g: tangency %.1e, conic residual %.1e, det %.4f = k(C^2-r^2) %.4f'
          % (c, r, np.abs(((P - c) * P).sum(1)).max(), np.abs(residual).max(), np.linalg.det(Q[:2, :2]), k * (C * C - r * r)))
    print('   centre %s, formula %s' % (ellipse_centre, C * c[:2] / (C * C - r * r)))
    w, v = np.linalg.eigh(Q[:2, :2])
    print('   axes %s (eigenvalues %s), radial direction %s' % (v.T.tolist(), w, c[:2] / np.linalg.norm(c[:2])))

# D2: parallel lines meet at (ux/uz, uy/uz); with uz = 0 they stay parallel
u = np.array([.3, -.2, .9])
u /= np.linalg.norm(u)
p1, p2 = np.array([1., 2., 5.]), np.array([-3., .5, 2.])
for p0 in (p1, p2):
    P = p0 + 1e8 * u
    print('D2 far point %s, formula %s' % (P[:2] / P[2], u[:2] / u[2]))
print('D2 (p1 x u) x (p2 x u) / u = %s' % (np.cross(np.cross(p1, u), np.cross(p2, u)) / u))
u0 = np.array([.6, .8, 0.])
l1, l2 = np.cross(p1, u0), np.cross(p2, u0)
print('D2 uz = 0: intersection %s, line normals %s %s'
      % (np.cross(l1, l2), l1[:2] / np.linalg.norm(l1[:2]), l2[:2] / np.linalg.norm(l2[:2])))
