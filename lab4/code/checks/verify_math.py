#!/usr/bin/env python3
"""Numeric check of Deliverable 1 (single-segment minimum-velocity QP) and the
constraint count of Deliverable 2."""
from fractions import Fraction as F

import numpy as np


def Q(n):
    """Cost matrix for P(t) = p1 t + ... + pn t^n:  Q_ij = i j / (i + j - 1)."""
    return np.array([[F(i * j, i + j - 1) for j in range(1, n + 1)] for i in range(1, n + 1)])


def solve(n, A, b):
    """min p^T Q p  s.t.  A p = b  ->  KKT system."""
    q, A, b = Q(n).astype(float), np.array(A, float), np.array(b, float)
    m = len(b)
    kkt = np.block([[2 * q, A.T], [A, np.zeros((m, m))]])
    p = np.linalg.solve(kkt, np.r_[np.zeros(n), b])[:n]
    return p, p @ q @ p


print("Q (degree 2):", Q(2).tolist())
print("Q (degree 3):", Q(3).tolist())
for name, n, A, b in [
    ("1   degree 1, P(1)=1", 1, [[1]], [1]),
    ("2c  degree 2, P(1)=1", 2, [[1, 1]], [1]),
    ("3b  degree 3, P(1)=1", 3, [[1, 1, 1]], [1]),
    ("4a  degree 2, P(1)=1, P'(1)=-2", 2, [[1, 1], [1, 2]], [1, -2]),
    ("4b  degree 3, P(1)=1, P'(1)=-2", 3, [[1, 1, 1], [1, 2, 3]], [1, -2]),
]:
    p, cost = solve(n, A, b)
    print(f"{name}: p = {np.round(p, 6).tolist()}, cost = {cost:.6f}")

print("D2: unknowns 2rk = waypoint 2k + fixed 2(r-1) + free-derivative continuity (k-1)(2r-2)")
for r, k in [(4, 3), (4, 5), (1, 4), (3, 2)]:
    lhs, parts = 2 * r * k, (2 * k, 2 * (r - 1), (k - 1) * (2 * r - 2))
    print(f"  r={r} k={k}: {lhs} = {parts[0]} + {parts[1]} + {parts[2]} -> {lhs == sum(parts)}")

# Q4 remark: with P'(1) = -2 fixed, higher degrees lower the cost towards the
# unconstrained optimum 1 (a point value of P' costs arbitrarily little energy)
for n in range(2, 9):
    A = [[1] * n, list(range(1, n + 1))]
    p, cost = solve(n, A, [1, -2])
    print(f"  4   degree {n}: cost = {cost:.6f}")
# exact fractions for 4b
from sympy import Rational, symbols, integrate, solve as ssolve, diff
t, a1, a2, a3, l1, l2 = symbols('t a1 a2 a3 l1 l2')
P = a1 * t + a2 * t**2 + a3 * t**3
J = integrate(diff(P, t)**2, (t, 0, 1))
L = J + l1 * (P.subs(t, 1) - 1) + l2 * (diff(P, t).subs(t, 1) + 2)
sol = ssolve([diff(L, v) for v in (a1, a2, a3, l1, l2)], [a1, a2, a3, l1, l2], dict=True)[0]
print("  4b exact:", {k: sol[k] for k in (a1, a2, a3)}, "cost", J.subs(sol))
