"""
ik_solver.py
------------
Pure-Python port of the Teensy IK / 4-bar linkage logic.
Each Leg instance holds its own branch-selection state (same hysteresis as C++).

Coordinate convention: same as Teensy
  x = abduction axis (L1 offset)
  y = forward/back
  z = up/down (negative = below hip)
"""

import math
from dataclasses import dataclass, field
from typing import Optional

# ── robot constants (mm) ──────────────────────────────────────────────────────
L1 = 26.086
L2 = 80
L3 = 80
L4 = 20
H  = 28.5
R  = 22       # crank radius  ← updated from 22 (keep in sync with CAD)
R3 = 20
R4 = 80

LIMIT_DEG = 70.0


# ── helpers ───────────────────────────────────────────────────────────────────
def _clamp(v, lo, hi): return max(lo, min(hi, v))
def _wrap_pi(a):
    while a <= -math.pi: a += 2*math.pi
    while a >   math.pi: a -= 2*math.pi
    return a
def _ang_diff(a, b): return _wrap_pi(a - b)
def _unwrap_near(a, ref): return ref + _ang_diff(a, ref)


def linkage_theta4_candidates(gamm: float):
    """Return the two passive-knee solutions for motor3 angle gamm."""
    a_ = R * math.sin(gamm)
    R1_ = math.sqrt(a_**2 + H**2)
    tmp = R * math.cos(gamm) - 3.0
    R2_sq = max(R4**2 - tmp**2, 0.0)
    theta1 = math.atan2(H, a_)

    A = 2.0 * R1_ * R3 * math.sin(theta1)
    B = 2.0 * R1_ * R3 * math.cos(theta1) - 2.0 * R3 * R4
    C = R1_**2 - R2_sq + R3**2 + R4**2 - 2.0 * R1_ * R4 * math.cos(theta1)

    disc = C**2 * (A**2 + B**2 - C**2)
    sq = math.sqrt(max(disc, 0.0))

    th4_1 = math.atan2(-A * B + sq, A**2 - C**2)
    th4_2 = math.atan2(-A * B - sq, A**2 - C**2)
    return th4_1, th4_2


def linkage_theta4_distance_error(gamm: float, th4: float):
    """Distance closure error for a theta4 candidate, in mm."""
    bx, by, bz = R * math.cos(gamm), R * math.sin(gamm), H
    c1x = 3.0
    c1y = R4 + R3 * math.cos(th4)
    c1z = R3 * math.sin(th4)
    d = math.sqrt((bx - c1x)**2 + (by - c1y)**2 + (bz - c1z)**2)
    return abs(d - L3)


class LinkageKneeMap:
    """Stateful nonlinear map from motor3 angle to passive knee angle."""

    def __init__(self, hysteresis_mm: float = 0.4):
        self.hysteresis_mm = hysteresis_mm
        self._inited = False
        self._choice = 1

    def compute(self, motor3_rad: float, do_update: bool = True):
        th4_1, th4_2 = linkage_theta4_candidates(motor3_rad)
        e1 = linkage_theta4_distance_error(motor3_rad, th4_1)
        e2 = linkage_theta4_distance_error(motor3_rad, th4_2)

        choice = self._choice
        if not self._inited:
            choice = 1 if e1 <= e2 else 2
            if do_update:
                self._inited = True
        elif choice == 1:
            if e2 < e1 - self.hysteresis_mm:
                choice = 2
        elif e1 < e2 - self.hysteresis_mm:
            choice = 1

        if do_update:
            self._choice = choice

        return th4_1 if choice == 1 else th4_2


def _R_apply(alp, bet, v):
    """Ry(alp) @ Rx(bet) @ v"""
    cb, sb = math.cos(bet), math.sin(bet)
    vx, vy, vz = v
    v1x = vx
    v1y = cb*vy + sb*vz
    v1z = -sb*vy + cb*vz
    ca, sa = math.cos(alp), math.sin(alp)
    v2x = ca*v1x - sa*v1z
    v2y = v1y
    v2z = sa*v1x + ca*v1z
    return (v2x, v2y, v2z)


# ── Leg class ─────────────────────────────────────────────────────────────────
class Leg:
    def __init__(self, z_on: float = -123.0, z_off: float = -123.0):
        # position target
        self.pos = [L1, 0.0, -155.0]

        # IK output (radians)
        self.angle1 = math.pi / 2
        self.angle2 = math.pi / 2
        self.angle3 = math.pi / 2

        # branch-selection state (same as Teensy)
        self._inited_c       = False
        self._last_c_local   = 0.0
        self._inited_theta4  = False
        self._last_theta4    = 1
        self._inited_pick    = False
        self._last_pick      = 1

        self.z_on  = z_on
        self.z_off = z_off

    def set_target(self, x: float, y: float, z: float):
        self.pos = [x, y, z]

    # ── FK end-effector (4-bar) ───────────────────────────────────────────────
    def _fk_end_effector(self, alp, bet, gamm, do_update: bool):
        th4_1, th4_2 = linkage_theta4_candidates(gamm)
        e1 = linkage_theta4_distance_error(gamm, th4_1)
        e2 = linkage_theta4_distance_error(gamm, th4_2)

        # hysteresis
        choice = self._last_theta4
        if not self._inited_theta4:
            choice = 1 if e1 <= e2 else 2
            if do_update:
                self._inited_theta4 = True
        else:
            thr = 0.4
            if choice == 1:
                if e2 < e1 - thr: choice = 2
            else:
                if e1 < e2 - thr: choice = 1

        if do_update:
            self._last_theta4 = choice

        th4 = th4_1 if choice == 1 else th4_2

        # end-effector position
        E_local = (0.0, R4*(1.0 - math.cos(th4)), -R4*math.sin(th4))
        p0 = _R_apply(alp, bet, (L1, 0.0, 0.0))
        Eg = _R_apply(alp, bet, E_local)
        return (Eg[0]+p0[0], Eg[1]+p0[1], Eg[2]+p0[2])

    # ── IK with FK-based branch selection ────────────────────────────────────
    def compute_ik(self):
        x, y, z = self.pos

        l2 = x**2 + y**2 + z**2
        l  = max(math.sqrt(l2), 1e-6)
        d2 = max(l2 - L1**2, 0.0)
        d  = max(math.sqrt(d2), 1e-6)

        cosK = _clamp((L1**2 + L2**2 + L3**2 - l2) / (2.0*L2*L3), -1, 1)
        theta_k = math.acos(cosK)

        s1   = _clamp(y / d, -1, 1)
        cos2 = _clamp((L2**2 + d**2 - L3**2) / (2.0*L2*d), -1, 1)
        theta_b = math.pi/2 - (math.asin(s1) + math.acos(cos2))

        ca      = _clamp(L1 / l, -1, 1)
        theta_a = math.atan2(x, abs(z)) + math.acos(ca) - math.pi/2

        # theta_c candidates
        A = 2.0*R*(L4*math.cos(theta_k) + L2)
        B = 6.0*R
        C = R**2 + (L2 + L4*math.cos(theta_k))**2 + 9.0 + (L4*math.sin(theta_k) - H)**2 - L3**2

        disc = (A*B)**2 - (A**2 - C**2)*(B**2 - C**2)
        sq   = math.sqrt(max(disc, 0.0))
        c1   = _wrap_pi(math.atan2(-A*B + sq, A**2 - C**2))
        c2   = _wrap_pi(math.atan2(-A*B - sq, A**2 - C**2))

        c1u = c1u_raw = c1
        c2u = c2u_raw = c2
        if self._inited_c:
            c1u = _unwrap_near(c1, self._last_c_local)
            c2u = _unwrap_near(c2, self._last_c_local)

        # FK error for each candidate
        E1 = self._fk_end_effector(theta_a, theta_b, c1u, False)
        E2 = self._fk_end_effector(theta_a, theta_b, c2u, False)

        def norm3(a, b): return math.sqrt(sum((ai-bi)**2 for ai,bi in zip(a,b)))
        pos_err1 = norm3(E1, (x,y,z))
        pos_err2 = norm3(E2, (x,y,z))

        cont1 = abs(c1u - self._last_c_local) if self._inited_c else 0.0
        cont2 = abs(c2u - self._last_c_local) if self._inited_c else 0.0

        W_POS, W_CONT, HYS = 1.0, 0.05, 0.5
        J1 = W_POS*pos_err1 + W_CONT*cont1
        J2 = W_POS*pos_err2 + W_CONT*cont2

        if not self._inited_pick:
            pick = 1 if J1 <= J2 else 2
            self._inited_pick = True
            self._inited_c    = True
        else:
            if self._last_pick == 1:
                pick = 2 if J2 < J1 - HYS else 1
            else:
                pick = 1 if J1 < J2 - HYS else 2

        chosen = c1u if pick == 1 else c2u

        # commit theta4 memory
        self._fk_end_effector(theta_a, theta_b, chosen, True)
        self._last_c_local = chosen
        self._last_pick    = pick

        self.angle1 = _wrap_pi(theta_a)
        self.angle2 = _wrap_pi(theta_b)
        self.angle3 = chosen

    # ── convenience ──────────────────────────────────────────────────────────
    @property
    def angles_deg(self):
        """Returns (a1_deg, a2_deg, a3_deg)"""
        return (math.degrees(self.angle1),
                math.degrees(self.angle2),
                math.degrees(self.angle3))
