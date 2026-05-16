"""Shared joint naming and safe pose helpers for Doggy control nodes."""

from quadruped_control.ik_solver import Leg, LIMIT_DEG, _clamp


# The existing Teensy command order is named as:
#   LB1 LB2 LB3 | RB1 RB2 RB3 | RF1 RF2 RF3 | LF1 LF2 LF3
# In the URDF this maps to:
#   LB -> RL, RB -> RR, RF -> FR, LF -> FL
COMMAND_LEG_ORDER = ("RL", "RR", "FR", "FL")

MOTOR_JOINTS_BY_LEG = {
    "RL": ("RL_hip_roll", "RL_hip_pitch", "RL_motor3"),
    "RR": ("RR_hip_roll", "RR_hip_pitch", "RR_motor3"),
    "FR": ("FR_hip_roll", "FR_hip_pitch", "FR_motor3"),
    "FL": ("FL_hip_roll", "FL_hip_pitch", "FL_motor3"),
}

KNEE_JOINT_BY_LEG = {
    "RL": "RL_knee",
    "RR": "RR_knee",
    "FR": "FR_knee",
    "FL": "FL_knee",
}

COMMAND_JOINT_NAMES = tuple(
    joint
    for leg in COMMAND_LEG_ORDER
    for joint in MOTOR_JOINTS_BY_LEG[leg]
)

STAND_X = 26.086
STAND_Y = 0.0
STAND_Z = -150.0


def clamp_joint_angles_deg(values, limit_deg: float = LIMIT_DEG):
    return [float(_clamp(v, -limit_deg, limit_deg)) for v in values]


def stand_joint_angles_deg():
    """Return the current safe standing motor command in Teensy order."""
    lb = Leg()
    rb = Leg()
    rf = Leg()
    lf = Leg()

    for leg in (lb, rb, rf, lf):
        leg.set_target(STAND_X, STAND_Y, STAND_Z)
        leg.compute_ik()

    angles = [
         lb.angles_deg[0], -lb.angles_deg[1],  lb.angles_deg[2],
        -rb.angles_deg[0],  rb.angles_deg[1], -rb.angles_deg[2],
         rf.angles_deg[0],  rf.angles_deg[1], -rf.angles_deg[2],
        -lf.angles_deg[0], -lf.angles_deg[1],  lf.angles_deg[2],
    ]
    return clamp_joint_angles_deg(angles)
