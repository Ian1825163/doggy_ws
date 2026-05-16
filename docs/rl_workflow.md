# Doggy RL Deployment Workflow

This is the working plan for moving Doggy from the current ROS 2 gait stack toward an RL policy that can be trained in simulation and deployed on the Jetson.

## Current Mechanical Check

Latest Onshape assembly checked: `doggy_ros_export`

- Estimated total mass from Onshape part mass properties: `1.348 kg`
- Mass-bearing part occurrences found: `44`
- Current local `src/doggy/robot.urdf` is not the final full robot export yet. It still represents an older partial export, so use the Onshape mass above until the URDF export succeeds again.

Largest mass contributors:

| Part | Mass |
| --- | ---: |
| `teensy_battery <1>` | `0.292 kg` |
| `jetson <1>` | `0.253 kg` |
| `body upper plate v2` | `0.0898 kg` |
| `body upper plate v3` | `0.0884 kg` |
| `Feetech STS2032` motors | `0.025 kg` each |
| `foot` | `0.020 kg` each |

Current export blocker:

- `onshape-to-robot doggy` finds all four legs and 16 DOFs.
- Export currently fails because `frame_base_link` has an empty Onshape occurrence.
- Fix in Onshape: attach `frame_base_link` to the `body <1>` instance, not to the assembly origin or a floating connector. Keep it at the body center with ROS orientation: X forward, Y left, Z up.

After that fix, rerun:

```bash
cd ~/doggy_ws/src
onshape-to-robot doggy
check_urdf doggy/robot.urdf
```

## Runtime Architecture

Keep motor commands and visualization state separate:

```text
policy_node or gait_generator_node
  publishes /joint_angles
  12 real motor angles

motor_control_node
  subscribes /joint_angles
  sends 12 real motor angles to Teensy

joint_state_bridge_node
  subscribes /joint_angles
  computes passive knee angles from nonlinear linkage FK
  publishes /joint_states

robot_state_publisher
  subscribes /joint_states
  publishes TF from the URDF
```

The URDF should include both motor joints and passive knee joints, but the nonlinear relation should not be encoded as a URDF mimic or gear relation.

Use this mapping for the current command order:

```text
/joint_angles:
  LB1 LB2 LB3 | RB1 RB2 RB3 | RF1 RF2 RF3 | LF1 LF2 LF3

Leg names:
  LB -> RL
  RB -> RR
  RF -> FR
  LF -> FL
```

For each leg:

```text
hip_roll  = motor 1
hip_pitch = motor 2
motor3    = real third motor angle
knee      = nonlinear_linkage_fk(motor3)
```

## ROS Nodes To Add

1. `joint_state_bridge_node`

   Inputs:

   - `/joint_angles` as `std_msgs/Float32MultiArray`

   Outputs:

   - `/joint_states` as `sensor_msgs/JointState`

   Responsibilities:

   - Convert degrees to radians.
   - Rename command array indices into URDF joint names.
   - Compute `*_knee` from `*_motor3` using the MATLAB/Python linkage FK.
   - Apply per-leg sign and zero offsets.

2. `policy_node`

   Inputs:

   - `/imu`
   - `/joint_states` or motor feedback when available
   - `/cmd_vel` or `/gait_cmd`

   Outputs:

   - `/joint_angles`

   Responsibilities:

   - Run the trained policy at a fixed control rate.
   - Clamp output angles.
   - Fall back to stand pose when policy is disabled.

3. `safety_node`

   Responsibilities:

   - E-stop gate.
   - Joint limit checks.
   - Body tilt/fall detection.
   - Command timeout handling.
   - Optional torque/current/temperature checks if Teensy exposes them later.

## Calibration Before RL

Finish these before trusting sim-to-real:

1. Motor zero offsets for all 12 actuators.
2. Motor sign convention for all four legs.
3. Passive knee sign and offset:

   ```text
   urdf_knee = sign * linkage_theta4(motor3) + offset
   ```

4. IMU frame orientation against `base_link`.
5. Foot frames at the foot-tip sphere centers.
6. Joint limits checked against physical hard stops.
7. Standing pose verified in RViz and on hardware.

## Training Plan

Preferred setup:

- Train on a desktop/cloud machine with GPU.
- Deploy only the exported policy on Jetson Orin Nano.
- Keep Jetson runtime simple: ROS 2 node, inference, safety clamps, serial motor bridge.

Simulation choices:

- Isaac Lab if GPU training and terrain/domain randomization matter most.
- MuJoCo if fast iteration and easier model debugging matter most.

Policy interface:

- Observation:
  - base orientation and angular velocity
  - commanded velocity
  - 12 motor positions
  - 12 motor velocities when available
  - previous action
  - optional gait phase
  - optional foot contact states
- Action:
  - 12 target motor angles, or 12 residual offsets around a nominal gait
- Control rate:
  - start with `50 Hz`, matching the current gait generator

Rewards:

- velocity tracking
- upright body orientation
- target body height
- low energy / smooth action
- foot clearance during swing
- low foot slip during stance
- joint limit avoidance
- fall penalty

Domain randomization:

- total mass around `1.348 kg`
- per-link mass and COM offsets
- motor strength
- joint damping/friction
- ground friction
- IMU noise and bias
- command latency
- battery/Jetson payload variation

## Deployment Stages

1. URDF validation

   - Fix `frame_base_link`.
   - Export full URDF.
   - Check root link is body/base.
   - Confirm all foot frames and IMU frame appear.
   - Confirm mass and inertials are reasonable.

2. Visualization bridge

   - Add `joint_state_bridge_node`.
   - Publish `/joint_states` from existing `/joint_angles`.
   - Confirm RViz motion matches the physical leg direction.

3. Hardware baseline

   - Stand pose only.
   - Slow scripted leg sweep.
   - Low-speed gait with rosbag recording.
   - Check communication drop rate.

4. Simulation baseline

   - Import URDF/MJCF/USD.
   - Reproduce standing pose.
   - Reproduce current scripted gait.
   - Validate foot locations and joint signs.

5. RL training

   - Start with stand/balance.
   - Add commanded forward velocity.
   - Add yaw command.
   - Add terrain and pushes only after flat ground works.

6. Sim-to-real rollout

   - Policy disabled by default.
   - E-stop active.
   - Test on stand or suspended robot first.
   - Touchdown test at low action scale.
   - Increase action scale and command speed gradually.

## Bag Topics

Record these for debugging:

```bash
ros2 bag record \
  /joint_angles \
  /joint_states \
  /imu \
  /gait_cmd \
  /comm_stats \
  /teensy_ack \
  /teensy_hb
```

Add later when available:

- `/cmd_vel`
- `/motor_feedback`
- `/foot_contacts`
- `/policy_debug`
