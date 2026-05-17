# doggy_ws

## Overview

A 12-DOF quadruped robot project focused on kinematics, servo control, and trajectory generation.

Doggy combines a custom mechanical design, STS-series serial bus servos, inverse/forward kinematics, ROS 2 control nodes, and an Onshape-to-URDF workflow for visualization and future simulation/RL deployment.

## My role

- Designed and implemented the IK/FK pipeline
- Calibrated STS3032 servos
- Generated foot trajectories for standing, walking, and trotting tests
- Integrated control logic with the robot hardware
- Built the ROS 2 workspace for gait control, motor communication, URDF export, and deployment testing

## System architecture

- Hardware: 12-DOF quadruped frame, STS3032 serial bus servos, Teensy motor controller, Jetson Orin Nano, IMU, battery, and custom 3D/CAD components
- Software: ROS 2 Python nodes for keyboard control, gait generation, motor communication, joint-state bridging, safety gating, and RL deployment scaffolding
- Communication: Jetson publishes motor angle commands over USB serial to Teensy; Teensy sends STS bus packets to the servos; feedback bring-up is being tested through a direct STS feedback test node
- Control loop: foot trajectory generation -> IK solver -> 12 motor angle commands -> Teensy serial bridge -> STS3032 servo bus

## Demo

Add videos, GIFs, or images here:

```md
![Doggy CAD overview](docs/cad/doggy-cad-overview.png)
![Doggy front](docs/photos/doggy-front.jpg)
![Doggy side](docs/photos/doggy-side.jpg)
```

Media folders:

- CAD files and screenshots: [`docs/cad/`](docs/cad/)
- Robot photos: [`docs/photos/`](docs/photos/)

## Challenges

- Multiple IK solutions and continuity selection
- Nonlinear calf/knee linkage driven by the third motor
- STS3032 servo calibration and sign convention matching
- Real-world trajectory stability under open-loop control
- Mapping CAD/Onshape joint frames into a usable URDF

## Results

- Robot can stand with commanded servo angles
- IK validated with circular and sinusoidal foot trajectories
- Workspace and reachable foot motion were defined from the kinematic model
- Full four-leg URDF export now includes 16 DOFs, `base_link`, `imu`, and four foot frames
- ROS 2 nodes are available for gait generation, motor communication, joint-state bridging, safety gating, and STS feedback testing

## Next steps

- Open-loop walking validation on hardware
- ROS/URDF visualization and joint-state calibration
- STS3032 feedback readback through Teensy or direct servo bus adapter
- Vision / closed-loop control
- RL policy training and sim-to-real deployment

## Build

```bash
cd ~/doggy_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Scripted gait bring-up:

```bash
ros2 launch quadruped_bringup bringup.launch.py serial_port:=/dev/ttyACM0 baud_rate:=115200
```

RL deployment scaffold:

```bash
ros2 launch quadruped_bringup rl_bringup.launch.py
```

STS feedback direct-bus test:

```bash
ros2 run quadruped_control sts_feedback_test_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=1000000 \
  -p servo_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]"
```

## Repository layout

```text
doggy_ws/
+-- docs/
|   +-- cad/                  # CAD screenshots, Onshape notes, exported CAD files
|   +-- photos/               # Robot build photos
|   +-- rl_workflow.md        # RL deployment and bring-up notes
+-- src/
|   +-- doggy/                # Onshape-generated URDF, meshes, and config
|   +-- quadruped_bringup/    # ROS 2 launch files
|   +-- quadruped_control/    # Gait, IK, safety, feedback, and motor nodes
```
