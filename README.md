# doggy_ws

ROS 2 workspace for the Doggy quadruped.

## Packages

- `quadruped_control`: gait generation, keyboard control, motor serial bridge.
- `quadruped_bringup`: launch files for the quadruped control stack.
- `doggy`: Onshape-generated URDF, meshes, and export config.

## Build

```bash
cd ~/doggy_ws
colcon build --symlink-install
source install/setup.bash
```

## Onshape URDF Export

Create `src/.env` from `src/.env.example` and fill in the Onshape API keys, then:

```bash
cd ~/doggy_ws/src
onshape-to-robot doggy
check_urdf doggy/robot.urdf
```
