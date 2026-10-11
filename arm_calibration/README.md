# arm_calibration

Joint zero calibration for robot arms: how far each joint's reading is from the angle the arm's
description means, measured with the arm itself and a pen, and folded into its URDF.

An actuator's zero is often set by eye. LeRobot's calibration, for example, sets it where a
person judges the middle of each joint's travel to be. A joint then reads a degree or a few off
its description's angle, and the tip lands millimetres from where the description puts it: about
5 mm for each degree at a shoulder, at 30 cm of reach.

## How it works

The arm's torque is off. Its tip is rested by hand on a few dots marked on the table, each dot in
several arm shapes. Every touch of one dot puts the tip on the same point, whatever the joints
read. So the zero offsets are the ones that make the description agree with itself about where
each dot is.

- **The dots are not measured.** Their positions are fitted too.
- **They lie on one plane:** the table's, with a known normal (the base's own z axis, for an
  arm clamped flat to the table) and a fitted height. That plane is what fixes a shoulder that
  lifts the arm.
- **Where the tip is** in the description's tip frame is fitted as well, near where the profile
  puts it (`tip_offset`).

Some zeros no touch can show, and an arm's profile holds them:

- a joint whose axis is the table's normal through the base: turning it turns every dot with it;
- a joint whose axis passes through the tip: turning it does not move the tip.

Every fitted zero comes with its uncertainty. One the touches barely turn is named as poorly
measured.

## Contract

The calibration works with any arm driven by ros2_control. It needs nothing from the arm beyond
what such a stack already offers; there is no code here for any particular actuator.

**The robot provides** (names are the defaults; each has a flag):

| What | Interface | Expectation |
|---|---|---|
| The description | `/robot_description`, `std_msgs/msg/String`, transient-local (as `robot_state_publisher` publishes it) | Holds the chain from the profile's `base_link` to its `tip_link`. `--urdf` gives a file instead. |
| The joints | `/joint_states`, `sensor_msgs/msg/JointState` | Every joint of that chain by name, `position` in radians in the description's convention, published while the arm is released (`joint_state_broadcaster` does this). |
| The controller manager | `/controller_manager`: `list_hardware_components`, `list_controllers`, `switch_controller`, `set_hardware_component_state` (`controller_manager_msgs/srv/...`) | As ros2_control's own. |
| Releasing the arm | The ros2_control hardware holding the chain's joints (a `<ros2_control>` block of the description) | **Inactive, it applies no torque, so the arm moves by hand, and keeps reading its state interfaces**, so `/joint_states` keeps updating. `feetech_ros2_driver` does both. |

**The calibration, with `--release lifecycle` (the default):**

1. Finds the ros2_control hardware (the controller manager's "hardware component") whose state
   interfaces hold every joint of the chain, or the one named by `--ros2-control NAME` (the `name`
   of its `<ros2_control>` block). It must be active.
2. Asks the person to hold the arm, then deactivates exactly the active controllers that claim
   its command interfaces (strictly), and takes the hardware to inactive.
   Controllers that only read, like `joint_state_broadcaster`, keep running.
3. Checks `/joint_states` is still updating. If not, it restores and stops: such an arm must be
   freed by other means and calibrated with `--release none`.
4. However it ends (finished, quit, error, Ctrl-C), takes the hardware back to active and
   reactivates those controllers. Activation seeds the commands with where the arm is, so nothing
   jumps.

With `--release none`, the arm is freed by other means (a teach button, gravity compensation) and
the calibration only reads.

**It produces** files, and changes nothing on the robot: the description it read is corrected
in a copy, `<ros2_control>` tags and all, and only joint origins and limits change.

## Calibrating

With the arm's ros2_control stack running, clamp the arm flat to a table, and mark three dots
within reach with a pen: one near the base, one far out in front, one to the side.

```bash
ros2 run arm_calibration calibrate_zeros --out ~/so101_zeros                                    # SO-ARM101
ros2 run arm_calibration calibrate_zeros --profile path/to/my_arm.yaml --out ~/my_arm_zeros   # any arm
```

- **The arm is released after it asks you to hold it,** and restored at the end.
- **For each dot, five shapes:** the tool straight down, then leaning about 40 degrees towards
  the base, away from it, and to either side. Rest the tip in the dot, hold still, press Enter.
  The tool says how the tool leans in each touch, so you can check the shape. A shape the arm
  cannot take at a dot can be skipped.
- **The result** is each zero with its uncertainty, and how far the tip was from its dots. A
  touch more than 2 mm off is named, so you can redo it; a poorly measured zero is named too.

It writes, in `--out`:

- **`joint_zeros.yaml`:** each offset in radians (add it to the joint's reading for the
  description's angle), its uncertainty, the tip's rms distance from its dots, and what was read
  (topics, the ros2_control hardware's name).
- **`<profile>_calibrated.urdf`:** the description with each offset folded in. Each joint's
  origin is turned by its offset about the joint's axis, and its limits shifted so they bound the
  same physical travel.
- **`touches.json` and `description.urdf`:** every touch and the description it was read
  against. `--refit touches.json` fits them again with no robot.

The offsets hold for the actuators' own calibration they were measured with (a Feetech arm's
homing offsets, say). After recalibrating the actuators, calibrate again.

To fold a `joint_zeros.yaml` into another URDF (one generated with other xacro arguments, say):

```bash
ros2 run arm_calibration apply_zeros --urdf arm.urdf --zeros ~/so101_zeros/joint_zeros.yaml --out arm_calibrated.urdf
```

## Arms

An arm is a profile: a YAML file of data, given by its path (`--profile path/to/arm.yaml`), or
one shipped in [`arm_calibration/arms/`](arm_calibration/arms), named by its stem. The default
is `so101`.

| Arm | Profile |
|---|---|
| SO-ARM101 | `so101` |

```yaml
name: so101
description: SO-ARM101, Feetech STS3215 servos homed with LeRobot's calibration
base_link: base_link
tip_link: gripper_frame_link  # the frame nearest the point that touches the dots
tip: the closed jaw tips      # what the person rests in the dot
tip_offset: [0, 0, 0.0063]    # where it is in tip_link, metres (fitted, give or take a few mm)
tool_axis: [0, 0, 1]          # the tip frame's axis pointing out of the tool
free_joints: [shoulder_lift_joint, elbow_flex_joint, wrist_flex_joint]
held_joints:                  # the rest of the chain's joints, and why
  shoulder_pan_joint: its axis is the table's normal through the base
  wrist_roll_joint: its axis passes almost through the closed tips
plane_normal: [0, 0, 1]       # the table's normal in the base frame
dots: 3
tilt_deg: 40
```

To add an arm: bring it up with ros2_control and write its profile. A zero the touches cannot
measure is reported with a large uncertainty; hold it in the profile.

## Tests

```bash
pixi run test   # from the repository's root; or pytest, in a sourced ROS 2 workspace
```
