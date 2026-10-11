# arm_calibration

Calibrates a robot arm against its URDF, with the arm itself and a fixed reference point:

- **Joint zeros:** how far each joint's reading is from the angle the description means. An
  actuator's zero is often set by eye (LeRobot's calibration sets it where a person judges the
  middle of each joint's travel to be), and a joint then reads a degree or a few off; at 30 cm of
  reach, a degree at the shoulder moves the tip about 5 mm.
- **Tool centre point (TCP):** where a tool's working point is on the arm. A tool mounted on the
  flange puts its point somewhere the description does not know; the TCP is added to the URDF as
  a link, so planning and servoing can work in the tool's own point.

| Arm | Calibrate | Reference | How the arm moves |
|---|---|---|---|
| Zeros set by eye (hobby and research arms) | `zeros` | pen dots on the table | released, guided by hand |
| Zeros set by eye, new tool | `zeros+tcp` | pen dots on the table | released, guided by hand |
| Factory-calibrated (industrial arms), new tool | `tcp` | a sharp pin | jogged under its own control |

## How it works

The arm's tip (a gripper's closed jaws, a tool's point) is brought onto a fixed reference several
times, the arm in a different shape each time: the tool straight down, then leaning towards the
base, away from it and to either side. Every touch of one reference puts the tip on the same
point, whatever the joints read. So the joint zeros, and where the tip is on the arm, are the
ones that make the description agree with itself about where each reference is.

The references are not measured; their positions are fitted too.

- **Pen dots** lie on the table the arm is clamped to, so on one plane, with a known normal (the
  base's own z axis for an arm clamped flat) and a fitted height. That plane is what fixes a
  shoulder that lifts the arm, so joint zeros need dots: three or more, spread over the arm's
  reach (near the base, far out in front, to either side).
- **A sharp pin**, fixed anywhere within reach, is the industrial way to find a TCP: the tool's
  point is jogged onto the pin's in four or more orientations. A pin fixes the TCP, not the
  zeros.

Some zeros no touch can show, and an arm's profile holds them:

- a joint whose axis is the table's normal through the base: turning it turns every reference
  with it;
- a joint whose axis passes through the tip: turning it does not move the tip.

Touches give the TCP's position, not the turn of a tool frame about it: the TCP link takes the
axes of the link it is fixed to.

Every fitted value comes with its uncertainty, and one the touches barely constrain is named as
poorly measured.

## Contract

The calibration works with any arm driven by ros2_control. It needs nothing from the arm beyond
what such a stack already offers; there is no code here for any particular actuator.

**The robot provides** (names are the defaults; each has a flag):

| What | Interface | Expectation |
|---|---|---|
| The description | `/robot_description`, `std_msgs/msg/String`, transient-local (as `robot_state_publisher` publishes it) | Holds the chain from the profile's `base_link` to its `tip_link`. `--urdf` gives a file instead. |
| The joints | `/joint_states`, `sensor_msgs/msg/JointState` | Every joint of that chain by name, `position` in radians in the description's convention (`joint_state_broadcaster` does this), also while the arm is released. |
| The controller manager | `/controller_manager`: `list_hardware_components`, `list_controllers`, `switch_controller`, `set_hardware_component_state` (`controller_manager_msgs/srv/...`) | As ros2_control's own. Needed only to release the arm. |
| Releasing the arm | The ros2_control hardware holding the chain's joints (a `<ros2_control>` block of the description) | **Inactive, it applies no torque, so the arm moves by hand, and keeps reading its state interfaces**, so `/joint_states` keeps updating. `feetech_ros2_driver` does both. |

**How the arm moves** (`--release`; the default follows the reference):

- **`lifecycle`** (the default for dots): the arm is released and guided by hand.
  1. Finds the ros2_control hardware (the controller manager's "hardware component") whose state
     interfaces hold every joint of the chain, or the one named by `--ros2-control NAME` (the
     `name` of its `<ros2_control>` block). It must be active.
  2. Asks the person to hold the arm, then deactivates exactly the active controllers that claim
     its command interfaces (strictly), and takes the hardware to inactive. Controllers that only
     read, like `joint_state_broadcaster`, keep running.
  3. Checks `/joint_states` is still updating. If not, it restores and stops: such an arm must be
     freed by other means and calibrated with `--release none`.
  4. However it ends (finished, quit, error, Ctrl-C), takes the hardware back to active and
     reactivates those controllers. Activation seeds the commands with where the arm is, so
     nothing jumps.
- **`none`** (the default for a pin): the calibration only reads. The arm is moved under its own
  control (a teach pendant, teleoperation, servoing), or freed by other means (a teach button,
  gravity compensation).

**It produces** files, and changes nothing on the robot: the description it read is corrected in
a copy, `<ros2_control>` tags and all.

## Calibrating

With the arm's ros2_control stack running:

```bash
# Joint zeros, from five pen dots on the table, the arm guided by hand (the SO-ARM101's default).
ros2 run arm_calibration calibrate_arm --out ~/so101_cal

# A TCP, from a sharp pin, the arm jogged under its own control.
ros2 run arm_calibration calibrate_arm --profile my_arm.yaml --solve tcp --reference pin --out ~/my_arm_cal

# Both, from dots.
ros2 run arm_calibration calibrate_arm --profile my_arm.yaml --solve zeros+tcp --reference dots --out ~/my_arm_cal
```

- **For each reference, the profile's shapes:** the tool straight down, then leaning towards the
  base, away from it, and to either side (by default 40 degrees each). An arm whose wrist cannot
  lean sideways leans twice each way instead. Turning the tool about its own axis between touches
  helps too. Bring the tip onto the reference, hold still, press Enter. The tool says how
  the tool leans in each touch, so you can check the shape; a shape the arm cannot take there can
  be skipped.
- **The result** is each value with its uncertainty, and how far the tip was from its
  references. A touch more than 2 mm off is named, so you can redo it.
- **What cannot be measured is refused:** zeros from a pin, or from fewer than three dots.

It writes, in `--out`:

- **`joint_zeros.yaml`** (`zeros`): each offset in radians (add it to the joint's reading for the
  description's angle) and its uncertainty.
- **`tcp.yaml`** (`tcp`): the TCP's position in `tip_link`, metres, its uncertainty on each
  axis, and the link it becomes.
- **`<profile>_calibrated.urdf`:** the description with the calibration folded in. Each joint's
  origin is turned by its zero offset about the joint's axis, its limits shifted so they bound the
  same physical travel; the TCP is a link fixed to `tip_link` (`tcp_link` by default), moved
  there if the description already has it.
- **`touches.json` and `description.urdf`:** every touch and the description it was read
  against. `--refit touches.json` fits them again with no robot.

Joint zeros hold for the actuators' own calibration they were measured with (a Feetech arm's
homing offsets, say): after recalibrating the actuators, calibrate again. A TCP holds for the tool
as mounted.

To fold a calibration into another URDF (one generated with other xacro arguments, say):

```bash
ros2 run arm_calibration apply_calibration --urdf arm.urdf --zeros joint_zeros.yaml --tcp tcp.yaml --out arm_calibrated.urdf
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
tip_link: gripper_frame_link  # the frame nearest the point that touches the references
tip: the closed jaw tips      # what touches them
tip_offset: [0, 0, 0.0063]    # where it is expected in tip_link, metres
tool_axis: [0, 0, 1]          # tip_link's axis pointing out of the tool
solve: zeros                  # zeros, tcp or zeros+tcp
reference: dots               # dots or pin
free_joints: [shoulder_lift_joint, elbow_flex_joint, wrist_flex_joint]
held_joints:                  # the rest of the chain's joints, and why
  shoulder_pan_joint: its axis is the table's normal through the base
  wrist_roll_joint: its axis passes almost through the closed tips
plane_normal: [0, 0, 1]       # the table's normal in the base frame
dots: 5
shapes: [down, towards 20, towards 40, away 20, away 40]  # its wrist cannot lean sideways
tilt_deg: 40                  # the lean of a shape given without one
```

An industrial arm with a tool needs only its chain and what to calibrate, for example:

```yaml
name: my_arm_with_torch
base_link: base_link
tip_link: tool0               # the flange
tip: the torch's wire tip
tip_offset: [0, 0, 0.25]      # roughly where the tool's point is, from its drawing
solve: tcp
reference: pin
tcp_link: torch_tcp
```

To add an arm: bring it up with ros2_control and write its profile. A value the touches cannot
measure is reported with a large uncertainty; hold it in the profile.

## Tests

```bash
pixi run test   # from the repository's root; or pytest, in a sourced ROS 2 workspace
```
