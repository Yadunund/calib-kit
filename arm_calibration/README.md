# arm_calibration

Makes a robot arm's URDF match the real arm. It measures two things:

- **Joint zeros:** how far each joint's reading is from the angle the URDF means. Zeros set by
  eye (as LeRobot's calibration does), or lost after a motor or encoder change, are often a few
  degrees off. At 30 cm of reach, one degree at the shoulder moves the tip about 5 mm.
- **Tool centre point (TCP):** where a tool's working point is on the arm.

You touch fixed points with the arm's tip, holding the tool a few different ways. The tool fits
the values that make the URDF agree with those touches, and writes a corrected URDF.

| Your arm | Use | You need |
|---|---|---|
| Zeros set by eye, or lost after a repair | `--solve zeros` | a pen and a flat table |
| Same, and a tool is fitted | `--solve zeros+tcp` | a pen and a flat table |
| Zeros fine, new tool fitted | `--solve tcp --reference pin` | a sharp pin fixed in reach |

## Calibrate joint zeros (SO-ARM101)

1. Clamp the arm flat to a table. Tape a sheet of paper under its reach. Have a fine pen ready.
2. Start the arm's ros2_control stack, then run:

   ```bash
   ros2 run arm_calibration calibrate_arm --out ~/so101_cal
   ```

3. **Hold the arm and press Enter.** Its motors switch off and it will drop if you let go. Say
   whether it now moves freely; if not, the tool restores the arm and stops.
4. The tool says how far from the base the dots can go, and where to put each of the three.
5. **For each dot:**
   1. Rest the closed jaw tips on the paper where the dot will go, gripper pointing straight down.
      The line under the prompt shows how far the gripper can tilt there. Press Enter when it says
      `GOOD SPOT`.
   2. Lift the gripper a little, mark a small dot (about 1 mm) where the tips were, and put the
      tips back in it.
   3. Touch the dot three ways, keeping the tips in the dot:
      - **straight down;**
      - **tilted towards the base:** the top of the gripper moves towards the base;
      - **tilted away from the base:** the top of the gripper moves away from it.

      Tilt as far as the arm goes (at least 15 degrees). Hold still and press Enter for each. The
      line under the prompt shows the current tilt. A touch tilted the wrong way, or too little, is
      not recorded, and the tool says what to change.
6. The tool prints the result and restores the arm.

At each prompt: **Enter** records, **r** redoes the last touch, **s** skips this one, **q**
finishes. If a dot's touches disagree by more than 20 mm, the tips probably slipped out of the
dot: the tool offers to redo that dot.

Then use `~/so101_cal/so101_calibrated.urdf` as the robot description.

## Calibrate a tool centre point (pin)

Fix a sharp pin within reach. Then run, with your arm's profile:

```bash
ros2 run arm_calibration calibrate_arm --profile my_arm.yaml --solve tcp --reference pin --out ~/my_arm_cal
```

The arm stays under its own control: jog it (pendant, teleoperation) until the tool's point meets
the pin's, in each way the tool asks. Turn the tool a little about its own axis between touches.

## The result

```text
9 touches on 3 dots; the closed jaw tips 0.58 mm rms from them after the fit.
  shoulder_lift_joint       +1.74 deg  (+- 0.23)
  elbow_flex_joint          -2.31 deg  (+- 0.35)
  wrist_flex_joint          +2.86 deg  (+- 0.32)
  shoulder_pan_joint       held: its axis is the table's normal through the base, so turning it turns every dot with it
  wrist_roll_joint         held: its axis passes almost through the closed tips, so turning it barely moves them
```

Each value comes with its uncertainty (one standard deviation). A value the touches could not pin
down is marked "poorly measured", and a touch more than 2 mm off is named so you can redo it.
Zeros from a pin, or from fewer than three dots, are refused.

Files written to `--out`:

| File | Holds |
|---|---|
| `<profile>_calibrated.urdf` | Your URDF with the zeros and TCP folded in |
| `joint_zeros.yaml` | Each zero offset in radians (add it to the joint's reading) and its uncertainty |
| `tcp.yaml` | The TCP in `tip_link`, metres, its uncertainty, and the link it becomes |
| `touches.json`, `description.urdf` | The raw touches and the URDF they were read against |

To fit recorded touches again, without the robot:

```bash
ros2 run arm_calibration calibrate_arm --refit ~/so101_cal/touches.json --out ~/so101_cal
```

To fold the result into another URDF (one built with other xacro arguments, say):

```bash
ros2 run arm_calibration apply_calibration --urdf arm.urdf --zeros joint_zeros.yaml --tcp tcp.yaml --out arm_calibrated.urdf
```

Zeros hold until the actuators' own calibration changes; recalibrate after it does. A TCP holds
for the tool as mounted.

## What the robot must provide

Any ros2_control arm works; there is no code for a particular actuator. Topic and service names
below are defaults, each with a flag.

| What | Interface | Needs |
|---|---|---|
| URDF | `/robot_description` (`std_msgs/msg/String`, transient-local), or `--urdf` | The chain from the profile's `base_link` to `tip_link`, with joint limits |
| Joints | `/joint_states` (`sensor_msgs/msg/JointState`) | Every chain joint by name, in radians, also while the arm is released |
| Controller manager | `/controller_manager` services: `list_hardware_components`, `list_controllers`, `switch_controller`, `set_hardware_component_state` | Only to release the arm |
| Release | The `<ros2_control>` hardware holding the chain's joints | **When inactive, its motors have no torque** and it keeps reading joint positions |

How the arm is moved (`--release`):

- `lifecycle` (default for dots): stops the controllers that command the arm, takes its hardware
  inactive so you can move it by hand, and always restores both at the end, however the run ends.
  `--ros2-control NAME` picks the hardware if more than one holds the joints.
- `none` (default for a pin): changes nothing on the robot. You move the arm with its own controls,
  or free it some other way.

Nothing is written to the robot; the corrected URDF is a new file.

## Arms

An arm is a profile: a YAML file. Shipped profiles are in
[`arm_calibration/arms/`](arm_calibration/arms) and named by file name (`--profile so101`, the
default); any other is given by its path.

```yaml
name: so101
base_link: base_link
tip_link: gripper_frame_link  # the frame nearest the point that touches
tip: the closed jaw tips      # what touches, in words
tip_offset: [0, 0, 0.0063]    # where it is in tip_link, metres
tool_axis: [0, 0, 1]          # tip_link's axis pointing out of the tool
solve: zeros                  # zeros, tcp or zeros+tcp
reference: dots               # dots or pin
free_joints: [shoulder_lift_joint, elbow_flex_joint, wrist_flex_joint]
held_joints:                  # joints touches cannot measure, and why
  shoulder_pan_joint: its axis is the table's normal through the base
  wrist_roll_joint: its axis passes almost through the closed tips
plane_normal: [0, 0, 1]       # the table's normal in the base frame
dots: 3
shapes: [down, towards, away] # the wrist cannot tilt sideways, so no left or right
min_lean_deg: 15              # each tilt at least this far
```

A tool on an industrial arm needs little more than its chain:

```yaml
name: my_arm_with_torch
base_link: base_link
tip_link: tool0
tip: the torch's wire tip
tip_offset: [0, 0, 0.25]      # roughly, from the tool's drawing
solve: tcp
reference: pin
tcp_link: torch_tcp
```

A shape is `down`, `towards`, `away`, `left` or `right`: which way the top of the tool moves.
Left and right are as seen from the base, looking out at the dot. List a shape twice to touch it
twice. A value that comes out poorly measured on a new arm should be held in its profile.

## Tests

```bash
pixi run test   # from the repository root; or pytest in a sourced ROS 2 workspace
```
