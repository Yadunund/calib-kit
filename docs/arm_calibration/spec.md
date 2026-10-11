# arm_calibration: specification

Implements [intent](intent.md), under the repository's [rules](../spec.md).

## Scope

`calibrate_arm` measures an arm's joint zeros, its tool centre point, or both, from touches on
fixed references, and writes them as YAML and as a corrected URDF. `apply_calibration` folds those
files into any URDF. Moving the arm is the person's; the arm's drivers are ros2_control's.

## Terms

| Term | Meaning |
|---|---|
| Chain | The URDF's joints from the profile's `base_link` to its `tip_link` |
| Tip | The point that touches the references, at `tip_offset` in `tip_link` (the profile's `tip` names it) |
| Zero offset | What to add to a joint's reading for the angle the description means, radians |
| TCP | The tool centre point: the tip's position in `tip_link`, measured |
| Reference | `dots` (pen dots on the table) or `pin` (a sharp fixed point) |
| Touch | The joint readings with the tip on a reference |
| Shape | How the tool is held for a touch: straight down, or leaning towards the base, away from it, left or right, by some degrees |
| Hardware | The ros2_control hardware component holding the chain's joints, named by its `<ros2_control>` block |

## Requirements

### Interfaces

- **ARM-IF-1** The description is read from `/robot_description` (`std_msgs/msg/String`,
  transient-local), or from a file given with `--urdf`.
- **ARM-IF-2** Joint positions are read from `/joint_states` (`sensor_msgs/msg/JointState`), by
  name, in radians, in the description's convention.
- **ARM-IF-3** The controller manager is reached at `/controller_manager` through
  `list_hardware_components`, `list_controllers`, `switch_controller` and
  `set_hardware_component_state`; it is needed only to release the arm.
- **ARM-IF-4** Each topic and the controller manager's namespace can be given on the command line.

### Releasing the arm

- **ARM-REL-1** With `--release lifecycle`, the hardware is the one whose state interfaces hold
  `<joint>/position` for every joint of the chain, or the one named by `--ros2-control NAME`.
  None or several matching, or a named one missing, is refused with the hardware there is.
- **ARM-REL-2** The person is asked to hold the arm before it is released. Releasing deactivates,
  strictly, exactly the active controllers that claim any of the hardware's command interfaces,
  then takes the hardware inactive; controllers that only read keep running. Hardware that is not
  active is refused.
- **ARM-REL-3** After releasing, joint states with a newer stamp must arrive within 2 s; otherwise
  the arm is restored and the run stops, naming `--release none`.
- **ARM-REL-4** However the run ends (finished, quit, error, interrupt), released hardware is taken
  back to active and the controllers deactivated are activated again.
- **ARM-REL-5** With `--release none`, nothing on the robot is changed: the arm is jogged under its
  own control or freed by other means. The default is `lifecycle` for dots and `none` for a pin.

### Recording

- **ARM-REC-1** For each reference (the profile's `dots` for dots, one pin by default; `--count`
  sets it), the person is asked for each of the profile's `shapes` in turn, and may skip a shape or
  finish early.
- **ARM-REC-2** A touch is the mean of 15 readings of every joint of the chain, 30 ms apart. If any
  joint moves more than 0.005 rad over them, the touch is not taken and the person is asked to
  hold still.
- **ARM-REC-3** After each touch, the tool's lean is reported as the description has it: degrees
  from straight down, and which way.
- **ARM-REC-4** The description is written to `description.urdf` before the first touch, and every
  touch to `touches.json` as it is taken, with the profile, reference, solve and what was read.

### Fit

- **ARM-FIT-1** The tip is placed by the chain's forward kinematics with each joint at its reading
  plus its zero offset. The fit minimises each touch's distance from its reference over 0.5 mm,
  with a robust (soft L1) loss.
- **ARM-FIT-2** Dots lie on one plane whose normal in the base frame is the profile's
  `plane_normal`; the plane's height and each dot's place on it are fitted.
- **ARM-FIT-3** Each pin is a point anywhere, its position fitted.
- **ARM-FIT-4** Zero offsets are fitted for the profile's `free_joints` when `--solve` includes
  `zeros`, each held weakly to none (10 degrees, one standard deviation); every other joint reads
  as it is.
- **ARM-FIT-5** The tip is expected at the profile's `tip_offset`. When `--solve` includes `tcp` it
  is fitted freely (1 m); otherwise it is held within 3 mm (one standard deviation).
- **ARM-FIT-6** Each fitted value's uncertainty is one standard deviation from the fit's normal
  matrix, the touches' noise taken as the larger of the fit's rms and 0.5 mm. With touches whose
  noise matches that, each fitted value's error is within three of its standard deviations.
- **ARM-FIT-7** With the shapes an arm can take on each of its dots (five shapes on three dots for
  an arm whose wrist leans every way, two leans each way on five dots for one whose wrist only
  pitches), a tip settling 0.5 mm from each dot's centre and offsets of up to 3 degrees, every zero
  with a standard deviation under 0.25 degrees is fitted to within 0.5 degrees, and a free TCP to
  within 1.5 mm. With those shapes on one pin, the tool turned about its axis between touches and
  the tip settling 0.2 mm from the pin's point, the TCP is fitted to within 1.5 mm.

### Refusals

- **ARM-NO-1** Zeros are refused from a pin, and from fewer than three dots, before any touch.
- **ARM-NO-2** A reference touched fewer than twice is refused by the fit.
- **ARM-NO-3** A run with no touches writes nothing and says so.

### Report

- **ARM-REP-1** The report gives the touches and references, the tip's rms distance from them, each
  zero with its standard deviation in degrees, the held joints with why, and the tip (with its
  standard deviation on each axis, in millimetres, when it is the TCP).
- **ARM-REP-2** A zero with a standard deviation over 0.5 degrees, or a TCP with one over 0.5 mm on
  any axis, is named as poorly measured; a touch over 2 mm from its reference is named for redoing.

### Outputs

- **ARM-OUT-1** With `zeros`, `joint_zeros.yaml` holds the zero offsets (`joint_zeros_rad`), their
  standard deviations, the held joints, the tip, the touches' rms, the reference, what was read
  and when.
- **ARM-OUT-2** With `tcp`, `tcp.yaml` holds the TCP (`xyz_m` in `parent`, the tip link), its
  standard deviation on each axis, the link it becomes (`link`), the touches' rms, the reference,
  what was read and when.
- **ARM-OUT-3** `<profile>_calibrated.urdf` is the description read, with the zeros folded in: each
  joint's origin turned by its offset about its axis and its limits shifted by the offset's
  negative, so that the corrected description at a reading is the original at the reading plus
  the offset, and the limits bound the same travel.
- **ARM-OUT-4** With `tcp`, the TCP is a link (the profile's `tcp_link`) on a fixed joint from the
  tip link at the measured position, with the tip link's axes. A link of that name already fixed
  to the tip link is moved; one of that name elsewhere is refused.
- **ARM-OUT-5** The calibrated URDF is built from the values as written to the YAML files, so
  `apply_calibration` with those files reproduces it.
- **ARM-OUT-6** `--refit touches.json` fits recorded touches again against `description.urdf`
  beside it (or `--urdf`), with the reference and solve recorded unless given.

### Configuration

- **ARM-CFG-1** A profile is a YAML file with `name`, `base_link`, `tip_link` and, optionally,
  `description`, `tip`, `tip_offset`, `tool_axis`, `solve` (`zeros`, `tcp` or `zeros+tcp`),
  `reference` (`dots` or `pin`), `free_joints`, `held_joints`, `tcp_link`, `plane_normal`, `dots`,
  `shapes` and `tilt_deg`. A shape is a direction (`down`, `towards`, `away`, `left`, `right`) and
  an optional lean in degrees (`tilt_deg` otherwise); the default is the five directions. Unknown
  settings, an unknown `solve` and an unknown shape are refused.
- **ARM-CFG-2** Profiles shipped in `arm_calibration/arms/` are named by their file's stem; any
  other is given by its path. The default is `so101`.
- **ARM-CFG-3** An arm is added by its profile alone.
- **ARM-CFG-4** `so101` fits the shoulder lift, elbow and wrist flex zeros from five dots, each
  touched straight down and leaning 20 and 40 degrees towards the base and away from it (its wrist
  cannot lean sideways); holds the base pan and wrist roll; and expects the closed jaw tips 6.3 mm
  beyond `gripper_frame_link` along its z axis.
