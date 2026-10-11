# arm_calibration: intent

## Problem

A robot arm's description and the arm disagree in two ways that every pose the stack computes
inherits:

- **Joint zeros.** A joint reads off the angle the description means when its zero was never set
  precisely (LeRobot's calibration homes each servo where a person judges the middle of its travel
  to be), or when it was lost: a motor or encoder replaced, an encoder battery run flat, a
  collision. At 30 cm of reach, a degree at the shoulder moves the tip about 5 mm; at an
  industrial arm's metre, nearly 20 mm.
- **The tool centre point (TCP).** A tool mounted on the flange puts its working point somewhere
  the description does not know, so planning and servoing in the tool's point are off by however
  far the mount and the drawing differ.

## Users

Anyone running an arm with ros2_control, from a low-cost arm to an industrial one, who needs it
to go where its description says:

- whose arm's zeros are off: set by eye on a low-cost arm, or lost on any arm after a repair;
- who has mounted a tool and needs its TCP, measured the way the industry does: jogging the tool
  onto a sharp pin in several orientations.

## Outcomes

- With dots on a table, a user measures an arm's joint zeros in minutes, however the arm is moved:
  released and guided by hand, guided in its own freedrive, or jogged.
- With a sharp pin, a user measures a tool's TCP, jogging the arm under its own control or
  guiding it.
- The arm's description comes out corrected: joint origins turned by their zeros, and the TCP as
  a link, so the rest of the stack uses them with no change.
- Every value comes with its uncertainty; what the touches cannot determine is held and named.
- A new arm is supported by writing its profile.

## Principles

- **Touches, not targets.** The arm's tip on a fixed reference is the whole measurement; the
  references' positions are fitted, so nothing needs measuring.
- **The table is a reference too.** Dots on one plane are what make a lifting shoulder's zero
  measurable without measuring anything.
- **Refuse what cannot be measured.** Zeros from a single pin, or from too few dots, are refused
  rather than returned.
- **Move the arm however it moves.** The measurement is the same whether the arm is released
  through ros2_control and guided by hand, guided in its own freedrive, or jogged under its own
  control from a pendant or teleoperation; the calibration releases nothing it does not have to.

## Non-goals

- Link length or joint origin errors beyond the zeros (a full kinematic identification).
- A TCP's orientation: point touches give the tool point's position, not the turn of a tool frame.
- Writing calibration into actuators (servo homing offsets).
- Moving the arm automatically: a person always moves it, by hand or by jogging.
