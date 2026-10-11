# arm_calibration: intent

## Problem

A robot arm's description and the arm disagree in two ways that every pose the stack computes
inherits:

- **Joint zeros.** An actuator's zero is often set by eye (LeRobot's calibration homes each servo
  where a person judges the middle of its travel to be), so a joint reads a degree or a few off
  the angle the description means. At 30 cm of reach, a degree at the shoulder moves the tip about
  5 mm.
- **The tool centre point (TCP).** A tool mounted on the flange puts its working point somewhere
  the description does not know, so planning and servoing in the tool's point are off by however
  far the mount and the drawing differ.

## Users

- People with low-cost arms whose zeros are set by eye (the SO-ARM101 and its kind), driven by
  ros2_control, who want the arm to go where the description says.
- Integrators fitting a tool to a factory-calibrated industrial arm, who need its TCP, measured
  the way the industry does: jogging the tool onto a sharp pin in several orientations.

## Outcomes

- With a pen and a table, a user measures an arm's joint zeros in minutes, guiding the released
  arm by hand.
- With a sharp pin, a user measures a tool's TCP, jogging the arm under its own control.
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
- **Move the arm the way its users do.** Hobby arms are guided by hand, released through
  ros2_control; industrial arms are jogged under their own control and never released.

## Non-goals

- Link length or joint origin errors beyond the zeros (a full kinematic identification).
- A TCP's orientation: point touches give the tool point's position, not the turn of a tool frame.
- Writing calibration into actuators (servo homing offsets).
- Moving the arm automatically: a person always moves it, by hand or by jogging.
