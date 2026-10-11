# calib-kit: intent

## Problem

Robots drift from their descriptions. Encoders are zeroed by eye, or lose their zero to a repair
or a collision; parts are printed or machined a little off; tools are mounted where no drawing
says; and sensors sit a few millimetres from their nominal mounts. The description a ROS 2 stack plans, transforms and simulates with is an idealised drawing
of the robot, so every pose the stack computes inherits the gap. Closing it today means a
vendor's tool for one robot, a fiducial rig, or hand-editing a URDF.

## Users

People running robots with ROS 2, from low-cost research and hobby arms to industrial ones: anyone
whose robot has drifted from its description, or has had a tool fitted. They have the robot, its
ros2_control stack and its description, and need nothing else: no measuring arm, no calibration
rig.

## Outcomes

- A user calibrates a robot in minutes, with the robot itself and an everyday reference (a pen
  dot, a sharp pin).
- The result is a corrected description, and every part of the stack that reads the description
  (planning, TF, visualisation, simulation) sees the robot as it is, with nothing new to run.
- Each value comes with how well it was measured, and anything the measurements cannot determine
  is named, never guessed.
- A robot the toolkit has never seen is supported by a short profile of data.

## Principles

- **The robot is the instrument.** Measure with the robot's own sensors and the fewest extra
  things, since every extra instrument brings its own calibration.
- **Write where the stack already looks.** Results are a corrected description and plain files;
  a calibration adds no runtime component, so nothing new can fail at run time.
- **Interfaces, not drivers.** A robot takes part through the ROS 2 interfaces its stack already
  has, stated as a contract. No code for any particular robot or actuator.
- **Honest numbers.** Every fitted value carries its uncertainty; a calibration refuses what its
  measurements cannot determine rather than returning a confident guess.
- **Leave the robot as found.** A calibration changes nothing on the robot, and restores anything
  it touches to do its work.
- **Standalone.** The toolkit depends on no application that uses it.

## Non-goals

- Drivers, or calibrations stored in actuators' own memory (homing offsets in servo EEPROM).
- Runtime correction services: the output is a description, not a node.
- Calibrations that need dedicated hardware (laser trackers, coordinate measuring arms).
- A user interface beyond a guided command line.
