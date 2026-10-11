# calib-kit: specification

Implements [intent](intent.md). The rules every calibration in the repository follows; each
calibration's own requirements are in `docs/<calibration>/spec.md`.

## Scope

The repository's layout, the contract every calibration states, what calibrations produce, and
how the repository is built and checked.

## Terms

| Term | Meaning |
|---|---|
| Calibration | One package that measures one kind of error of a robot, with its own intent and spec |
| Contract | The ROS 2 interfaces a calibration needs from a robot's stack, and what it guarantees in return |
| Profile | A YAML file of data describing one kind of robot to a calibration |
| Reference | Something fixed the robot is brought onto, whose position the calibration fits |

## Requirements

### Layout

- **KIT-LAY-1** Each calibration is a ROS 2 package in its own top-level folder, built with colcon.
- **KIT-LAY-2** Each calibration has `docs/<package>/intent.md` and `docs/<package>/spec.md`, written
  from `docs/templates/`, and a `README.md` in its package that a user can follow.
- **KIT-LAY-3** Every source file carries the Apache-2.0 header with its copyright holder.

### Robots

- **KIT-ROB-1** A calibration reaches a robot only through ROS 2 interfaces (topics, services,
  actions, the robot description) that it states as a contract in its spec and its README.
- **KIT-ROB-2** A calibration contains no code for a particular robot or actuator; robots differ
  only in profiles of data.
- **KIT-ROB-3** A calibration that changes a robot's state to do its work (releasing torque, say)
  restores it however the run ends.

### Results

- **KIT-RES-1** A calibration writes its result to files: a corrected robot description, and the
  values in YAML. It persists nothing on the robot.
- **KIT-RES-2** Every fitted value is reported and written with its uncertainty; a value the
  measurements cannot determine is refused or held, and named.
- **KIT-RES-3** The raw measurements are written too, and the calibration can fit them again
  without the robot.

### Checks

- **KIT-CHK-1** Tests check each requirement against simulated measurements whose truth is known,
  and name the requirement IDs they check in their docstrings.
- **KIT-CHK-2** `pixi run lint`, `pixi run build` and `pixi run test` pass; CI runs them on every
  push to `main` and on pull requests.
- **KIT-CHK-3** Nothing in the repository mentions or depends on an application that uses it.
