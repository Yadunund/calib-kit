# Copyright 2026 Yadunund Vijay
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Find an arm's joint zero offsets against its description, by touching dots on the table.

An actuator's zero is often set by eye (LeRobot's calibration sets it where a person judges the
middle of each joint's travel to be), so a joint reads a degree or a few off the angle its
description means, and the arm's tip lands millimetres from where the description puts it. This
finds those offsets (`fit`) and writes them as a YAML file and as a corrected description.

With the arm's ros2_control stack running and the arm clamped flat to a table:

1. Mark dots on the table with a pen, within the arm's reach: one near the base, one far out in
   front, one to the side.
2. With the torque off, hold the arm so its tip rests in a dot. Do each dot in five shapes: the
   tool straight down, then leaning towards the base, away from it, and to either side. Hold still
   and press Enter for each.
3. The fit prints each zero with its uncertainty and how far the tip was from its dots, and writes
   `joint_zeros.yaml` and the corrected URDF.

    calibrate_zeros --release                                   # the SO-ARM101, ros2_control running
    calibrate_zeros --profile my_arm.yaml --release
    calibrate_zeros --refit ~/arm_zeros/touches.json            # no arm needed

What the robot must offer is the contract in README.md: the description and joint states on
topics, and a controller manager that releases the arm by taking its ros2_control hardware inactive.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np
import yaml

from arm_calibration import fit as zero_fit
from arm_calibration import profiles
from arm_calibration.urdf_chain import Chain, apply_zeros

SHAPES = ("down", "towards", "away", "left", "right")
# A touch further than this from the fit's dot, metres, is named for redoing.
SUSPECT_M = 0.002
# A zero measured no better than this, degrees (one standard deviation), is named as poorly measured.
POOR_SIGMA_DEG = 0.5


def shape_words(shape: str, tilt_deg: float) -> str:
    if shape == "down":
        return "the tool straight down"
    way = {
        "towards": "towards the arm's base",
        "away": "away from the base",
        "left": "to the left",
        "right": "to the right",
    }[shape]
    return f"the tool leaning about {tilt_deg:.0f} degrees {way}"


def lean(chain: Chain, positions: dict[str, float], tool_axis) -> str:
    """How the tool leans, by the description: degrees from straight down, and which way."""
    pose = chain.forward(positions)
    pointing, tip = pose[:3, :3] @ np.asarray(tool_axis, float), pose[:3, 3]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -pointing[2]))))
    if tilt < 10:
        return f"{tilt:.0f} deg from straight down"
    radial = tip[:2] / max(math.hypot(tip[0], tip[1]), 1e-9)
    horizontal = pointing[:2] / max(float(np.linalg.norm(pointing[:2])), 1e-9)
    along = float(horizontal @ radial)
    across = float(horizontal[1] * radial[0] - horizontal[0] * radial[1])
    way = (
        "away from the base"
        if along > 0.7
        else "towards the base"
        if along < -0.7
        else "to the left"
        if across > 0
        else "to the right"
    )
    return f"{tilt:.0f} deg {way}"


def record(args, profile: profiles.Profile, arm, chain: Chain, out: pathlib.Path) -> tuple[list[dict], dict]:
    """The person's touches, saved after each to `out`/touches.json; and what they were read against."""
    joints = chain.movable
    arm.positions(joints)
    about = {"joint_states": arm.topics[0], "controller_manager": arm.topics[2]}
    if args.release == "lifecycle":
        hardware = arm.hardware_for(joints, args.ros2_control)
        about["ros2_control"] = hardware.name
        input(
            f"Hold the arm: {hardware.name} goes inactive, its torque off, and the arm will fall. "
            "Press Enter when holding it. "
        )
        controllers = arm.release(hardware)
        print(f"  released {hardware.name}" + (f" (deactivated {', '.join(controllers)})" if controllers else ""))
        if not arm.updating():
            raise SystemExit(
                f"{arm.topics[0]} stopped updating with {hardware.name} inactive: the contract needs "
                "its state interfaces read while inactive. Restored; release it by other means and "
                "run with --release none."
            )
    print("\n\n".join(__doc__.split("\n\n")[2:4]).replace("its tip", profile.tip))
    saved = []
    for dot in range(args.dots or profile.dots):
        shape = 0
        while shape < len(SHAPES):
            answer = (
                input(
                    f"\nDot {dot + 1}, {shape_words(SHAPES[shape], profile.tilt_deg)}. "
                    "Enter to record, s to skip, q to finish: "
                )
                .strip()
                .lower()
            )
            if answer == "q":
                return saved, about
            if answer == "s":
                shape += 1
                continue
            positions = arm.still_positions(joints)
            if positions is None:
                print("  the arm moved while recording: hold it still and press Enter again")
                continue
            saved.append({"dot": dot, "shape": SHAPES[shape], "positions": positions})
            (out / "touches.json").write_text(
                json.dumps({"profile": profile.name, "read": about, "touches": saved}, indent=1)
            )
            print(f"  recorded: {lean(chain, positions, profile.tool_axis)}")
            shape += 1
    return saved, about


def report(result: zero_fit.Result, saved: list[dict], profile: profiles.Profile):
    print(
        f"\n{len(saved)} touches on {len(result.dots)} dots; {profile.tip} {result.rms_m * 1000:.2f} mm rms "
        "from their dots after the fit."
    )
    for joint, value in result.zeros.items():
        sigma = math.degrees(result.sigma[joint])
        poor = "  poorly measured: more dots, or shapes that turn this joint more" if sigma > POOR_SIGMA_DEG else ""
        print(f"  {joint:24s} {math.degrees(value):+6.2f} deg  (+- {sigma:.2f}){poor}")
    for joint, why in profile.held_joints.items():
        print(f"  {joint:24s} held: {why}")
    print(f"  {profile.tip} at {np.round(result.tip_offset * 1000, 1)} mm in {profile.tip_link}")
    for touch, distance in zip(saved, result.residuals_m, strict=True):
        if distance > SUSPECT_M:
            print(
                f"  dot {touch['dot'] + 1}, {touch['shape']}: {distance * 1000:.1f} mm off its dot; redo it if "
                "the tip was not in the dot"
            )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--profile",
        default=profiles.DEFAULT,
        help=f"the arm's profile: a path, or one shipped ({', '.join(profiles.available())}); "
        f"default {profiles.DEFAULT}",
    )
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path.cwd() / "arm_zeros")
    parser.add_argument(
        "--urdf", type=pathlib.Path, help="the description to correct, instead of the one on --robot-description"
    )
    parser.add_argument("--joint-states", default="/joint_states", help="sensor_msgs/msg/JointState topic")
    parser.add_argument(
        "--robot-description", default="/robot_description", help="std_msgs/msg/String topic, transient-local"
    )
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument(
        "--ros2-control",
        metavar="NAME",
        help="the <ros2_control> block to release, by its name in the description (default: the one whose "
        "joints are the chain's)",
    )
    parser.add_argument(
        "--release",
        choices=("lifecycle", "none"),
        default="lifecycle",
        help="lifecycle: take the ros2_control hardware inactive to cut torque, and restore it after; "
        "none: the arm is already free to move by hand",
    )
    parser.add_argument("--dots", type=int, help="how many dots to touch (default: the profile's)")
    parser.add_argument(
        "--refit",
        type=pathlib.Path,
        help="fit touches recorded before (touches.json, with description.urdf beside it), no robot",
    )
    args = parser.parse_args(argv)

    profile = profiles.load(args.profile)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.refit:
        urdf = args.urdf.read_text() if args.urdf else (args.refit.parent / "description.urdf").read_text()
        data = json.loads(args.refit.read_text())
        saved, about = data["touches"], data.get("read", {})
        chain = Chain(urdf, profile.base_link, profile.tip_link)
    else:
        from arm_calibration.ros_arm import ContractError, RosArm

        arm = RosArm(args.joint_states, args.robot_description, args.controller_manager)
        try:
            urdf = args.urdf.read_text() if args.urdf else arm.description()
            (args.out / "description.urdf").write_text(urdf)
            chain = Chain(urdf, profile.base_link, profile.tip_link)
            saved, about = record(args, profile, arm, chain, args.out)
        except ContractError as exc:
            sys.exit(f"the robot does not meet the contract: {exc}")
        finally:
            try:
                if arm.released:
                    arm.restore()
                    print("restored the ros2_control hardware and controllers released")
            finally:
                arm.close()
    if not saved:
        sys.exit("no touches recorded: nothing to fit")
    touches = [zero_fit.Touch(t["dot"], t["positions"]) for t in saved]
    result = zero_fit.fit(chain, touches, profile.free_joints, profile.plane_normal, profile.tip_offset)
    report(result, saved, profile)

    out = {
        "profile": profile.name,
        "joint_zeros_rad": {j: round(v, 6) for j, v in result.zeros.items()},
        "joint_zeros_sigma_rad": {j: round(v, 6) for j, v in result.sigma.items()},
        "held_joints": list(profile.held_joints),
        "touch_rms_m": round(result.rms_m, 6),
        "touches": len(touches),
        "tip_link": profile.tip_link,
        "tip_offset_m": [round(float(v), 6) for v in result.tip_offset],
        "read": about,
        "measured": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    (args.out / "joint_zeros.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
    urdf_out = args.out / f"{profile.name}_calibrated.urdf"
    urdf_out.write_text(apply_zeros(urdf, result.zeros))
    print(f"\nwrote {args.out / 'joint_zeros.yaml'} and {urdf_out}")


if __name__ == "__main__":
    main()
