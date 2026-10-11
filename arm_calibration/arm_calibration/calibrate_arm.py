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

"""Calibrate an arm against its description: its joint zeros, its tool centre point, or both.

An actuator's zero is often set by eye (LeRobot's calibration sets it where a person judges the
middle of each joint's travel to be), so a joint reads a degree or a few off the angle its
description means; and a tool mounted on an arm puts its point somewhere the description does not
know. Both are found by bringing the arm's tip onto fixed reference points in several arm shapes
(`fit`), and written as YAML files and as a corrected description.

The references are either:

- dots: pen dots on the table the arm is clamped flat to, within reach (one near the base, one far
  out in front, one to the side). The arm is released and its tip rested in each dot by hand.
  Dots find joint zeros, the tool centre point, or both.
- pin: a sharp point fixed within reach. The arm is jogged under its own control (a teach
  pendant, teleoperation, servoing) until the tool's point meets the pin's, then reoriented and
  jogged onto it again. A pin finds the tool centre point.

For each reference, five shapes: the tool straight down, then leaning towards the base, away from
it, and to either side; turning the tool about its own axis between touches helps too. Hold still
and press Enter for each. The fit prints each value with its uncertainty and how far the tip was
from its references, and writes joint_zeros.yaml, tcp.yaml and the corrected URDF.

    calibrate_arm --solve zeros --reference dots --out ~/arm_cal                    # zeros by eye
    calibrate_arm --profile my_arm.yaml --solve tcp --reference pin --out ~/arm_cal  # a new tool
    calibrate_arm --refit ~/arm_cal/touches.json --out ~/arm_cal                    # no robot needed

What the robot must offer is the contract in README.md: the description and joint states on
topics, and, to release the arm, a controller manager that takes its ros2_control hardware
inactive.
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

from arm_calibration import fit as arm_fit
from arm_calibration import profiles
from arm_calibration.urdf_chain import Chain, add_tcp, apply_zeros

SHAPES = ("down", "towards", "away", "left", "right")
# A touch further than this from the fit's reference, metres, is named for redoing.
SUSPECT_M = 0.002
# A zero measured no better than this, degrees (one standard deviation), is named as poorly measured.
POOR_SIGMA_DEG = 0.5
# A tool centre point measured no better than this on any axis, metres, likewise.
POOR_TIP_SIGMA_M = 0.0005
# How the arm is moved onto each kind of reference, unless --release says otherwise.
RELEASE = {"dots": "lifecycle", "pin": "none"}


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


def check(solve: str, reference: str, count: int):
    """Refuse what touches on these references cannot measure."""
    if "zeros" in solve and reference != "dots":
        raise SystemExit("joint zeros need dots on the table: a pin fixes the tool centre point, not the zeros")
    if "zeros" in solve and count < 3:
        raise SystemExit("joint zeros need three dots or more")


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
    else:
        print("  the arm stays under its own control: jog it onto each reference")
    print("\n\n".join(__doc__.split("\n\n")[2:4]).replace("its tip", profile.tip))
    saved = []
    name = "Dot" if args.reference == "dots" else "Pin"
    for point in range(args.count):
        shape = 0
        while shape < len(SHAPES):
            answer = (
                input(
                    f"\n{name} {point + 1}, {shape_words(SHAPES[shape], profile.tilt_deg)}. "
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
            saved.append({"dot": point, "shape": SHAPES[shape], "positions": positions})
            (out / "touches.json").write_text(
                json.dumps(
                    {
                        "profile": profile.name,
                        "reference": args.reference,
                        "solve": args.solve,
                        "read": about,
                        "touches": saved,
                    },
                    indent=1,
                )
            )
            print(f"  recorded: {lean(chain, positions, profile.tool_axis)}")
            shape += 1
    return saved, about


def report(result: arm_fit.Result, saved: list[dict], profile: profiles.Profile, solve: str, reference: str):
    print(
        f"\n{len(saved)} touches on {len(result.dots)} {'dots' if reference == 'dots' else 'pins'}; "
        f"{profile.tip} {result.rms_m * 1000:.2f} mm rms from them after the fit."
    )
    for joint, value in result.zeros.items():
        sigma = math.degrees(result.sigma[joint])
        poor = "  poorly measured: more dots, or shapes that turn this joint more" if sigma > POOR_SIGMA_DEG else ""
        print(f"  {joint:24s} {math.degrees(value):+6.2f} deg  (+- {sigma:.2f}){poor}")
    if "zeros" in solve:
        for joint, why in profile.held_joints.items():
            print(f"  {joint:24s} held: {why}")
    tip = np.round(result.tip_offset * 1000, 1)
    if "tcp" in solve:
        sd = np.round(result.tip_sigma * 1000, 2)
        poor = (
            "  poorly measured: more references, or more varied shapes"
            if float(np.max(result.tip_sigma)) > POOR_TIP_SIGMA_M
            else ""
        )
        print(f"  tool centre point {tip} mm in {profile.tip_link} (+- {sd}){poor}")
    else:
        print(f"  {profile.tip} at {tip} mm in {profile.tip_link}")
    for touch, distance in zip(saved, result.residuals_m, strict=True):
        if distance > SUSPECT_M:
            print(
                f"  reference {touch['dot'] + 1}, {touch['shape']}: {distance * 1000:.1f} mm off; redo it if "
                "the tip was not on the reference"
            )


def write(args, profile: profiles.Profile, urdf: str, result: arm_fit.Result, touches: int, about: dict):
    common = {
        "profile": profile.name,
        "reference": args.reference,
        "touch_rms_m": round(result.rms_m, 6),
        "touches": touches,
        "read": about,
        "measured": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    written = []
    if "zeros" in args.solve:
        zeros = {
            "joint_zeros_rad": {j: round(v, 6) for j, v in result.zeros.items()},
            "joint_zeros_sigma_rad": {j: round(v, 6) for j, v in result.sigma.items()},
            "held_joints": list(profile.held_joints),
            "tip_link": profile.tip_link,
            "tip_offset_m": [round(float(v), 6) for v in result.tip_offset],
            **common,
        }
        (args.out / "joint_zeros.yaml").write_text(yaml.safe_dump(zeros, sort_keys=False))
        written.append(args.out / "joint_zeros.yaml")
        # The description from the values as written, so applying the files gives the same one.
        urdf = apply_zeros(urdf, zeros["joint_zeros_rad"])
    if "tcp" in args.solve:
        tcp = {
            "parent": profile.tip_link,
            "link": profile.tcp_link,
            "xyz_m": [round(float(v), 6) for v in result.tip_offset],
            "sigma_m": [round(float(v), 6) for v in result.tip_sigma],
            **common,
        }
        (args.out / "tcp.yaml").write_text(yaml.safe_dump(tcp, sort_keys=False))
        written.append(args.out / "tcp.yaml")
        urdf = add_tcp(urdf, profile.tip_link, profile.tcp_link, tcp["xyz_m"])
    urdf_out = args.out / f"{profile.name}_calibrated.urdf"
    urdf_out.write_text(urdf)
    written.append(urdf_out)
    print("\nwrote " + ", ".join(str(p) for p in written))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--profile",
        default=profiles.DEFAULT,
        help=f"the arm's profile: a path, or one shipped ({', '.join(profiles.available())}); "
        f"default {profiles.DEFAULT}",
    )
    parser.add_argument("--solve", choices=profiles.SOLVES, help="what to calibrate (default: the profile's)")
    parser.add_argument("--reference", choices=arm_fit.REFERENCES, help="what to touch (default: the profile's)")
    parser.add_argument(
        "--count", type=int, help="how many references to touch (default: the profile's dots, or one pin)"
    )
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path.cwd() / "arm_calibration")
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
        help="lifecycle: take the ros2_control hardware inactive to cut torque, and restore it after; "
        "none: the arm is moved under its own control or already free (default: lifecycle for dots, "
        "none for a pin)",
    )
    parser.add_argument(
        "--refit",
        type=pathlib.Path,
        help="fit touches recorded before (touches.json, with description.urdf beside it), no robot needed",
    )
    args = parser.parse_args(argv)

    profile = profiles.load(args.profile)
    args.out.mkdir(parents=True, exist_ok=True)
    recorded = json.loads(args.refit.read_text()) if args.refit else {}
    args.solve = args.solve or recorded.get("solve") or profile.solve
    args.reference = args.reference or recorded.get("reference") or profile.reference
    args.count = args.count or (profile.dots if args.reference == "dots" else 1)
    args.release = args.release or RELEASE[args.reference]
    if args.refit:
        urdf = args.urdf.read_text() if args.urdf else (args.refit.parent / "description.urdf").read_text()
        saved, about = recorded["touches"], recorded.get("read", {})
        chain = Chain(urdf, profile.base_link, profile.tip_link)
    else:
        check(args.solve, args.reference, args.count)
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
    check(args.solve, args.reference, len({t["dot"] for t in saved}))
    touches = [arm_fit.Touch(t["dot"], t["positions"]) for t in saved]
    result = arm_fit.fit(
        chain,
        touches,
        profile.free_joints if "zeros" in args.solve else [],
        profile.plane_normal,
        profile.tip_offset,
        reference=args.reference,
        tip_free="tcp" in args.solve,
    )
    report(result, saved, profile, args.solve, args.reference)
    write(args, profile, urdf, result, len(touches), about)


if __name__ == "__main__":
    main()
