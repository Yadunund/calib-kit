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

"""Calibrate an arm against its URDF: its joint zeros, its tool centre point (TCP), or both.

You bring the arm's tip onto a fixed reference several times, holding the tool a different way each
time, and press Enter for each touch. Every touch of one reference puts the tip on the same point,
so the zeros and the TCP are the values that make the URDF agree with itself about that point.

References:

- dots: small pen dots on the table the arm stands on. The arm is released and you guide it by
  hand. Dots find joint zeros, the TCP, or both.
- pin: a sharp point fixed within reach. You move the arm with its own controls (a pendant,
  teleoperation) until the tool's point meets the pin. A pin finds the TCP only.

    calibrate_arm --out ~/arm_cal                                                    # zeros, from dots
    calibrate_arm --profile my_arm.yaml --solve tcp --reference pin --out ~/arm_cal  # a tool's TCP
    calibrate_arm --refit ~/arm_cal/touches.json --out ~/arm_cal                    # fit again, no robot

What the robot must provide is in README.md.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import select
import sys
import time

import numpy as np
import yaml

from arm_calibration import fit as arm_fit
from arm_calibration import profiles
from arm_calibration import reach as arm_reach
from arm_calibration.urdf_chain import Chain, add_tcp, apply_zeros

# A touch further than this from the fit's reference, metres, is named for redoing.
SUSPECT_M = 0.002
# A zero measured no better than this, degrees (one standard deviation), is named as poorly measured.
POOR_SIGMA_DEG = 0.5
# A tool centre point measured no better than this on any axis, metres, likewise.
POOR_TIP_SIGMA_M = 0.0005
# How the arm is moved onto each kind of reference, unless --release says otherwise.
RELEASE = {"dots": "lifecycle", "pin": "none"}
# Dots at least this far apart, metres.
DOT_SPACING_M = 0.05
# A dot's touches whose tips, by the URDF, are further than this from its first touch's, metres, are
# named: the tip was likely not in the dot.
SPREAD_M = 0.02
WAY_WORDS = {
    "towards": "towards the base",
    "away": "away from the base",
    "left": "to the left",
    "right": "to the right",
}


def ask(prompt: str, status=None) -> str:
    """The person's answer to `prompt`, trimmed and lowercased. On a terminal, `status()` is shown
    on the line below and redrawn while it changes, until they press Enter."""
    if status is None or not sys.stdin.isatty():
        return input(prompt).strip().lower()
    print(prompt)
    shown = None
    while True:
        line = status()
        if line != shown:
            sys.stdout.write("\r\033[K  " + line)
            sys.stdout.flush()
            shown = line
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.readline().strip().lower()


def lean_of(chain: Chain, profile: profiles.Profile, positions: dict[str, float]) -> arm_reach.Lean:
    return arm_reach.lean(chain, positions, profile.tip_offset, profile.tool_axis, profile.plane_normal)


def lean_words(lean: arm_reach.Lean) -> str:
    """How the tool leans, in words."""
    if lean.way is None:
        return f"pointing down ({lean.deg:.0f} deg from straight down)"
    return f"leaning {lean.deg:.0f} deg, its top {WAY_WORDS[lean.way]}"


def shape_words(shape: str, profile: profiles.Profile, reference: str) -> str:
    """What to do for a touch in `shape`."""
    on = "in the dot" if reference == "dots" else "on the pin's point"
    if shape == "down":
        return f"point the tool straight down with {profile.tip} {on}"
    return (
        f"keep {profile.tip} {on} and tilt the tool so its top moves {WAY_WORDS[shape]}, "
        f"as far as it goes (at least {profile.min_lean_deg:.0f} deg)"
    )


def shape_problem(shape: str, lean: arm_reach.Lean, min_lean_deg: float) -> str | None:
    """Why a touch leaning as `lean` is not in `shape`; None if it is."""
    if shape == "down":
        if lean.way is None:
            return None
        return f"it leans {lean.deg:.0f} deg: point it straight down (within {arm_reach.DOWN_DEG:.0f} deg)"
    if lean.deg < min_lean_deg:
        return f"it leans only {lean.deg:.0f} deg: tilt it at least {min_lean_deg:.0f} deg"
    if lean.way != shape:
        return f"its top moves {WAY_WORDS[lean.way]}: tilt it so its top moves {WAY_WORDS[shape]}"
    return None


def _planar(vector, plane_normal) -> float:
    up = np.asarray(plane_normal, float) / np.linalg.norm(plane_normal)
    v = np.asarray(vector, float)
    return float(np.linalg.norm(v - (v @ up) * up))


def spot_check(
    chain: Chain, profile: profiles.Profile, positions: dict[str, float], marked: list[np.ndarray]
) -> tuple[bool, str]:
    """Whether the tip, where `positions` put it, is a good place for a dot, and why, in words."""
    lean = lean_of(chain, profile, positions)
    where = f"{_planar(lean.tip, profile.plane_normal) * 100:.0f} cm from the base"
    if lean.way is not None:
        return False, f"{where}; point the tool straight down (it leans {lean.deg:.0f} deg)"
    for i, dot in enumerate(marked):
        apart = _planar(lean.tip - dot, profile.plane_normal)
        if apart < DOT_SPACING_M:
            return False, (
                f"{where}; only {apart * 100:.0f} cm from dot {i + 1}: "
                f"move at least {DOT_SPACING_M * 100:.0f} cm from it"
            )
    start = [positions[j] for j in chain.movable]
    leans = [s for s in profile.shapes if s != "down"]
    reaches = {
        s: arm_reach.reach(chain, start, lean.tip, s, profile.tip_offset, profile.tool_axis, profile.plane_normal)
        for s in leans
    }
    if any(r is None for r in reaches.values()):
        return False, f"{where}; the URDF cannot hold the tip here"
    can = ", ".join(f"{WAY_WORDS[s]} up to {r:.0f} deg" for s, r in reaches.items())
    if any(r < profile.min_lean_deg for r in reaches.values()):
        return False, (
            f"{where}; the tool can tilt {can}, but needs {profile.min_lean_deg:.0f} deg each way: "
            "move nearer to or further from the base"
        )
    return True, f"{where}; the tool can tilt {can}"


def suggest_dots(chain: Chain, profile: profiles.Profile, count: int):
    """Say how far from the base the dots can go, and where to put each."""
    leans = [s for s in profile.shapes if s != "down"]
    print("\nWorking out where the dots can go...")
    span = arm_reach.spread(
        chain, leans, profile.min_lean_deg, profile.tip_offset, profile.tool_axis, profile.plane_normal
    )
    if span is None:
        print(
            f"  The URDF's joint limits leave no distance from the base where the tool can tilt "
            f"{profile.min_lean_deg:.0f} deg every way asked. Each spot is checked as you go."
        )
        return
    near, far = span
    places = [near + (far - near) * i / max(count - 1, 1) for i in range(count)]
    print(
        f"  Dots can go {near * 100:.0f} to {far * 100:.0f} cm from the centre of the base. Spread them out:\n"
        + "\n".join(f"    dot {i + 1}: about {p * 100:.0f} cm from the base" for i, p in enumerate(places))
        + f"\n  and some to each side, at least {DOT_SPACING_M * 100:.0f} cm apart."
    )


def find_spot(args, profile, arm, chain, point: int, marked: list[np.ndarray]) -> np.ndarray | None:
    """Where the person chooses to put dot `point`, checked before they mark it; None if they finish."""
    joints = chain.movable

    def status() -> str:
        good, words = spot_check(chain, profile, arm.positions(joints), marked)
        return f"{'GOOD SPOT' if good else 'not yet'}: {words}"

    while True:
        answer = ask(
            f"\nDot {point + 1} of {args.count}: rest {profile.tip} on the table where this dot will go, "
            "with the tool pointing straight down. Do not mark it yet.\n"
            "  Press Enter when the line below says the spot is good (q: finish). ",
            status=status,
        )
        if answer == "q":
            return None
        positions = arm.still_positions(joints)
        if positions is None:
            print("  The arm moved. Hold it still and press Enter again.")
            continue
        good, words = spot_check(chain, profile, positions, marked)
        if not good:
            print(f"  Not here: {words}.")
            continue
        print(
            f"  Spot accepted: {words}.\n"
            f"  Lift the tool a little, mark a small dot (about 1 mm) where {profile.tip} were, and put "
            "them back in it."
        )
        return lean_of(chain, profile, positions).tip


def keep(out: pathlib.Path, args, profile: profiles.Profile, about: dict, saved: list[dict]):
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


def touch(args, profile, arm, chain, out, about, saved: list[dict], point: int, label: str) -> bool:
    """The person's touches of reference `point`, one per shape, added to `saved`. True if they
    chose to finish."""
    joints = chain.movable
    shapes = profile.shapes
    taken: list[int] = []  # the shape of each of this reference's touches, by its index
    shape = 0
    while shape < len(shapes):
        want = shapes[shape]
        answer = ask(
            f"\n{label}, touch {shape + 1} of {len(shapes)}: {shape_words(want, profile, args.reference)}.\n"
            "  Hold still and press Enter to record (r: redo the last touch, s: skip this one, q: finish). ",
            status=lambda: lean_words(lean_of(chain, profile, arm.positions(joints))),
        )
        if answer == "q":
            return True
        if answer == "s":
            shape += 1
            continue
        if answer == "r":
            if taken:
                saved.pop()
                shape = taken.pop()
                keep(out, args, profile, about, saved)
                print("  Removed the last touch.")
            else:
                print(f"  No touch of this {'dot' if args.reference == 'dots' else 'pin'} to redo.")
            continue
        positions = arm.still_positions(joints)
        if positions is None:
            print("  The arm moved. Hold it still and press Enter again.")
            continue
        lean = lean_of(chain, profile, positions)
        problem = shape_problem(want, lean, profile.min_lean_deg)
        if problem:
            print(f"  Not recorded: {problem}.")
            continue
        saved.append({"dot": point, "shape": want, "positions": positions})
        taken.append(shape)
        keep(out, args, profile, about, saved)
        print(f"  Recorded: {lean_words(lean)}.")
        shape += 1
    return False


def spread_of(chain: Chain, profile: profiles.Profile, touches: list[dict]) -> float:
    """How far, metres, the URDF puts the furthest of `touches`' tips from the first's."""
    tips = [lean_of(chain, profile, t["positions"]).tip for t in touches]
    return max((float(np.linalg.norm(t - tips[0])) for t in tips), default=0.0)


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
    dots = args.reference == "dots"
    if args.release == "lifecycle":
        hardware = arm.hardware_for(joints, args.ros2_control)
        about["ros2_control"] = hardware.name
        input(
            f"\nHold the arm now. When you press Enter, {hardware.name} switches its motors off and the arm "
            "will drop if you let go. "
        )
        controllers = arm.release(hardware)
        print(f"  Released {hardware.name}" + (f" (stopped {', '.join(controllers)})." if controllers else "."))
        if not arm.updating():
            raise SystemExit(
                f"{arm.topics[0]} stopped updating with {hardware.name} inactive, and the calibration needs "
                "to read the joints while the arm is free. The arm is restored. Free it some other way and "
                "run with --release none."
            )
        if ask("  Does the arm now move freely by hand? (y/n) ") not in ("y", "yes"):
            raise SystemExit(
                f"{hardware.name} is inactive but its motors kept their torque: its driver must switch them "
                "off when the hardware is deactivated (README.md, What the robot must provide). The arm is "
                "restored."
            )
    else:
        print("\nThe arm stays under its own control: move it onto each reference with its own controls.")
    ways = ", ".join(["straight down" if s == "down" else f"tilted {WAY_WORDS[s]}" for s in profile.shapes])
    if dots:
        print(
            f"\nYou will mark {args.count} dots on the table and touch each with {profile.tip}, holding the "
            f"tool {len(profile.shapes)} ways: {ways}. 'Top' is the end of the tool away from the tip; left "
            "and right are as seen from the base, looking out at the dot. For each touch, hold still and "
            "press Enter."
        )
        suggest_dots(chain, profile, args.count)
    else:
        print(
            f"\nTouch the pin's point with {profile.tip}, holding the tool {len(profile.shapes)} ways: {ways}. "
            "Turn the tool a little about its own axis between touches. For each touch, hold still and "
            "press Enter."
        )
    saved: list[dict] = []
    marked: list[np.ndarray] = []
    point = 0
    while point < args.count:
        if dots:
            spot = find_spot(args, profile, arm, chain, point, marked)
            if spot is None:
                return saved, about
            marked.append(spot)
        while True:
            first = len(saved)
            label = f"Dot {point + 1} of {args.count}" if dots else f"Pin {point + 1} of {args.count}"
            if touch(args, profile, arm, chain, out, about, saved, point, label):
                return saved, about
            apart = spread_of(chain, profile, saved[first:])
            if not dots or apart <= SPREAD_M:
                break
            again = ask(
                f"  These touches put {profile.tip} up to {apart * 1000:.0f} mm apart, by the URDF; more than "
                f"{SPREAD_M * 1000:.0f} mm usually means they were not in the dot every time. Touch this dot "
                "again? (y/n) "
            )
            if again not in ("y", "yes"):
                break
            del saved[first:]
            keep(out, args, profile, about, saved)
        point += 1
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
