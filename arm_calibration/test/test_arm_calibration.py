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

"""Joint zeros from touches on table dots, on arms of different shapes, with a simulated hand."""

import argparse
import json
import math
import re
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from arm_calibration.urdf_chain import Chain, add_tcp, apply_zeros, matrix_to_rpy, rpy_to_matrix
from scipy.optimize import least_squares

from arm_calibration import apply_calibration, calibrate_arm, profiles
from arm_calibration import fit as zero_fit
from arm_calibration import reach as arm_reach

FULL = ["down", "towards", "away", "left", "right"]
# An arm whose wrist cannot lean sideways leans each way in the plane it reaches in.
IN_PLANE = ["down", "towards", "away"]
TILT = math.radians(40)
# Three dots spread from near the base to far, to either side, where every arm here can lean at
# least 15 degrees every way it leans.
DOTS = np.array([(0.11, -0.04, 0.0), (0.16, 0.08, 0.0), (0.21, -0.05, 0.0)])


def joint(name, parent, child, xyz, rpy="0 0 0", axis="0 0 1", kind="revolute"):
    return (
        f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
        f'<origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis}"/>'
        '<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
    )


def with_limits(urdf: str, limits: dict[str, tuple[float, float]]) -> str:
    """`urdf` with the joints in `limits` bounded to (lower, upper), radians."""
    root = ET.fromstring(urdf)
    for element in root.iter("joint"):
        if element.get("name") in limits:
            lower, upper = limits[element.get("name")]
            element.find("limit").set("lower", str(lower))
            element.find("limit").set("upper", str(upper))
    return ET.tostring(root, encoding="unicode")


def arm_urdf(joints) -> str:
    links = {"base"} | {j[2] for j in joints}
    return (
        '<robot name="arm">'
        + "".join(f'<link name="{n}"/>' for n in sorted(links))
        + "".join(joint(*j) for j in joints)
        + "</robot>"
    )


# A 5-joint arm shaped like most hobby arms: base pan, shoulder, elbow, wrist pitch, wrist roll.
# The tool points along the tip frame's z.
FIVE = arm_urdf(
    [
        ("pan", "base", "l1", "0 0 0.06"),
        ("lift", "l1", "l2", "0.02 0 0.05", "1.5708 0 0"),
        ("elbow", "l2", "l3", "0 0.12 0"),
        ("pitch", "l3", "l4", "0.13 0 0"),
        ("roll", "l4", "l5", "0.05 0 0", "0 1.5708 0"),
        ("tip_joint", "l5", "tip", "0.004 0 0.09", "0 0 0", "0 0 0", "fixed"),
    ]
)
# A 6-joint arm with a different layout: an offset shoulder, a forearm twist, and longer links.
SIX = arm_urdf(
    [
        ("j1", "base", "a1", "0 0 0.08"),
        ("j2", "a1", "a2", "0 0.03 0.04", "1.5708 0 0"),
        ("j3", "a2", "a3", "0 0.15 0"),
        ("j4", "a3", "a4", "0.07 0 0", "0 1.5708 0"),
        ("j5", "a4", "a5", "0 0 0.08", "0 -1.5708 0"),
        ("j6", "a5", "a6", "0.04 0 0", "0 1.5708 0"),
        ("tip_joint", "a6", "tip", "0 0.006 0.07", "0 0 0", "0 0 0", "fixed"),
    ]
)
ARMS = {
    "five": (FIVE, ["pan", "lift", "elbow", "pitch", "roll"], ["lift", "elbow", "pitch"]),
    "six": (SIX, ["j1", "j2", "j3", "j4", "j5", "j6"], ["j2", "j3", "j4", "j5"]),
}
# The shapes each arm can take, and the dots it is calibrated on.
PROTOCOL = {"five": (IN_PLANE, DOTS), "six": (FULL, DOTS)}
# The five-joint arm with joint limits like a hobby arm's, so that it cannot lean far every way on
# every dot.
FIVE_LIMITED = with_limits(FIVE, {"lift": (-1.7, 1.7), "elbow": (-1.7, 1.5), "pitch": (-1.6, 1.6)})
# How far a person leans the tool, degrees: as far as it goes, up to the first.
LEANS = (40.0, 30.0, 20.0, 15.0)


def approach(point, shape, tilt=TILT):
    """The tool's axis, top to tip, leaning `tilt` with its top moving `shape` at `point`."""
    radial = np.array([point[0], point[1], 0.0]) / math.hypot(point[0], point[1])
    across = np.array([-radial[1], radial[0], 0.0])
    top = {"down": np.zeros(3), "towards": -radial, "away": radial, "left": across, "right": -across}[shape]
    direction = np.array([0.0, 0.0, -math.cos(tilt)]) - math.sin(tilt) * top
    return direction / np.linalg.norm(direction)


def hand(chain, joints, true_zeros, tip, rng, dots=DOTS, noise=0.0005, table=None, shapes=FULL, leans=LEANS):
    """A person's touches: the true arm, within its joint limits, put with its tip (`tip` in the tip
    frame) in each dot, missed by `noise`, the tool along each shape, leaning by the first of `leans`
    the arm can make; the joints read the true angles less the zeros. `table` turns the dots and
    shapes with a table that is not square to the base."""
    table = np.eye(3) if table is None else table
    lower = np.array([chain.limits[j][0] for j in joints])
    upper = np.array([chain.limits[j][1] for j in joints])
    touches = []
    for d, dot in enumerate(dots):
        for shape in shapes:
            for tilt in [0.0] if shape == "down" else leans:
                touch = _pose(chain, joints, true_zeros, tip, rng, dot, shape, tilt, noise, table, lower, upper)
                if touch is not None:
                    touches.append(zero_fit.Touch(d, touch))
                    break
    return touches


def _pose(chain, joints, true_zeros, tip, rng, dot, shape, tilt, noise, table, lower, upper):
    """The joint readings with the tip on `dot` leaning `tilt` degrees `shape`; None if the arm
    cannot take that shape there."""
    target = table @ (dot + np.array([*rng.normal(0.0, noise, 2), 0.0]))
    pointing = table @ approach(dot, shape, math.radians(tilt))
    best = None
    for start in range(6):
        guess = rng.uniform(-1.0, 1.0, len(joints))
        guess[0] = math.atan2(target[1], target[0])
        guess = np.clip(guess, lower + 1e-6, upper - 1e-6)

        # The person also turns the tool about its own axis between touches, as asked:
        # its x axis towards a direction drawn for the touch, as far as the wrist allows.
        turn = rng.uniform(-math.pi, math.pi)
        facing = np.array([math.cos(turn), math.sin(turn), 0.0])

        def residual(theta, target=target, pointing=pointing, facing=facing):
            pose = chain.forward(dict(zip(joints, theta, strict=True)))
            side = facing - (facing @ pointing) * pointing
            return np.concatenate(
                [
                    (pose[:3, :3] @ tip + pose[:3, 3] - target) / 0.001,
                    pose[:3, 2] - pointing,
                    0.2 * (pose[:3, 0] - side / np.linalg.norm(side)),
                    0.01 * theta,
                ]
            )

        solution = least_squares(residual, guess, bounds=(lower, upper))
        if best is None or solution.cost < best.cost:
            best = solution
        if np.linalg.norm(best.fun[:3]) < 0.01 and start >= 1:
            break
    # A shape the arm cannot take there (out of reach, or a lean its joints cannot make) is
    # skipped, as a person would.
    if np.linalg.norm(best.fun[:3]) > 0.05 or np.linalg.norm(best.fun[3:6]) > math.radians(5):
        return None
    return {j: float(t - true_zeros.get(j, 0.0)) for j, t in zip(joints, best.x, strict=True)}


def test_rpy_round_trip():
    """ARM-OUT-3"""
    for rpy in [(0.1, -0.4, 2.0), (-1.5708, -1.5708, 0.0), (3.0, 0.2, -1.0)]:
        np.testing.assert_allclose(rpy_to_matrix(*matrix_to_rpy(rpy_to_matrix(*rpy))), rpy_to_matrix(*rpy), atol=1e-9)


@pytest.mark.parametrize("arm", list(ARMS))
def test_zeros_folded_into_the_description_move_the_arm_as_the_offsets_do(arm):
    """ARM-OUT-3"""
    urdf, joints, free = ARMS[arm]
    zeros = {j: 0.01 * (i + 1) * (-1) ** i for i, j in enumerate(free)}
    plain, corrected = Chain(urdf, "base", "tip"), Chain(apply_zeros(urdf, zeros), "base", "tip")
    rng = np.random.default_rng(0)
    for _ in range(10):
        q = dict(zip(joints, rng.uniform(-1.2, 1.2, len(joints)), strict=True))
        np.testing.assert_allclose(corrected.forward(q), plain.forward(q, zeros), atol=1e-9)


def test_limits_keep_the_same_physical_travel():
    """ARM-OUT-3"""

    def limits(text):
        element = next(j for j in ET.fromstring(text).findall("joint") if j.get("name") == "elbow")
        return float(element.find("limit").get("lower")), float(element.find("limit").get("upper"))

    assert limits(apply_zeros(FIVE, {"elbow": 0.05})) == pytest.approx((-3.05, 2.95))


@pytest.mark.parametrize("arm", list(ARMS))
def test_touches_on_table_dots_recover_the_zeros(arm):
    """ARM-FIT-1, ARM-FIT-2, ARM-FIT-4, ARM-FIT-6, ARM-FIT-7"""
    urdf, joints, free = ARMS[arm]
    chain = Chain(urdf, "base", "tip")
    rng = np.random.default_rng(1)
    shapes, dots = PROTOCOL[arm]
    for _ in range(3):
        true_zeros = dict(zip(free, rng.uniform(-math.radians(3), math.radians(3), len(free)), strict=True))
        touches = hand(chain, joints, true_zeros, rng.uniform(-0.002, 0.002, 3), rng, dots=dots, shapes=shapes)
        assert len(touches) == 3 * len(shapes)
        result = zero_fit.fit(chain, touches, free)
        assert result.rms_m < 0.001
        for joint_name, value in true_zeros.items():
            error, sigma = abs(math.degrees(result.zeros[joint_name] - value)), math.degrees(result.sigma[joint_name])
            # Within its own uncertainty, and well measured where the touches turn the joint enough.
            assert error < 3 * sigma
            if sigma < 0.25:
                assert error < 0.5


def test_a_table_not_square_to_the_base_is_given_by_its_normal():
    """ARM-FIT-2"""
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    rng = np.random.default_rng(3)
    table = rpy_to_matrix(math.radians(4), math.radians(-3), 0.0)
    true_zeros = {"lift": 0.03, "elbow": -0.02, "pitch": 0.04}
    touches = hand(chain, joints, true_zeros, np.zeros(3), rng, dots=DOTS, table=table, shapes=IN_PLANE)
    result = zero_fit.fit(chain, touches, free, plane_normal=table[:, 2])
    for joint_name, value in true_zeros.items():
        assert result.zeros[joint_name] == pytest.approx(value, abs=math.radians(0.3))


def test_a_tip_beyond_its_frame_is_given_by_the_profile():
    """ARM-FIT-5, ARM-FIT-6"""
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    tip = np.array([0.0, 0.0, 0.008])
    true_zeros = {"lift": 0.03, "elbow": -0.02, "pitch": 0.04}
    touches = hand(chain, joints, true_zeros, tip, np.random.default_rng(5), dots=DOTS, shapes=IN_PLANE)
    result = zero_fit.fit(chain, touches, free, tip_offset=tip)
    for joint_name, value in true_zeros.items():
        error, sigma = abs(math.degrees(result.zeros[joint_name] - value)), math.degrees(result.sigma[joint_name])
        assert error < 3 * sigma
        assert error < 0.5
    np.testing.assert_allclose(result.tip_offset, tip, atol=0.001)


def test_a_dot_touched_once_is_refused():
    """ARM-NO-2"""
    chain = Chain(FIVE, "base", "tip")
    touches = [zero_fit.Touch(0, {}), zero_fit.Touch(1, {}), zero_fit.Touch(1, {"elbow": 0.1})]
    with pytest.raises(ValueError, match="reference 0 has one touch"):
        zero_fit.fit(chain, touches, ["lift"])


def write_profile(tmp_path, joints, free, **settings) -> str:
    profile = {
        "name": "test_arm",
        "description": "an arm made up for the tests",
        "base_link": "base",
        "tip_link": "tip",
        "free_joints": free,
        "held_joints": {joints[0]: "turns every dot with it"},
        **settings,
    }
    (tmp_path / "test_arm.yaml").write_text(yaml.safe_dump(profile))
    return str(tmp_path / "test_arm.yaml")


def test_an_arm_is_added_by_a_profile_and_refitted_from_its_touches(tmp_path):
    """ARM-CFG-3, ARM-OUT-1, ARM-OUT-3, ARM-OUT-6"""
    urdf, joints, free = ARMS["six"]
    profile = write_profile(tmp_path, joints, free)
    (tmp_path / "description.urdf").write_text(urdf)
    chain = Chain(urdf, "base", "tip")
    true_zeros = {"j2": 0.03, "j3": -0.04, "j4": 0.02, "j5": -0.03}
    touches = hand(chain, joints, true_zeros, np.zeros(3), np.random.default_rng(2))
    (tmp_path / "touches.json").write_text(
        json.dumps(
            {
                "profile": "test_arm",
                "read": {},
                "touches": [{"dot": t.dot, "shape": "down", "positions": t.positions} for t in touches],
            }
        )
    )

    calibrate_arm.main(["--profile", profile, "--refit", str(tmp_path / "touches.json"), "--out", str(tmp_path)])

    written = yaml.safe_load((tmp_path / "joint_zeros.yaml").read_text())
    assert written["profile"] == "test_arm"
    for joint_name, value in true_zeros.items():
        assert written["joint_zeros_rad"][joint_name] == pytest.approx(value, abs=math.radians(0.3))
    corrected = Chain((tmp_path / "test_arm_calibrated.urdf").read_text(), "base", "tip")
    q = touches[0].positions
    np.testing.assert_allclose(corrected.forward(q), chain.forward(q, written["joint_zeros_rad"]), atol=1e-5)


def test_a_profile_with_unknown_settings_is_refused(tmp_path):
    """ARM-CFG-1"""
    (tmp_path / "arm.yaml").write_text(
        yaml.safe_dump(
            {"name": "a", "base_link": "b", "tip_link": "t", "free_joints": [], "hardware": {"type": "feetech"}}
        )
    )
    with pytest.raises(ValueError, match="unknown settings hardware"):
        profiles.load(str(tmp_path / "arm.yaml"))
    (tmp_path / "lean.yaml").write_text(
        yaml.safe_dump({"name": "a", "base_link": "b", "tip_link": "t", "shapes": ["down", "towards 20"]})
    )
    with pytest.raises(ValueError, match="shape 'towards 20' is one of"):
        profiles.load(str(tmp_path / "lean.yaml"))
    (tmp_path / "plain.yaml").write_text(yaml.safe_dump({"name": "a", "base_link": "b", "tip_link": "t"}))
    plain = profiles.load(str(tmp_path / "plain.yaml"))
    assert (plain.dots, plain.shapes, plain.min_lean_deg) == (3, FULL, 15.0)


def test_shipped_profiles_load():
    """ARM-CFG-2, ARM-CFG-4"""
    assert "so101" in profiles.available()
    so101 = profiles.load("so101")
    assert so101.free_joints == ["shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint"]
    assert (so101.dots, so101.shapes, so101.min_lean_deg) == (3, IN_PLANE, 15)
    assert so101.tip_offset == [0, 0, 0.0063]
    assert profiles.load(str(profiles.ARMS / "so101.yaml")) == so101.__class__(**so101.__dict__)


def test_so101_zeros_on_its_own_description():
    """ARM-CFG-4, ARM-FIT-7"""
    profile = profiles.load("so101")
    xacro = pytest.importorskip("xacro")
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("so_arm101_description")
    except (ImportError, LookupError) as exc:
        pytest.skip(f"so_arm101_description is not available: {exc}")
    urdf = xacro.process_file(f"{share}/urdf/so_arm101.urdf.xacro").toxml()
    chain = Chain(urdf, profile.base_link, profile.tip_link)
    joints = chain.movable
    true_zeros = {"shoulder_lift_joint": 0.03, "elbow_flex_joint": -0.04, "wrist_flex_joint": 0.05}
    touches = hand(
        chain,
        joints,
        true_zeros,
        np.array(profile.tip_offset),
        np.random.default_rng(4),
        # Near, middle and far within the 12 to 26 cm its joint limits allow, to either side.
        dots=np.array([(0.13, -0.05, 0.0), (0.19, 0.07, 0.0), (0.25, -0.04, 0.0)]),
        shapes=profile.shapes,
    )
    assert len(touches) == 9
    result = zero_fit.fit(chain, touches, profile.free_joints, profile.plane_normal, profile.tip_offset)
    for joint_name, value in true_zeros.items():
        error, sigma = abs(math.degrees(result.zeros[joint_name] - value)), math.degrees(result.sigma[joint_name])
        assert error < 3 * sigma
        assert sigma < calibrate_arm.POOR_SIGMA_DEG


# A tool on the arm: its point 35 mm out along the tool and a few millimetres off its axis.
TOOL = np.array([0.004, -0.003, 0.035])
# A sharp pin fixed above the table, within reach.
PIN = np.array([[0.24, 0.02, 0.05]])
# On one pin, an arm whose wrist only pitches touches each of its shapes twice, the tool turned
# about its axis between.
PIN_SHAPES = {"five": IN_PLANE * 2, "six": FULL}


@pytest.mark.parametrize("arm", list(ARMS))
def test_a_pin_finds_the_tool_centre_point_of_an_arm_with_true_zeros(arm):
    """ARM-FIT-3, ARM-FIT-5, ARM-FIT-6, ARM-FIT-7"""
    urdf, joints, _free = ARMS[arm]
    chain = Chain(urdf, "base", "tip")
    for seed in range(3):
        touches = hand(
            chain, joints, {}, TOOL, np.random.default_rng(seed), dots=PIN, noise=0.0002, shapes=PIN_SHAPES[arm]
        )
        result = zero_fit.fit(chain, touches, [], reference="pin", tip_free=True)
        error = result.tip_offset - TOOL
        assert np.all(np.abs(error) < 3 * result.tip_sigma + 1e-4)
        assert np.linalg.norm(error) < 0.0015
        np.testing.assert_allclose(result.dots[0], PIN[0], atol=0.002)


def test_dots_find_the_tool_centre_point_and_the_zeros_together():
    """ARM-FIT-4, ARM-FIT-5, ARM-FIT-6, ARM-FIT-7"""
    urdf, joints, free = ARMS["six"]
    chain = Chain(urdf, "base", "tip")
    true_zeros = {"j2": 0.03, "j3": -0.04, "j4": 0.02, "j5": -0.03}
    touches = hand(chain, joints, true_zeros, TOOL, np.random.default_rng(7))
    result = zero_fit.fit(chain, touches, free, tip_free=True)
    assert np.linalg.norm(result.tip_offset - TOOL) < 0.0015
    for joint_name, value in true_zeros.items():
        error, sigma = abs(math.degrees(result.zeros[joint_name] - value)), math.degrees(result.sigma[joint_name])
        assert error < 3 * sigma


def test_a_pin_cannot_find_zeros():
    """ARM-NO-1"""
    chain = Chain(FIVE, "base", "tip")
    touches = [zero_fit.Touch(0, {}), zero_fit.Touch(0, {"elbow": 0.1})]
    with pytest.raises(ValueError, match="joint zeros need dots"):
        zero_fit.fit(chain, touches, ["lift"], reference="pin")


def test_the_tool_centre_point_is_a_link_fixed_to_the_tip():
    """ARM-OUT-4"""
    with_tcp = add_tcp(FIVE, "tip", "tcp_link", TOOL)
    q = dict.fromkeys(ARMS["five"][1], 0.3)
    tip, tcp = Chain(with_tcp, "base", "tip").forward(q), Chain(with_tcp, "base", "tcp_link").forward(q)
    np.testing.assert_allclose(tcp[:3, 3], tip[:3, :3] @ TOOL + tip[:3, 3], atol=1e-9)
    np.testing.assert_allclose(tcp[:3, :3], tip[:3, :3], atol=1e-9)
    moved = add_tcp(with_tcp, "tip", "tcp_link", [0.0, 0.0, 0.05])
    assert moved.count('name="tcp_link"') == 1
    np.testing.assert_allclose(Chain(moved, "tip", "tcp_link").forward({})[:3, 3], [0.0, 0.0, 0.05])
    with pytest.raises(ValueError, match="not fixed to l3"):
        add_tcp(moved, "l3", "tcp_link", TOOL)


def test_the_tool_refuses_zeros_from_a_pin_and_from_too_few_dots():
    """ARM-NO-1"""
    with pytest.raises(SystemExit, match="joint zeros need dots"):
        calibrate_arm.check("zeros", "pin", 1)
    with pytest.raises(SystemExit, match="three dots or more"):
        calibrate_arm.check("zeros+tcp", "dots", 2)
    calibrate_arm.check("tcp", "pin", 1)


def test_a_tool_centre_point_is_refitted_and_applied(tmp_path):
    """ARM-OUT-2, ARM-OUT-4, ARM-OUT-5, ARM-OUT-6"""
    urdf, joints, _free = ARMS["six"]
    profile = write_profile(tmp_path, joints, [])
    (tmp_path / "description.urdf").write_text(urdf)
    chain = Chain(urdf, "base", "tip")
    touches = hand(chain, joints, {}, TOOL, np.random.default_rng(3), dots=PIN, noise=0.0002)
    (tmp_path / "touches.json").write_text(
        json.dumps(
            {
                "profile": "test_arm",
                "reference": "pin",
                "solve": "tcp",
                "read": {},
                "touches": [{"dot": t.dot, "shape": "down", "positions": t.positions} for t in touches],
            }
        )
    )

    calibrate_arm.main(["--profile", profile, "--refit", str(tmp_path / "touches.json"), "--out", str(tmp_path)])

    tcp = yaml.safe_load((tmp_path / "tcp.yaml").read_text())
    assert (tcp["parent"], tcp["link"], tcp["reference"]) == ("tip", "tcp_link", "pin")
    np.testing.assert_allclose(tcp["xyz_m"], TOOL, atol=0.0015)
    assert not (tmp_path / "joint_zeros.yaml").exists()
    calibrated = (tmp_path / "test_arm_calibrated.urdf").read_text()
    np.testing.assert_allclose(Chain(calibrated, "tip", "tcp_link").forward({})[:3, 3], tcp["xyz_m"], atol=1e-6)

    apply_calibration.main(
        [
            "--urdf",
            str(tmp_path / "description.urdf"),
            "--tcp",
            str(tmp_path / "tcp.yaml"),
            "--out",
            str(tmp_path / "applied.urdf"),
        ]
    )
    assert (tmp_path / "applied.urdf").read_text() == calibrated


class _ArmAtHand:
    """A robot whose joints read each of `poses` in turn, one per touch, never released
    (`--release none`)."""

    topics = ("/joint_states", "/robot_description", "/controller_manager")
    released = None

    def __init__(self, poses):
        self.poses, self.taken = list(poses), 0

    def positions(self, joints):
        return dict(self.poses[min(self.taken, len(self.poses) - 1)])

    def still_positions(self, joints):
        self.taken += 1
        return dict(self.poses[self.taken - 1])


class _ArmReleased(_ArmAtHand):
    """The same robot, released through ros2_control."""

    def hardware_for(self, joints, name=None):
        return types.SimpleNamespace(name="TestArm")

    def release(self, hardware):
        self.released = (hardware.name, [])
        return []

    def updating(self):
        return True


def poses(chain, joints, dot, shapes, leans=LEANS):
    """The true arm's joint positions with its tip on `dot` in each of `shapes`."""
    touches = hand(
        chain, joints, {}, np.zeros(3), np.random.default_rng(0), dots=[dot], noise=0.0, shapes=shapes, leans=leans
    )
    assert len(touches) == len(shapes)
    return [t.positions for t in touches]


def answering(monkeypatch, answers) -> list[str]:
    """Answer the tool's questions with `answers`; the questions asked are kept in the list returned."""
    answers, asked = iter(answers), []

    def reply(prompt=""):
        asked.append(prompt)
        return next(answers)

    monkeypatch.setattr("builtins.input", reply)
    return asked


def test_recording_asks_for_each_shape_and_keeps_every_touch(tmp_path, monkeypatch, capsys):
    """ARM-REC-1, ARM-REC-3, ARM-REC-4, ARM-REL-5"""
    urdf, joints, _free = ARMS["six"]
    chain = Chain(urdf, "base", "tip")
    profile = profiles.load(write_profile(tmp_path, joints, []))
    down, _towards, away, left, right = poses(chain, joints, PIN[0], FULL)
    # Record down, skip towards, record away, redo it, then record left and right.
    answering(monkeypatch, ["", "s", "", "r", "", "", ""])
    args = argparse.Namespace(release="none", reference="pin", count=1, solve="tcp", ros2_control=None)

    saved, about = calibrate_arm.record(args, profile, _ArmAtHand([down, away, away, left, right]), chain, tmp_path)

    assert [t["shape"] for t in saved] == ["down", "away", "left", "right"]
    assert "ros2_control" not in about
    kept = json.loads((tmp_path / "touches.json").read_text())
    assert (kept["reference"], kept["solve"], len(kept["touches"])) == ("pin", "tcp", 4)
    out = capsys.readouterr().out
    assert "Removed the last touch" in out
    assert out.count("Recorded:") == 5
    assert "its top away from the base" in out
    assert calibrate_arm.RELEASE == {"dots": "lifecycle", "pin": "none"}


def test_the_lean_is_said_from_the_description(tmp_path):
    """ARM-REC-3"""
    urdf, joints, _free = ARMS["six"]
    chain = Chain(urdf, "base", "tip")
    profile = profiles.load(write_profile(tmp_path, joints, []))
    for shape, pose in zip(FULL, poses(chain, joints, DOTS[1], FULL), strict=True):
        lean = calibrate_arm.lean_of(chain, profile, pose)
        words = calibrate_arm.lean_words(lean)
        if shape == "down":
            assert lean.way is None and words.startswith("pointing down")
        else:
            assert lean.way == shape and lean.deg == pytest.approx(40.0, abs=5.0)
            assert words.endswith(calibrate_arm.WAY_WORDS[shape])


def test_the_lean_is_shown_live_on_a_terminal(monkeypatch, capsys):
    """ARM-REC-3"""
    leans = iter(["pointing down (2 deg from straight down)", "leaning 9 deg, its top towards the base"])
    shown = []

    class Terminal:
        def isatty(self):
            return True

        def readline(self):
            return "\n"

    def ready(*_args):
        shown.append(None)
        return ([Terminal()] if len(shown) > 3 else [], [], [])

    monkeypatch.setattr(calibrate_arm.sys, "stdin", Terminal())
    monkeypatch.setattr(calibrate_arm.select, "select", ready)
    status = [next(leans, "leaning 9 deg, its top towards the base") for _ in range(5)]
    assert calibrate_arm.ask("Touch it", status=lambda: status.pop(0)) == ""
    out = capsys.readouterr().out
    assert "Touch it" in out
    assert "pointing down (2 deg" in out and out.count("leaning 9 deg, its top towards the base") == 1


def test_a_touch_is_taken_only_in_its_shape(tmp_path, monkeypatch, capsys):
    """ARM-REC-5"""
    urdf, joints, _free = ARMS["six"]
    chain = Chain(urdf, "base", "tip")
    profile = profiles.load(write_profile(tmp_path, joints, []))
    shaped = dict(zip(FULL, poses(chain, joints, DOTS[1], FULL), strict=True))
    for asked in FULL:
        for held, pose in shaped.items():
            problem = calibrate_arm.shape_problem(asked, calibrate_arm.lean_of(chain, profile, pose), 15.0)
            assert (problem is None) == (asked == held), (asked, held, problem)
    (slight,) = poses(chain, joints, DOTS[1], ["towards"], leans=(12.0,))
    assert "leans only 12 deg" in calibrate_arm.shape_problem(
        "towards", calibrate_arm.lean_of(chain, profile, slight), 15.0
    )

    # Asked to lean towards the base, the person leans away, then too little, then as asked.
    answering(monkeypatch, ["", "", "", "q"])
    args = argparse.Namespace(release="none", reference="pin", count=1, solve="tcp", ros2_control=None)
    profile.shapes = ["towards", "away"]
    arm = _ArmAtHand([shaped["away"], slight, shaped["towards"]])
    saved, _ = calibrate_arm.record(args, profile, arm, chain, tmp_path)
    assert [t["shape"] for t in saved] == ["towards"]
    out = capsys.readouterr().out
    assert "its top moves away from the base: tilt it so its top moves towards the base" in out
    assert "leans only 12 deg: tilt it at least 15 deg" in out


def test_the_tool_says_where_the_dots_can_go(tmp_path, capsys):
    """ARM-REC-6"""
    chain = Chain(FIVE_LIMITED, "base", "tip")
    joints = ARMS["five"][1]
    profile = profiles.load(write_profile(tmp_path, joints, [], shapes=IN_PLANE))
    calibrate_arm.suggest_dots(chain, profile, 3)
    out = capsys.readouterr().out
    near, far = (float(v) / 100 for v in re.search(r"Dots can go (\d+) to (\d+) cm", out).groups())
    places = [float(v) / 100 for v in re.findall(r"dot \d: about (\d+) cm", out)]
    assert places[0] == pytest.approx(near) and places[-1] == pytest.approx(far) and len(places) == 3
    # The simulated person can lean 15 degrees each way within that span, and not beyond it.
    for distance in (near, (near + far) / 2, far):
        poses(chain, joints, (distance, 0.0, 0.0), ["towards", "away"], leans=(15.0,))
    beyond = hand(
        chain,
        joints,
        {},
        np.zeros(3),
        np.random.default_rng(0),
        dots=[(far + 0.04, 0.0, 0.0)],
        noise=0.0,
        shapes=["towards", "away"],
        leans=(15.0,),
    )
    assert len(beyond) < 2


def test_a_dot_is_marked_only_where_every_lean_can_be_made(tmp_path):
    """ARM-REC-7"""
    chain = Chain(FIVE_LIMITED, "base", "tip")
    joints = ARMS["five"][1]
    profile = profiles.load(write_profile(tmp_path, joints, [], shapes=IN_PLANE))
    down, _towards, away = poses(chain, joints, DOTS[1], IN_PLANE)
    good, words = calibrate_arm.spot_check(chain, profile, down, [])
    assert good, words
    assert "18 cm from the base" in words

    # The reach it states is a lean the simulated person can make there, and 10 degrees more is not.
    for way in ("towards", "away"):
        reach = arm_reach.reach(
            chain,
            [down[j] for j in joints],
            calibrate_arm.lean_of(chain, profile, down).tip,
            way,
            [0, 0, 0],
            [0, 0, 1],
            [0, 0, 1],
        )
        assert f"up to {reach:.0f} deg" in words
        poses(chain, joints, DOTS[1], [way], leans=(reach,))
        more = hand(
            chain,
            joints,
            {},
            np.zeros(3),
            np.random.default_rng(1),
            dots=[DOTS[1]],
            noise=0.0,
            shapes=[way],
            leans=(reach + 10.0,),
        )
        assert reach + 10.0 > arm_reach.MOST_DEG or not more

    tip = calibrate_arm.lean_of(chain, profile, down).tip
    good, words = calibrate_arm.spot_check(chain, profile, down, [tip + np.array([0.03, 0.0, 0.0])])
    assert not good and "only 3 cm from dot 1" in words
    good, words = calibrate_arm.spot_check(chain, profile, away, [])
    assert not good and "point the tool straight down" in words
    (far,) = poses(chain, joints, (0.26, 0.0, 0.0), ["down"])
    good, words = calibrate_arm.spot_check(chain, profile, far, [])
    assert not good and "needs 15 deg each way" in words


def test_a_dot_whose_touches_disagree_can_be_touched_again(tmp_path, monkeypatch, capsys):
    """ARM-REC-8"""
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    profile = profiles.load(write_profile(tmp_path, joints, free, shapes=IN_PLANE))
    here = poses(chain, joints, DOTS[1], IN_PLANE)
    elsewhere = poses(chain, joints, DOTS[2], IN_PLANE)
    # The spot; three touches, the last on another dot; touch the dot again; three touches.
    asked = answering(monkeypatch, ["", "", "", "", "y", "", "", ""])
    args = argparse.Namespace(release="none", reference="dots", count=1, solve="zeros", ros2_control=None)
    arm = _ArmAtHand([here[0], here[0], here[1], elsewhere[2], *here])

    saved, _ = calibrate_arm.record(args, profile, arm, chain, tmp_path)

    assert [t["positions"] for t in saved] == here
    assert len(json.loads((tmp_path / "touches.json").read_text())["touches"]) == 3
    assert re.search(r"up to \d+ mm apart", asked[4])
    out = capsys.readouterr().out
    assert "Dots can go" in out and "Spot accepted" in out


def test_an_arm_that_keeps_its_torque_is_restored(tmp_path, monkeypatch):
    """ARM-REL-6"""
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    profile = profiles.load(write_profile(tmp_path, joints, free, shapes=IN_PLANE))
    answering(monkeypatch, ["", "n"])
    args = argparse.Namespace(release="lifecycle", reference="dots", count=3, solve="zeros", ros2_control=None)
    arm = _ArmReleased(poses(chain, joints, DOTS[1], IN_PLANE))
    with pytest.raises(SystemExit, match="kept their torque"):
        calibrate_arm.record(args, profile, arm, chain, tmp_path)
    assert arm.released == ("TestArm", [])  # left for main to restore, as ARM-REL-4 has it


def test_a_touch_is_taken_only_while_the_arm_is_still():
    """ARM-REC-2"""
    ros_arm = pytest.importorskip("arm_calibration.ros_arm")

    class Readings:
        def __init__(self, values):
            self.values = iter(values)

        def positions(self, joints):
            return dict.fromkeys(joints, next(self.values))

    still = ros_arm.RosArm.still_positions(Readings([0.100, 0.102, 0.104]), ["a"], samples=3, period_s=0.0)
    assert still == pytest.approx({"a": 0.102})
    assert ros_arm.RosArm.still_positions(Readings([0.1, 0.1, 0.106]), ["a"], samples=3, period_s=0.0) is None


def test_the_report_names_poor_values_and_far_touches(tmp_path, capsys):
    """ARM-REP-1, ARM-REP-2"""
    _urdf, joints, _free = ARMS["five"]
    profile = profiles.load(write_profile(tmp_path, joints, ["lift", "elbow"]))
    result = zero_fit.Result(
        zeros={"lift": math.radians(1.0), "elbow": math.radians(-2.0)},
        sigma={"lift": math.radians(0.2), "elbow": math.radians(0.8)},
        tip_offset=np.array([0.0, 0.0, 0.035]),
        tip_sigma=np.array([0.0002, 0.0002, 0.0009]),
        dots=np.zeros((1, 3)),
        rms_m=0.0006,
        residuals_m=[0.0005, 0.0031],
    )
    saved = [{"dot": 0, "shape": "down"}, {"dot": 0, "shape": "away"}]
    calibrate_arm.report(result, saved, profile, "zeros+tcp", "dots")
    out = capsys.readouterr().out
    assert "lift" in out and "+1.00 deg" in out
    assert out.count("poorly measured") == 2  # the elbow's zero, and the TCP's z
    assert "held: turns every dot with it" in out
    assert "away: 3.1 mm off" in out


def test_the_interfaces_are_chosen_on_the_command_line(tmp_path, monkeypatch):
    """ARM-IF-4"""
    ros_arm = pytest.importorskip("arm_calibration.ros_arm")
    asked = {}

    class Robot:
        released = None

        def __init__(self, joint_states, robot_description, controller_manager):
            asked.update(js=joint_states, rd=robot_description, cm=controller_manager)

        def description(self):
            raise ros_arm.ContractError("no description")

        def close(self):
            pass

    monkeypatch.setattr(ros_arm, "RosArm", Robot)
    _urdf, joints, _free = ARMS["five"]
    with pytest.raises(SystemExit, match="does not meet the contract"):
        calibrate_arm.main(
            [
                "--profile",
                write_profile(tmp_path, joints, []),
                "--solve",
                "tcp",
                "--reference",
                "pin",
                "--joint-states",
                "/arm/joint_states",
                "--robot-description",
                "/arm/robot_description",
                "--controller-manager",
                "/arm/controller_manager",
                "--out",
                str(tmp_path),
            ]
        )
    assert asked == {"js": "/arm/joint_states", "rd": "/arm/robot_description", "cm": "/arm/controller_manager"}
