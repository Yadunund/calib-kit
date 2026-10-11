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

import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from arm_calibration.urdf_chain import Chain, add_tcp, apply_zeros, matrix_to_rpy, rpy_to_matrix
from scipy.optimize import least_squares

from arm_calibration import apply_calibration, calibrate_arm, profiles
from arm_calibration import fit as zero_fit

FULL = ["down", "towards", "away", "left", "right"]
# An arm whose wrist cannot lean sideways leans twice each way in the plane it reaches in.
IN_PLANE = ["down", "towards 20", "towards 40", "away 20", "away 40"]
TILT = math.radians(40)
DOTS = np.array([(0.18, -0.10, 0.0), (0.27, 0.05, 0.0), (0.20, 0.14, 0.0)])
DOTS5 = np.array([(0.18, -0.10, 0.0), (0.27, 0.05, 0.0), (0.20, 0.14, 0.0), (0.30, -0.08, 0.0), (0.15, 0.04, 0.0)])


def joint(name, parent, child, xyz, rpy="0 0 0", axis="0 0 1", kind="revolute"):
    return (
        f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
        f'<origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis}"/>'
        '<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
    )


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
PROTOCOL = {"five": (IN_PLANE, DOTS5), "six": (FULL, DOTS)}


def approach(point, shape, tilt=TILT):
    radial = np.array([point[0], point[1], 0.0]) / math.hypot(point[0], point[1])
    across = np.array([-radial[1], radial[0], 0.0])
    lean = {"down": np.zeros(3), "towards": -radial, "away": radial, "left": across, "right": -across}[shape]
    direction = np.array([0.0, 0.0, -math.cos(tilt)]) + math.sin(tilt) * lean
    return direction / np.linalg.norm(direction)


def hand(chain, joints, true_zeros, tip, rng, dots=DOTS, noise=0.0005, table=None, shapes=FULL):
    """A person's touches: the true arm put with its tip (`tip` in the tip frame) in each dot,
    missed by `noise`, the tool along each shape; the joints read the true angles less the zeros.
    `table` turns the dots and shapes with a table that is not square to the base."""
    table = np.eye(3) if table is None else table
    touches = []
    for d, dot in enumerate(dots):
        for shape in shapes:
            direction, tilt = profiles.parse_shape(shape, 40.0)
            target = table @ (dot + np.array([*rng.normal(0.0, noise, 2), 0.0]))
            pointing = table @ approach(dot, direction, math.radians(tilt))
            best = None
            for start in range(6):
                guess = rng.uniform(-1.0, 1.0, len(joints))
                guess[0] = math.atan2(target[1], target[0])

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

                solution = least_squares(residual, guess)
                if best is None or solution.cost < best.cost:
                    best = solution
                if np.linalg.norm(best.fun[:3]) < 0.01 and start >= 1:
                    break
            # A shape the arm cannot take there (out of reach, or a lean its joints cannot make) is
            # skipped, as a person would.
            if np.linalg.norm(best.fun[:3]) > 0.05 or np.linalg.norm(best.fun[3:6]) > math.radians(5):
                continue
            touches.append(
                zero_fit.Touch(d, {j: float(t - true_zeros.get(j, 0.0)) for j, t in zip(joints, best.x, strict=True)})
            )
    return touches


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
        assert len(touches) >= 12
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
    touches = hand(chain, joints, true_zeros, np.zeros(3), rng, dots=DOTS5, table=table, shapes=IN_PLANE)
    result = zero_fit.fit(chain, touches, free, plane_normal=table[:, 2])
    for joint_name, value in true_zeros.items():
        assert result.zeros[joint_name] == pytest.approx(value, abs=math.radians(0.3))


def test_a_tip_beyond_its_frame_is_given_by_the_profile():
    """ARM-FIT-5, ARM-FIT-6"""
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    tip = np.array([0.0, 0.0, 0.008])
    true_zeros = {"lift": 0.03, "elbow": -0.02, "pitch": 0.04}
    touches = hand(chain, joints, true_zeros, tip, np.random.default_rng(5), dots=DOTS5, shapes=IN_PLANE)
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


def write_profile(tmp_path, joints, free) -> str:
    profile = {
        "name": "test_arm",
        "description": "an arm made up for the tests",
        "base_link": "base",
        "tip_link": "tip",
        "free_joints": free,
        "held_joints": {joints[0]: "turns every dot with it"},
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


def test_shipped_profiles_load():
    """ARM-CFG-2, ARM-CFG-4"""
    assert "so101" in profiles.available()
    so101 = profiles.load("so101")
    assert so101.free_joints == ["shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint"]
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
        dots=DOTS5,
        shapes=profile.shapes,
    )
    result = zero_fit.fit(chain, touches, profile.free_joints, profile.plane_normal, profile.tip_offset)
    for joint_name, value in true_zeros.items():
        assert result.zeros[joint_name] == pytest.approx(value, abs=math.radians(0.3))


# A tool on the arm: its point 35 mm out along the tool and a few millimetres off its axis.
TOOL = np.array([0.004, -0.003, 0.035])
# A sharp pin fixed above the table, within reach.
PIN = np.array([[0.24, 0.02, 0.05]])


@pytest.mark.parametrize("arm", list(ARMS))
def test_a_pin_finds_the_tool_centre_point_of_an_arm_with_true_zeros(arm):
    """ARM-FIT-3, ARM-FIT-5, ARM-FIT-6, ARM-FIT-7"""
    urdf, joints, _free = ARMS[arm]
    chain = Chain(urdf, "base", "tip")
    for seed in range(3):
        touches = hand(
            chain, joints, {}, TOOL, np.random.default_rng(seed), dots=PIN, noise=0.0002, shapes=PROTOCOL[arm][0]
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
    """A robot whose joints read a new pose for each touch, never released (`--release none`)."""

    topics = ("/joint_states", "/robot_description", "/controller_manager")
    released = None

    def __init__(self, joints):
        self.joints, self.taken = joints, 0

    def positions(self, joints):
        return dict.fromkeys(joints, 0.1)

    def still_positions(self, joints):
        self.taken += 1
        return {j: 0.1 * self.taken for j in joints}


def test_recording_asks_for_each_shape_and_keeps_every_touch(tmp_path, monkeypatch, capsys):
    """ARM-REC-1, ARM-REC-3, ARM-REC-4, ARM-REL-5"""
    import argparse

    urdf, joints, _free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    profile = profiles.load(write_profile(tmp_path, joints, []))
    answers = iter(["", "s", "", "", ""])
    monkeypatch.setattr("builtins.input", lambda _prompt="": next(answers))
    args = argparse.Namespace(release="none", reference="pin", count=1, solve="tcp", ros2_control=None)

    saved, about = calibrate_arm.record(args, profile, _ArmAtHand(joints), chain, tmp_path)

    assert [t["shape"] for t in saved] == ["down", "away", "left", "right"]
    assert "ros2_control" not in about
    kept = json.loads((tmp_path / "touches.json").read_text())
    assert (kept["reference"], kept["solve"], len(kept["touches"])) == ("pin", "tcp", 4)
    assert capsys.readouterr().out.count("recorded:") == 4
    assert calibrate_arm.RELEASE == {"dots": "lifecycle", "pin": "none"}


def test_the_lean_is_said_from_the_description():
    """ARM-REC-3"""
    urdf, joints, _free = ARMS["six"]
    chain = Chain(urdf, "base", "tip")
    touches = hand(chain, joints, {}, np.zeros(3), np.random.default_rng(0), dots=PIN, noise=0.0)
    words = [calibrate_arm.lean(chain, t.positions, [0, 0, 1]) for t in touches]
    assert words[0].endswith("from straight down")
    assert {"towards the base", "away from the base", "to the left", "to the right"} == {
        w.split("deg ")[1] for w in words[1:]
    }


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
