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
from arm_calibration.urdf_chain import Chain, apply_zeros, matrix_to_rpy, rpy_to_matrix
from scipy.optimize import least_squares

from arm_calibration import calibrate_zeros, profiles
from arm_calibration import fit as zero_fit

SHAPES = ("down", "towards", "away", "left", "right")
TILT = math.radians(40)
DOTS = np.array([(0.18, -0.10, 0.0), (0.27, 0.05, 0.0), (0.20, 0.14, 0.0)])


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


def approach(point, shape, tilt=TILT):
    radial = np.array([point[0], point[1], 0.0]) / math.hypot(point[0], point[1])
    across = np.array([-radial[1], radial[0], 0.0])
    lean = {"down": np.zeros(3), "towards": -radial, "away": radial, "left": across, "right": -across}[shape]
    direction = np.array([0.0, 0.0, -math.cos(tilt)]) + math.sin(tilt) * lean
    return direction / np.linalg.norm(direction)


def hand(chain, joints, true_zeros, tip, rng, dots=DOTS, noise=0.0005, table=None):
    """A person's touches: the true arm put with its tip (`tip` in the tip frame) in each dot,
    missed by `noise`, the tool along each shape; the joints read the true angles less the zeros.
    `table` turns the dots and shapes with a table that is not square to the base."""
    table = np.eye(3) if table is None else table
    touches = []
    for d, dot in enumerate(dots):
        for shape in SHAPES:
            target = table @ (dot + np.array([*rng.normal(0.0, noise, 2), 0.0]))
            pointing = table @ approach(dot, shape)
            best = None
            for start in range(6):
                guess = rng.uniform(-1.0, 1.0, len(joints))
                guess[0] = math.atan2(target[1], target[0])

                def residual(theta, target=target, pointing=pointing):
                    pose = chain.forward(dict(zip(joints, theta, strict=True)))
                    return np.concatenate(
                        [(pose[:3, :3] @ tip + pose[:3, 3] - target) / 0.001, pose[:3, 2] - pointing, 0.01 * theta]
                    )

                solution = least_squares(residual, guess)
                if best is None or solution.cost < best.cost:
                    best = solution
                if np.linalg.norm(best.fun[:3]) < 0.01 and start >= 1:
                    break
            if np.linalg.norm(best.fun[:3]) > 0.05:  # out of reach in this shape
                continue
            touches.append(
                zero_fit.Touch(d, {j: float(t - true_zeros.get(j, 0.0)) for j, t in zip(joints, best.x, strict=True)})
            )
    return touches


def test_rpy_round_trip():
    for rpy in [(0.1, -0.4, 2.0), (-1.5708, -1.5708, 0.0), (3.0, 0.2, -1.0)]:
        np.testing.assert_allclose(rpy_to_matrix(*matrix_to_rpy(rpy_to_matrix(*rpy))), rpy_to_matrix(*rpy), atol=1e-9)


@pytest.mark.parametrize("arm", list(ARMS))
def test_zeros_folded_into_the_description_move_the_arm_as_the_offsets_do(arm):
    urdf, joints, free = ARMS[arm]
    zeros = {j: 0.01 * (i + 1) * (-1) ** i for i, j in enumerate(free)}
    plain, corrected = Chain(urdf, "base", "tip"), Chain(apply_zeros(urdf, zeros), "base", "tip")
    rng = np.random.default_rng(0)
    for _ in range(10):
        q = dict(zip(joints, rng.uniform(-1.2, 1.2, len(joints)), strict=True))
        np.testing.assert_allclose(corrected.forward(q), plain.forward(q, zeros), atol=1e-9)


def test_limits_keep_the_same_physical_travel():
    def limits(text):
        element = next(j for j in ET.fromstring(text).findall("joint") if j.get("name") == "elbow")
        return float(element.find("limit").get("lower")), float(element.find("limit").get("upper"))

    assert limits(apply_zeros(FIVE, {"elbow": 0.05})) == pytest.approx((-3.05, 2.95))


@pytest.mark.parametrize("arm", list(ARMS))
def test_touches_on_table_dots_recover_the_zeros(arm):
    urdf, joints, free = ARMS[arm]
    chain = Chain(urdf, "base", "tip")
    rng = np.random.default_rng(1)
    for _ in range(3):
        true_zeros = dict(zip(free, rng.uniform(-math.radians(3), math.radians(3), len(free)), strict=True))
        touches = hand(chain, joints, true_zeros, rng.uniform(-0.002, 0.002, 3), rng)
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
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    rng = np.random.default_rng(3)
    table = rpy_to_matrix(math.radians(4), math.radians(-3), 0.0)
    true_zeros = {"lift": 0.03, "elbow": -0.02, "pitch": 0.04}
    touches = hand(chain, joints, true_zeros, np.zeros(3), rng, table=table)
    result = zero_fit.fit(chain, touches, free, plane_normal=table[:, 2])
    for joint_name, value in true_zeros.items():
        assert result.zeros[joint_name] == pytest.approx(value, abs=math.radians(0.3))


def test_a_tip_beyond_its_frame_is_given_by_the_profile():
    urdf, joints, free = ARMS["five"]
    chain = Chain(urdf, "base", "tip")
    tip = np.array([0.0, 0.0, 0.008])
    true_zeros = {"lift": 0.03, "elbow": -0.02, "pitch": 0.04}
    touches = hand(chain, joints, true_zeros, tip, np.random.default_rng(5))
    result = zero_fit.fit(chain, touches, free, tip_offset=tip)
    for joint_name, value in true_zeros.items():
        error, sigma = abs(math.degrees(result.zeros[joint_name] - value)), math.degrees(result.sigma[joint_name])
        assert error < 3 * sigma
        assert error < 0.5
    np.testing.assert_allclose(result.tip_offset, tip, atol=0.001)


def test_a_dot_touched_once_is_refused():
    chain = Chain(FIVE, "base", "tip")
    touches = [zero_fit.Touch(0, {}), zero_fit.Touch(1, {}), zero_fit.Touch(1, {"elbow": 0.1})]
    with pytest.raises(ValueError, match="dot 0 has one touch"):
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

    calibrate_zeros.main(["--profile", profile, "--refit", str(tmp_path / "touches.json"), "--out", str(tmp_path)])

    written = yaml.safe_load((tmp_path / "joint_zeros.yaml").read_text())
    assert written["profile"] == "test_arm"
    for joint_name, value in true_zeros.items():
        assert written["joint_zeros_rad"][joint_name] == pytest.approx(value, abs=math.radians(0.3))
    corrected = Chain((tmp_path / "test_arm_calibrated.urdf").read_text(), "base", "tip")
    q = touches[0].positions
    np.testing.assert_allclose(corrected.forward(q), chain.forward(q, written["joint_zeros_rad"]), atol=1e-5)


def test_a_profile_with_unknown_settings_is_refused(tmp_path):
    (tmp_path / "arm.yaml").write_text(
        yaml.safe_dump(
            {"name": "a", "base_link": "b", "tip_link": "t", "free_joints": [], "hardware": {"type": "feetech"}}
        )
    )
    with pytest.raises(ValueError, match="unknown settings hardware"):
        profiles.load(str(tmp_path / "arm.yaml"))


def test_shipped_profiles_load():
    assert "so101" in profiles.available()
    so101 = profiles.load("so101")
    assert so101.free_joints == ["shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint"]
    assert profiles.load(str(profiles.ARMS / "so101.yaml")) == so101.__class__(**so101.__dict__)


def test_so101_zeros_on_its_own_description():
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
    touches = hand(chain, joints, true_zeros, np.zeros(3), np.random.default_rng(4))
    result = zero_fit.fit(chain, touches, profile.free_joints, profile.plane_normal)
    for joint_name, value in true_zeros.items():
        assert result.zeros[joint_name] == pytest.approx(value, abs=math.radians(0.3))
