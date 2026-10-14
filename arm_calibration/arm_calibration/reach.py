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

"""How the tool leans, and how far it can lean, by the description.

Down is into the table (against the profile's `plane_normal`). A lean is the angle between the
tool's axis, from its top to its tip, and down. Its way is where the tool's top moves, seen along
the line from the base to the tip: `towards` the base, `away` from it, `left` or `right`.

A spot's reach for one way is how far the tool can lean that way with the tip held on the spot,
within the description's joint limits: what tells a person, before they mark a dot, whether the
arm can make every lean there.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from arm_calibration.urdf_chain import Chain

WAYS = ("towards", "away", "left", "right")
# A lean this small or less, degrees, is down.
DOWN_DEG = 10.0
# Reach is found in steps of this, degrees, up to the most.
STEP_DEG = 5.0
MOST_DEG = 60.0
# A pose holds the tip on a spot within this, metres, and the tool's lean within this, degrees.
HOLD_M = 0.001
HOLD_DEG = 2.0
# Solver evaluations a pose is searched for: a reachable one, started near, takes far fewer.
SOLVE_STEPS = 40


@dataclass
class Lean:
    tip: np.ndarray  # the tip, base frame, metres
    deg: float  # degrees from down
    way: str | None  # where the tool's top moves; None when down


def _up(plane_normal) -> np.ndarray:
    n = np.asarray(plane_normal, float)
    return n / np.linalg.norm(n)


def ways(tip, plane_normal) -> dict[str, np.ndarray]:
    """Each way as the unit direction, in the table's plane, the tool's top moves at `tip`."""
    up = _up(plane_normal)
    radial = np.asarray(tip, float) - (np.asarray(tip, float) @ up) * up
    if np.linalg.norm(radial) < 1e-6:
        radial = np.array([1.0, 0.0, 0.0]) - up[0] * up
    radial /= np.linalg.norm(radial)
    left = np.cross(up, radial)
    return {"towards": -radial, "away": radial, "left": left, "right": -left}


def pointing(tip, way: str | None, deg: float, plane_normal) -> np.ndarray:
    """The tool's axis, top to tip, leaning `deg` degrees `way` at `tip`."""
    up = _up(plane_normal)
    if way is None or deg == 0.0:
        return -up
    t = math.radians(deg)
    return -math.cos(t) * up - math.sin(t) * ways(tip, plane_normal)[way]


def _tip_and_axis(chain: Chain, positions, tip_offset, tool_axis) -> tuple[np.ndarray, np.ndarray]:
    pose = chain.forward(positions)
    axis = pose[:3, :3] @ np.asarray(tool_axis, float)
    return pose[:3, :3] @ np.asarray(tip_offset, float) + pose[:3, 3], axis / np.linalg.norm(axis)


def lean(chain: Chain, positions, tip_offset, tool_axis, plane_normal) -> Lean:
    """How the tool leans with the joints at `positions`."""
    tip, axis = _tip_and_axis(chain, positions, tip_offset, tool_axis)
    up = _up(plane_normal)
    deg = math.degrees(math.acos(max(-1.0, min(1.0, float(-axis @ up)))))
    top = -axis - float(-axis @ up) * up
    if deg <= DOWN_DEG or np.linalg.norm(top) < 1e-9:
        return Lean(tip, deg, None)
    way = max(WAYS, key=lambda w: float(top @ ways(tip, plane_normal)[w]))
    return Lean(tip, deg, way)


def _cross(a, b) -> np.ndarray:
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def _kinematics(chain: Chain, q, tip_offset, tool_axis, jacobians: bool = True):
    """The tip, the tool's axis, and (with `jacobians`) both's Jacobians in the movable joints `q`."""
    t = np.eye(4)
    frames = []
    values = iter(q)
    for joint in chain.joints:
        if joint.kind == "fixed":
            t = t @ joint.transform(0.0)
            continue
        at = t @ joint.transform(0.0)
        frames.append((joint.kind, at[:3, :3] @ joint.axis, at[:3, 3].copy()))
        t = t @ joint.transform(next(values))
    tip = t[:3, :3] @ np.asarray(tip_offset, float) + t[:3, 3]
    axis = t[:3, :3] @ np.asarray(tool_axis, float)
    axis = axis / np.linalg.norm(axis)
    if not jacobians:
        return tip, axis, None, None
    j_tip = np.zeros((3, len(frames)))
    j_axis = np.zeros((3, len(frames)))
    for i, (kind, a, origin) in enumerate(frames):
        if kind == "prismatic":
            j_tip[:, i] = a
        else:
            j_tip[:, i] = _cross(a, tip - origin)
            j_axis[:, i] = _cross(a, axis)
    return tip, axis, j_tip, j_axis


def hold(chain: Chain, start, spot, axis, tip_offset, tool_axis) -> np.ndarray | None:
    """Joint positions (in `chain.movable`'s order), within the joint limits and from `start`,
    that hold the tip on `spot` with the tool along `axis`; None if none is found."""
    names = chain.movable
    lo = np.array([chain.limits[n][0] for n in names])
    hi = np.array([chain.limits[n][1] for n in names])
    inside = np.clip(np.asarray(start, float), lo + 1e-6, hi - 1e-6)
    scale = np.array([1 / HOLD_M] * 3 + [1 / math.radians(HOLD_DEG)] * 3)

    def residual(q):
        tip, along, _, _ = _kinematics(chain, q, tip_offset, tool_axis, jacobians=False)
        return np.concatenate([tip - spot, along - axis]) * scale

    def jacobian(q):
        _, _, j_tip, j_axis = _kinematics(chain, q, tip_offset, tool_axis)
        return np.vstack([j_tip, j_axis]) * scale[:, None]

    q = least_squares(residual, inside, jac=jacobian, bounds=(lo, hi), max_nfev=SOLVE_STEPS).x
    tip, along, _, _ = _kinematics(chain, q, tip_offset, tool_axis)
    off = math.degrees(math.acos(max(-1.0, min(1.0, float(along @ axis)))))
    return q if np.linalg.norm(tip - spot) <= HOLD_M and off <= HOLD_DEG else None


def reach(chain: Chain, start, spot, way: str, tip_offset, tool_axis, plane_normal, most=MOST_DEG) -> float | None:
    """How far, in steps of `STEP_DEG` up to `most` degrees, the tool can lean `way` with the tip
    on `spot`, leaning from a down pose found from the joint positions `start`; None if the tip
    cannot be held there even down."""
    q = hold(chain, start, spot, pointing(spot, None, 0.0, plane_normal), tip_offset, tool_axis)
    if q is None:
        return None
    best, deg = 0.0, STEP_DEG
    while deg <= most + 1e-9:
        q = hold(chain, q, spot, pointing(spot, way, deg, plane_normal), tip_offset, tool_axis)
        if q is None:
            break
        best, deg = deg, deg + STEP_DEG
    return best


def _starts(chain: Chain, rng, count: int) -> list[np.ndarray]:
    lo = np.array([max(chain.limits[n][0], -math.pi) for n in chain.movable])
    hi = np.array([min(chain.limits[n][1], math.pi) for n in chain.movable])
    return [rng.uniform(lo, hi) for _ in range(count)]


def spread(
    chain: Chain, asked: list[str], least_deg: float, tip_offset, tool_axis, plane_normal, step_m: float = 0.02
) -> tuple[float, float] | None:
    """The nearest and farthest distances from the base, along its x axis on the plane through its
    origin, at which every way in `asked` reaches `least_deg`; None if there is none."""
    up = _up(plane_normal)
    out = np.array([1.0, 0.0, 0.0]) - up[0] * up
    out /= np.linalg.norm(out)
    longest = sum(float(np.linalg.norm(j.origin_xyz)) for j in chain.joints) + float(np.linalg.norm(tip_offset))
    most = math.ceil(least_deg / STEP_DEG) * STEP_DEG
    rng = np.random.default_rng(0)
    good, previous = [], None
    for distance in np.arange(step_m * 4, longest, step_m):
        spot = distance * out
        down = pointing(spot, None, 0.0, plane_normal)
        start = None
        for seed in ([previous] if previous is not None else []) + _starts(chain, rng, 8):
            start = hold(chain, seed, spot, down, tip_offset, tool_axis)
            if start is not None:
                break
        previous = start
        if start is None:
            continue
        reaches = [reach(chain, start, spot, w, tip_offset, tool_axis, plane_normal, most) for w in asked]
        if all(r is not None and r >= least_deg for r in reaches):
            good.append(float(distance))
    return (good[0], good[-1]) if good else None
