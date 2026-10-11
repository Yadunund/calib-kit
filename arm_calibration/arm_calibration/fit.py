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

"""Joint zero offsets from touches on dots on a table.

The arm's torque is off and its tip is rested, by hand, on a few dots marked anywhere on the
table the arm stands on, each dot in several arm shapes (the tool straight down, and leaning
towards the base, away from it and to either side). Every touch of one dot puts the tip on the
same point, whatever the joints read, so the joints' zero offsets are the ones that make the
description agree with itself about where each dot is.

The dots need not be measured: their positions are fitted too. They lie on one plane, the
table's, whose normal in the base frame is known (`plane_normal`: the base's own z axis for an
arm clamped flat to the table) and whose height is fitted. The plane is what fixes a shoulder
that lifts the arm: with each dot free in height, lifting the whole arm about that joint would
move every dot along with it and fit as well.

Some zeros cannot be found from such touches at all, and the arm's profile leaves them out of
`free_joints`: a joint whose axis is the table's normal through the base (turning it turns every
dot with it and fits as well), and a joint whose axis passes through the tip (turning it does not
move the tip). `Result.sigma` shows how well each fitted zero was measured.

The tip is where the profile says it is in the description's tip frame (`tip_offset`, its expected
place), give or take a few millimetres (`TIP_SIGMA_M`): fitted, held lightly to that place.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from arm_calibration.urdf_chain import Chain

# How far a hand rests the tip from the dot's centre, metres: one standard deviation.
TOUCH_SIGMA_M = 0.0005
# How far the tip is expected from where the profile puts it in the tip frame, metres.
TIP_SIGMA_M = 0.003
# How far a zero is expected to be off, radians: a weak prior, only to keep a fit bounded.
ZERO_SIGMA_RAD = math.radians(10.0)


@dataclass
class Touch:
    """The tip resting on dot `dot`, with the joints reading `positions` (radians)."""

    dot: int
    positions: dict[str, float]


@dataclass
class Result:
    zeros: dict[str, float]  # radians, to add to each joint's reading
    sigma: dict[str, float]  # one standard deviation of each zero, radians
    tip_offset: np.ndarray  # the tip in the tip frame, metres
    dots: np.ndarray  # (N, 3), base frame, metres
    rms_m: float  # touched tips from their dots after the fit
    residuals_m: list[float] = field(default_factory=list)  # each touch's distance from its dot


def _plane_basis(normal) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    return n, u, np.cross(n, u)


def fit(
    chain: Chain, touches: list[Touch], free_joints, plane_normal=(0.0, 0.0, 1.0), tip_offset=(0.0, 0.0, 0.0)
) -> Result:
    """The zero offsets of `free_joints` (and the dots, the plane's height and the tip, expected at
    `tip_offset` in the tip frame) that best explain `touches`."""
    free = list(free_joints)
    unknown = [j for j in free if j not in chain.movable]
    if unknown:
        raise ValueError(f"not joints of the chain: {', '.join(unknown)}")
    dots = sorted({t.dot for t in touches})
    index = {d: i for i, d in enumerate(dots)}
    for d in dots:
        if sum(t.dot == d for t in touches) < 2:
            raise ValueError(f"dot {d} has one touch: each dot needs several, in different arm shapes")
    n, u, v = _plane_basis(plane_normal)
    expected_tip = np.asarray(tip_offset, float)
    nz, nd = len(free), len(dots)

    def unpack(x):
        zeros = dict(zip(free, x[:nz], strict=True))
        tip = x[nz : nz + 3]
        ab = x[nz + 3 : nz + 3 + 2 * nd].reshape(nd, 2)
        points = x[-1] * n + ab[:, :1] * u + ab[:, 1:] * v
        return zeros, tip, points

    def tip_of(touch, zeros, tip):
        pose = chain.forward(touch.positions, zeros)
        return pose[:3, :3] @ tip + pose[:3, 3]

    def residual(x):
        zeros, tip, points = unpack(x)
        out = [(tip_of(t, zeros, tip) - points[index[t.dot]]) / TOUCH_SIGMA_M for t in touches]
        out.append(x[:nz] / ZERO_SIGMA_RAD)
        out.append((tip - expected_tip) / TIP_SIGMA_M)
        return np.concatenate(out)

    first = {d: next(t for t in touches if t.dot == d) for d in dots}
    starts = np.array([tip_of(first[d], {}, expected_tip) for d in dots])
    x0 = np.concatenate(
        [np.zeros(nz), expected_tip, np.column_stack([starts @ u, starts @ v]).ravel(), [float(np.mean(starts @ n))]]
    )
    solution = least_squares(residual, x0, loss="soft_l1", f_scale=3.0, x_scale="jac")
    zeros, tip, points = unpack(solution.x)
    distances = [float(np.linalg.norm(tip_of(t, zeros, tip) - points[index[t.dot]])) for t in touches]
    rms = float(np.sqrt(np.mean(np.square(distances))))
    # Uncertainty from the normal matrix, the touches' noise taken as what the fit leaves.
    scale = max(rms / TOUCH_SIGMA_M, 1.0)
    try:
        cov = np.linalg.inv(solution.jac.T @ solution.jac) * scale**2
        sigma = {j: float(math.sqrt(max(cov[i, i], 0.0))) for i, j in enumerate(free)}
    except np.linalg.LinAlgError:
        sigma = dict.fromkeys(free, float("inf"))
    return Result({j: float(v) for j, v in zeros.items()}, sigma, tip, points, rms, distances)
