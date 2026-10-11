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

"""Joint zero offsets and the tool centre point, from touches on fixed reference points.

The arm's tip (a gripper's closed jaws, a tool's point) is brought onto a fixed reference point
several times, the arm in a different shape each time: the tool straight down, and leaning
towards the base, away from it and to either side. Every touch of one reference puts the tip on
the same point, whatever the joints read. So the joints' zero offsets, and where the tip is on the
arm, are the ones that make the description agree with itself about where each reference is.

The references need not be measured; their positions are fitted too. They are either:

- `dots`: pen dots on the table the arm stands on. They lie on one plane, whose normal in the
  base frame is known (`plane_normal`: the base's own z axis for an arm clamped flat to the
  table) and whose height is fitted. The plane is what fixes a shoulder that lifts the arm: with
  each dot free in height, lifting the whole arm about that joint would move every dot along with
  it and fit as well. Joint zeros need dots, three or more.
- `pin`: sharp fixed points, each anywhere in space, the industrial way to find a tool centre
  point. A pin fixes where the tip is on the arm, not the joints' zeros.

Some zeros cannot be found from touches at all, and the arm's profile leaves them out of
`free_joints`: a joint whose axis is the table's normal through the base (turning it turns every
reference with it and fits as well), and a joint whose axis passes through the tip (turning it
does not move the tip).

The tip is expected where the profile puts it in the description's tip frame (`tip_offset`). With
`tip_free` it is fitted as freely as the touches allow (the tool centre point); otherwise it is
held within a few millimetres of that place (`TIP_SIGMA_M`). `Result` gives each fitted value's
uncertainty.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from arm_calibration.urdf_chain import Chain

REFERENCES = ("dots", "pin")
# How far the tip rests from the reference's centre, metres: one standard deviation.
TOUCH_SIGMA_M = 0.0005
# How far the tip is expected from where the profile puts it in the tip frame, metres.
TIP_SIGMA_M = 0.003
# How far the tip may be from where the profile puts it when it is fitted freely, metres: a
# weak prior, only to keep a fit bounded.
FREE_TIP_SIGMA_M = 1.0
# How far a zero is expected to be off, radians: a weak prior, only to keep a fit bounded.
ZERO_SIGMA_RAD = math.radians(10.0)


@dataclass
class Touch:
    """The tip resting on reference `dot`, with the joints reading `positions` (radians)."""

    dot: int
    positions: dict[str, float]


@dataclass
class Result:
    zeros: dict[str, float]  # radians, to add to each joint's reading
    sigma: dict[str, float]  # one standard deviation of each zero, radians
    tip_offset: np.ndarray  # the tip in the tip frame, metres
    tip_sigma: np.ndarray  # one standard deviation of each of its components, metres
    dots: np.ndarray  # (N, 3), base frame, metres
    rms_m: float  # touched tips from their references after the fit
    residuals_m: list[float] = field(default_factory=list)  # each touch's distance from its reference


def _plane_basis(normal) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    return n, u, np.cross(n, u)


def fit(
    chain: Chain,
    touches: list[Touch],
    free_joints,
    plane_normal=(0.0, 0.0, 1.0),
    tip_offset=(0.0, 0.0, 0.0),
    reference: str = "dots",
    tip_free: bool = False,
) -> Result:
    """The zero offsets of `free_joints`, the references and the tip (expected at `tip_offset`
    in the tip frame; fitted freely with `tip_free`) that best explain `touches`."""
    if reference not in REFERENCES:
        raise ValueError(f"unknown reference {reference!r}: one of {', '.join(REFERENCES)}")
    free = list(free_joints)
    unknown = [j for j in free if j not in chain.movable]
    if unknown:
        raise ValueError(f"not joints of the chain: {', '.join(unknown)}")
    dots = sorted({t.dot for t in touches})
    if free and reference != "dots":
        raise ValueError("joint zeros need dots on the table: a pin fixes the tip, not the zeros")
    index = {d: i for i, d in enumerate(dots)}
    for d in dots:
        if sum(t.dot == d for t in touches) < 2:
            raise ValueError(f"reference {d} has one touch: each needs several, in different arm shapes")
    n, u, v = _plane_basis(plane_normal)
    expected_tip = np.asarray(tip_offset, float)
    tip_sigma = FREE_TIP_SIGMA_M if tip_free else TIP_SIGMA_M
    nz, nd = len(free), len(dots)
    n_points = 2 * nd + 1 if reference == "dots" else 3 * nd

    def unpack(x):
        zeros = dict(zip(free, x[:nz], strict=True))
        tip = x[nz : nz + 3]
        p = x[nz + 3 : nz + 3 + n_points]
        if reference == "dots":
            ab = p[:-1].reshape(nd, 2)
            points = p[-1] * n + ab[:, :1] * u + ab[:, 1:] * v
        else:
            points = p.reshape(nd, 3)
        return zeros, tip, points

    def tip_of(touch, zeros, tip):
        pose = chain.forward(touch.positions, zeros)
        return pose[:3, :3] @ tip + pose[:3, 3]

    def residual(x):
        zeros, tip, points = unpack(x)
        out = [(tip_of(t, zeros, tip) - points[index[t.dot]]) / TOUCH_SIGMA_M for t in touches]
        out.append(x[:nz] / ZERO_SIGMA_RAD)
        out.append((tip - expected_tip) / tip_sigma)
        return np.concatenate(out)

    first = {d: next(t for t in touches if t.dot == d) for d in dots}
    starts = np.array([tip_of(first[d], {}, expected_tip) for d in dots])
    if reference == "dots":
        start_points = np.concatenate([np.column_stack([starts @ u, starts @ v]).ravel(), [float(np.mean(starts @ n))]])
    else:
        start_points = starts.ravel()
    x0 = np.concatenate([np.zeros(nz), expected_tip, start_points])
    solution = least_squares(residual, x0, loss="soft_l1", f_scale=3.0, x_scale="jac")
    zeros, tip, points = unpack(solution.x)
    distances = [float(np.linalg.norm(tip_of(t, zeros, tip) - points[index[t.dot]])) for t in touches]
    rms = float(np.sqrt(np.mean(np.square(distances))))
    # Uncertainty from the normal matrix, the touches' noise taken as what the fit leaves.
    scale = max(rms / TOUCH_SIGMA_M, 1.0)
    try:
        cov = np.linalg.inv(solution.jac.T @ solution.jac) * scale**2
        variances = np.clip(np.diag(cov), 0.0, None)
    except np.linalg.LinAlgError:
        variances = np.full(len(solution.x), np.inf)
    sigma = {j: float(math.sqrt(variances[i])) for i, j in enumerate(free)}
    tip_sd = np.sqrt(variances[nz : nz + 3])
    return Result({j: float(v) for j, v in zeros.items()}, sigma, tip, tip_sd, points, rms, distances)
