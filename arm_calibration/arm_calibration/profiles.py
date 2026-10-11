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

"""Arm profiles: what the calibration needs to know about one kind of arm, in a YAML file.

A profile names:

- the chain: `base_link`, and `tip_link`, the frame nearest the point that touches the references;
- the tip: what touches the references (`tip`), where it is expected in `tip_link` (`tip_offset`,
  metres) and which of `tip_link`'s axes points out of the tool (`tool_axis`);
- what to calibrate by default (`solve`: `zeros`, `tcp` or `zeros+tcp`) and against what
  (`reference`: `dots` on the table or a sharp `pin`);
- the `free_joints` whose zeros touches can find, and why the others are held (`held_joints`);
- the link a measured tool centre point is added as (`tcp_link`, fixed to `tip_link`);
- the table's normal in the base frame (`plane_normal`) and how many dots to touch (`dots`);
- the shapes to touch each reference in (`shapes`): `down`, or a lean `towards` the base, `away`
  from it, to the `left` or the `right`, by `tilt_deg` or by the degrees given (`away 20`). An arm
  whose wrist cannot lean sideways lists only the leans it can make.

Nothing in it is about the arm's hardware: that is ros2_control's.

Profiles shipped with the package are in `arms/` and are named by their file's stem (``so101``);
any other is given by its path.
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass, field

import yaml

ARMS = pathlib.Path(__file__).parent / "arms"
DEFAULT = "so101"
DIRECTIONS = ("down", "towards", "away", "left", "right")
SOLVES = ("zeros", "tcp", "zeros+tcp")


@dataclass
class Profile:
    name: str
    base_link: str
    tip_link: str
    free_joints: list[str] = field(default_factory=list)
    description: str = ""
    solve: str = "zeros"
    reference: str = "dots"
    tcp_link: str = "tcp_link"
    tip: str = "the tip"
    tool_axis: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    tip_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    held_joints: dict[str, str] = field(default_factory=dict)
    plane_normal: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    dots: int = 3
    tilt_deg: float = 40.0
    shapes: list[str] = field(default_factory=lambda: list(DIRECTIONS))
    path: pathlib.Path | None = None


def available() -> list[str]:
    return sorted(p.stem for p in ARMS.glob("*.yaml"))


def load(name_or_path: str) -> Profile:
    """The profile `name_or_path`: a shipped arm's name, or a YAML file's path."""
    path = pathlib.Path(name_or_path).expanduser()
    if not path.suffix:
        path = ARMS / f"{name_or_path}.yaml"
        if not path.exists():
            raise ValueError(f"no arm {name_or_path!r}: one of {', '.join(available())}, or a profile's path")
    data = yaml.safe_load(path.read_text())
    known = set(Profile.__dataclass_fields__) - {"path"}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"{path}: unknown settings {', '.join(sorted(unknown))}")
    profile = Profile(**data, path=path)
    if profile.solve not in SOLVES:
        raise ValueError(f"{path}: solve is one of {', '.join(SOLVES)}")
    for shape in profile.shapes:
        parse_shape(shape, profile.tilt_deg, path)
    return profile


def parse_shape(shape: str, tilt_deg: float, where="") -> tuple[str, float]:
    """A shape (`away`, `away 20`, `down`) as its direction and its lean, degrees."""
    words = str(shape).split()
    if not words or words[0] not in DIRECTIONS or len(words) > 2:
        raise ValueError(f"{where}: shape {shape!r} is a direction ({', '.join(DIRECTIONS)}) and an optional lean")
    if words[0] == "down":
        return "down", 0.0
    return words[0], float(words[1]) if len(words) == 2 else float(tilt_deg)
