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

A profile names the chain's `base_link` and `tip_link` (the frame nearest the point that touches
the dots), what touches the dots (`tip`), where it is in that frame (`tip_offset`, metres) and
which way the tool points (`tool_axis`), the
`free_joints` whose zeros touches can find (`fit`) and why the others are held (`held_joints`), the
table's normal in the base frame (`plane_normal`), how many dots to touch (`dots`) and how far to
lean the tool (`tilt_deg`). Nothing in it is about the arm's hardware: that is ros2_control's.

Profiles shipped with the package are in `arms/` and are named by their file's stem (``so101``);
any other is given by its path.
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass, field

import yaml

ARMS = pathlib.Path(__file__).parent / "arms"
DEFAULT = "so101"


@dataclass
class Profile:
    name: str
    base_link: str
    tip_link: str
    free_joints: list[str]
    description: str = ""
    tip: str = "the tip"
    tool_axis: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    tip_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    held_joints: dict[str, str] = field(default_factory=dict)
    plane_normal: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    dots: int = 3
    tilt_deg: float = 40.0
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
    return profile
