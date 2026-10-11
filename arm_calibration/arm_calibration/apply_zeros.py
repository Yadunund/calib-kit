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

"""Fold joint zero offsets (`calibrate_zeros`'s joint_zeros.yaml) into a URDF.

apply_zeros --urdf arm.urdf --zeros joint_zeros.yaml --out arm_calibrated.urdf
"""

from __future__ import annotations

import argparse
import pathlib

import yaml

from arm_calibration.urdf_chain import apply_zeros


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", type=pathlib.Path, required=True)
    parser.add_argument("--zeros", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args(argv)
    zeros = yaml.safe_load(args.zeros.read_text())["joint_zeros_rad"]
    args.out.write_text(apply_zeros(args.urdf.read_text(), {j: float(v) for j, v in zeros.items()}))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
