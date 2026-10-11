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

"""Fold a calibration (`calibrate_arm`'s joint_zeros.yaml and tcp.yaml) into a URDF.

apply_calibration --urdf arm.urdf --zeros joint_zeros.yaml --tcp tcp.yaml --out arm_calibrated.urdf
"""

from __future__ import annotations

import argparse
import pathlib

import yaml

from arm_calibration.urdf_chain import add_tcp, apply_zeros


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", type=pathlib.Path, required=True)
    parser.add_argument("--zeros", type=pathlib.Path, help="joint_zeros.yaml")
    parser.add_argument("--tcp", type=pathlib.Path, help="tcp.yaml")
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args(argv)
    if not args.zeros and not args.tcp:
        parser.error("give --zeros, --tcp or both")
    urdf = args.urdf.read_text()
    if args.zeros:
        zeros = yaml.safe_load(args.zeros.read_text())["joint_zeros_rad"]
        urdf = apply_zeros(urdf, {j: float(v) for j, v in zeros.items()})
    if args.tcp:
        tcp = yaml.safe_load(args.tcp.read_text())
        urdf = add_tcp(urdf, tcp["parent"], tcp["link"], tcp["xyz_m"])
    args.out.write_text(urdf)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
