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

from glob import glob

from setuptools import find_packages, setup

package_name = "arm_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["arms/*.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/arms", glob("arm_calibration/arms/*.yaml")),
    ],
    install_requires=["numpy", "scipy", "pyyaml"],
    zip_safe=False,
    maintainer="Yadunund Vijay",
    maintainer_email="yadunund@gmail.com",
    description="Joint zero calibration for robot arms",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calibrate_arm = arm_calibration.calibrate_arm:main",
            "apply_calibration = arm_calibration.apply_calibration:main",
        ],
    },
)
