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

"""The ROS contract (README.md), against a real controller manager on mock hardware: the description
and joint states arrive, the arm's ros2_control hardware is found from its joints, released with the
controllers that command it, keeps publishing joint states while released, and is restored."""

import os
import random
import shutil
import subprocess
import time

import pytest

# Kept to this machine, on a domain of its own, so it cannot meet a robot that is running. Set before
# rclpy loads, which reads them once.
os.environ.update(
    RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
    ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
    ROS_DOMAIN_ID=str(random.randint(100, 200)),
)
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("controller_manager_msgs")
if shutil.which("ros2") is None:
    pytest.skip("no ros2 command: source a ROS 2 install", allow_module_level=True)

from test_arm_calibration import FIVE

JOINTS = ["pan", "lift", "elbow", "pitch", "roll"]
CONTROL = (
    '<ros2_control name="MockArm" type="system"><hardware><plugin>mock_components/GenericSystem</plugin>'
    "</hardware>"
    + "".join(
        f'<joint name="{j}"><command_interface name="position"/>'
        f'<state_interface name="position"><param name="initial_value">0.1</param></state_interface>'
        '<state_interface name="velocity"/></joint>'
        for j in JOINTS
    )
    + "</ros2_control>"
)
CONTROLLERS = f"""controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
arm_controller:
  ros__parameters:
    joints: {JOINTS}
    command_interfaces: [position]
    state_interfaces: [position, velocity]
"""


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("stack")
    urdf = FIVE.replace("</robot>", CONTROL + "</robot>")
    (tmp / "arm.urdf").write_text(urdf)
    (tmp / "controllers.yaml").write_text(CONTROLLERS)
    env = dict(os.environ)
    processes = [
        subprocess.Popen(
            [
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "-p",
                f"robot_description:={urdf}",
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ),
        subprocess.Popen(
            [
                "ros2",
                "run",
                "controller_manager",
                "ros2_control_node",
                "--ros-args",
                "--params-file",
                str(tmp / "controllers.yaml"),
                "-r",
                "~/robot_description:=/robot_description",
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ),
    ]
    try:
        subprocess.run(
            [
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                "joint_state_broadcaster",
                "arm_controller",
                "--param-file",
                str(tmp / "controllers.yaml"),
                "--controller-manager-timeout",
                "60",
            ],
            env=env,
            check=True,
            timeout=90,
            capture_output=True,
        )
        from arm_calibration.ros_arm import RosArm

        arm = RosArm()
        yield arm, urdf
        arm.close()
    finally:
        for p in processes:
            p.terminate()
        for p in processes:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()


def controller_states(arm):
    from controller_manager_msgs.srv import ListControllers

    return {c.name: c.state for c in arm._call(arm._list_ctrl, ListControllers.Request()).controller}


def test_description_and_joint_states_arrive(stack):
    """ARM-IF-1, ARM-IF-2"""
    arm, _urdf = stack
    assert "<robot" in arm.description()
    assert arm.positions(JOINTS) == pytest.approx(dict.fromkeys(JOINTS, 0.1))


def test_the_hardware_is_found_from_the_chains_joints(stack):
    """ARM-IF-3, ARM-REL-1"""
    arm, _ = stack
    assert arm.hardware_for(JOINTS).name == "MockArm"
    assert arm.hardware_for(JOINTS, "MockArm").name == "MockArm"


def test_release_and_restore(stack):
    """ARM-IF-3, ARM-REL-2, ARM-REL-3, ARM-REL-4"""
    from lifecycle_msgs.msg import State

    arm, _ = stack
    hardware = arm.hardware_for(JOINTS)
    released = arm.release(hardware)
    try:
        assert released == ["arm_controller"]
        assert arm.hardware_for(JOINTS).state.id == State.PRIMARY_STATE_INACTIVE
        states = controller_states(arm)
        assert states["arm_controller"] == "inactive"
        assert states["joint_state_broadcaster"] == "active"
        assert arm.updating()
    finally:
        arm.restore()
    assert arm.hardware_for(JOINTS).state.id == State.PRIMARY_STATE_ACTIVE
    assert controller_states(arm)["arm_controller"] == "active"
    assert arm.released is None
    time.sleep(0.2)
    assert arm.positions(JOINTS) == pytest.approx(dict.fromkeys(JOINTS, 0.1), abs=1e-6)


def test_the_tool_restores_the_arm_however_it_ends(stack, tmp_path, monkeypatch):
    """ARM-REL-4, ARM-REC-4, ARM-NO-3"""
    from lifecycle_msgs.msg import State

    from arm_calibration import calibrate_arm

    arm, _ = stack
    (tmp_path / "arm.yaml").write_text("name: mock\nbase_link: base\ntip_link: tip\nfree_joints: [lift]\n")
    answers = iter(["", "q"])  # hold the arm; then quit before any touch
    monkeypatch.setattr("builtins.input", lambda _prompt="": next(answers))
    with pytest.raises(SystemExit, match="no touches recorded"):
        calibrate_arm.main(["--profile", str(tmp_path / "arm.yaml"), "--out", str(tmp_path)])
    assert arm.hardware_for(JOINTS).state.id == State.PRIMARY_STATE_ACTIVE
    assert controller_states(arm)["arm_controller"] == "active"
    assert (tmp_path / "description.urdf").read_text().startswith("<")
