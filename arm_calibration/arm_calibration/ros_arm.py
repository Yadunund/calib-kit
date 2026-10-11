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

"""The arm, as a ros2_control stack presents it: its description, its joints, and its torque.

Everything the calibration needs from a robot comes through the interfaces any ros2_control arm
already has (README.md, "Contract"): the description on a ``std_msgs/msg/String`` topic, the
joints' positions on a ``sensor_msgs/msg/JointState`` topic, and the controller manager's services
to release the arm. Releasing deactivates the controllers that claim the ros2_control hardware's
command interfaces, then takes the hardware to inactive, which the contract requires to cut
torque while its state interfaces keep updating. `restore` undoes exactly that.
"""

from __future__ import annotations

import threading
import time

import rclpy
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ContractError(RuntimeError):
    """The robot does not offer what the calibration needs (README.md, "Contract")."""


class RosArm:
    def __init__(
        self,
        joint_states: str = "/joint_states",
        robot_description: str = "/robot_description",
        controller_manager: str = "/controller_manager",
        node_name: str = "arm_calibration",
    ):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._stamps: list[float] = []
        self._description: str | None = None
        self.node.create_subscription(JointState, joint_states, self._on_joint_states, 10)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )
        self.node.create_subscription(String, robot_description, self._on_description, latched)
        cm = controller_manager.rstrip("/")
        self._list_hw = self.node.create_client(ListHardwareComponents, f"{cm}/list_hardware_components")
        self._list_ctrl = self.node.create_client(ListControllers, f"{cm}/list_controllers")
        self._switch = self.node.create_client(SwitchController, f"{cm}/switch_controller")
        self._set_hw = self.node.create_client(SetHardwareComponentState, f"{cm}/set_hardware_component_state")
        self.topics = (joint_states, robot_description, cm)
        self.released: tuple[str, list[str]] | None = None
        self._spin.start()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def _on_joint_states(self, msg: JointState):
        with self._lock:
            self._positions.update(zip(msg.name, msg.position, strict=False))
            self._stamps = (self._stamps + [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9])[-50:]

    def _on_description(self, msg: String):
        self._description = msg.data

    def description(self, timeout_s: float = 5.0) -> str:
        end = time.monotonic() + timeout_s
        while self._description is None and time.monotonic() < end:
            time.sleep(0.05)
        if self._description is None:
            raise ContractError(
                f"no robot description on {self.topics[1]} within {timeout_s:.0f} s "
                "(it is published transient-local, as robot_state_publisher does), or pass --urdf"
            )
        return self._description

    def positions(self, joints: list[str], timeout_s: float = 5.0) -> dict[str, float]:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            with self._lock:
                if all(j in self._positions for j in joints):
                    return {j: self._positions[j] for j in joints}
            time.sleep(0.05)
        with self._lock:
            missing = [j for j in joints if j not in self._positions]
        raise ContractError(f"no position for {', '.join(missing)} on {self.topics[0]} within {timeout_s:.0f} s")

    def still_positions(
        self, joints: list[str], samples: int = 15, period_s: float = 0.03, max_spread_rad: float = 0.005
    ) -> dict[str, float] | None:
        """Each joint's position averaged over `samples` looks, or None if any moved more than
        `max_spread_rad` meanwhile (the arm was not held still)."""
        reads = []
        for _ in range(samples):
            reads.append(self.positions(joints))
            time.sleep(period_s)
        out = {}
        for j in joints:
            values = [r[j] for r in reads]
            if max(values) - min(values) > max_spread_rad:
                return None
            out[j] = sum(values) / len(values)
        return out

    def updating(self, within_s: float = 2.0) -> bool:
        """Whether joint states with a newer stamp than now's latest arrive within `within_s`."""
        with self._lock:
            last = self._stamps[-1] if self._stamps else float("-inf")
        end = time.monotonic() + within_s
        while time.monotonic() < end:
            with self._lock:
                if self._stamps and self._stamps[-1] > last:
                    return True
            time.sleep(0.05)
        return False

    def _call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise ContractError(f"no service {client.srv_name}: is the controller manager at {self.topics[2]}?")
        future = client.call_async(request)
        end = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        if not future.done():
            raise ContractError(f"{client.srv_name} did not answer within {timeout_s:.0f} s")
        return future.result()

    def hardware_for(self, joints: list[str], name: str | None = None):
        """The ros2_control hardware (the controller manager's hardware component, named by its
        <ros2_control> block) whose state interfaces hold every one of `joints`' positions, or the
        one called `name`."""
        components = self._call(self._list_hw, ListHardwareComponents.Request()).component
        if name is not None:
            found = [c for c in components if c.name == name]
            if not found:
                raise ContractError(
                    f"no hardware component {name!r}: there are {', '.join(c.name for c in components)}"
                )
            return found[0]
        wanted = {f"{j}/position" for j in joints}
        found = [c for c in components if wanted <= {i.name for i in c.state_interfaces}]
        if len(found) != 1:
            raise ContractError(
                f"{len(found)} ros2_control hardware hold the positions of {', '.join(joints)}: name one with --ros2-control"
            )
        return found[0]

    def release(self, hardware) -> list[str]:
        """Deactivate the active controllers claiming `hardware`'s command interfaces, then take
        it to inactive. Returns the controllers deactivated, for `restore`."""
        if hardware.state.id != State.PRIMARY_STATE_ACTIVE:
            raise ContractError(f"{hardware.name} is {hardware.state.label}, not active: nothing to release")
        commands = {i.name for i in hardware.command_interfaces}
        controllers = [
            c.name
            for c in self._call(self._list_ctrl, ListControllers.Request()).controller
            if c.state == "active" and commands & set(c.claimed_interfaces)
        ]
        self.released = (hardware.name, controllers)
        if controllers:
            self._switch_controllers(deactivate=controllers)
        self._set_state(hardware.name, State.PRIMARY_STATE_INACTIVE)
        return controllers

    def restore(self):
        """Undo `release`: the hardware active again, then the controllers it had."""
        if self.released is None:
            return
        name, controllers = self.released
        self._set_state(name, State.PRIMARY_STATE_ACTIVE)
        if controllers:
            self._switch_controllers(activate=controllers)
        self.released = None

    def _switch_controllers(self, activate=(), deactivate=()):
        request = SwitchController.Request(
            activate_controllers=list(activate),
            deactivate_controllers=list(deactivate),
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(seconds=5).to_msg(),
        )
        response = self._call(self._switch, request)
        if not response.ok:
            raise ContractError(
                f"switch_controller refused (activate {list(activate)}, deactivate {list(deactivate)}): "
                f"{response.message}"
            )

    def _set_state(self, name: str, state_id: int):
        request = SetHardwareComponentState.Request(name=name, target_state=State(id=state_id))
        response = self._call(self._set_hw, request)
        if not response.ok:
            raise ContractError(f"{name} could not be taken to state {state_id}")
