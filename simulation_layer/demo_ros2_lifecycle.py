"""실제 ROS 2 topic으로 frame lifecycle 1회 데모 (rclpy 필요, ROS 2 Humble).

MockSimulatorAdapter + 실제 avva_interfaces ROS 2 message + 실제 rclpy publish/subscribe.
gate.py 로직은 이전 데모들과 동일 — 이번엔 파이썬 콜백이 아니라 진짜 DDS topic을
왕복한다는 점만 다르다. SimulationLayerNode와 mock Core/Ego/Traffic 역할의
별도 rclpy 노드 2개가 같은 프로세스 안에서 실제 topic pub/sub으로 통신한다.
"""
from __future__ import annotations

import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import avva_interfaces.msg as ros_msgs
import avva_phase1 as m
from simulation_layer import ros2_convert as conv
from simulation_layer.ros2_node import RELIABLE_NO_DROP, SimulationLayerNode
from simulation_layer.test_frame_lifecycle import (
    EGO_ID, EPOCH, RUN_CONFIG, RUN_ID, make_mock_adapter, mock_ack,
    mock_advance, mock_ego_command, mock_npc_batch,
)
from simulation_layer.gate import RegistryEntry

SIM_ID = "ros2_demo"
D001 = uuid.uuid5(uuid.NAMESPACE_OID, "D001").bytes
REGISTRY = [
    RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.TRUCK),
]


class MockCoreEgoTraffic(Node):
    """Core/Ego/Traffic 대역 — 전부 진짜 rclpy publisher/subscriber."""

    def __init__(self) -> None:
        super().__init__("mock_core_ego_traffic")
        self.latest: dict[str, object] = {}
        self.received_hop_count = 0
        self.create_subscription(ros_msgs.WorldStateFrame, f"avva/v1/sim/{SIM_ID}/world_state",
                                 self._store("world_state", m.WorldStateFrame), RELIABLE_NO_DROP)
        self.create_subscription(ros_msgs.EgoObservationFrame,
                                 f"avva/v1/sim/{SIM_ID}/ego/{EGO_ID}/observation",
                                 self._store("observation", m.EgoObservationFrame), RELIABLE_NO_DROP)
        self.create_subscription(ros_msgs.CommandSetReady, f"avva/v1/sim/{SIM_ID}/command_ready",
                                 self._store("command_ready", m.CommandSetReady), RELIABLE_NO_DROP)
        self.create_subscription(ros_msgs.FrameComplete, f"avva/v1/sim/{SIM_ID}/frame_complete",
                                 self._store_and_count("frame_complete", m.FrameComplete), RELIABLE_NO_DROP)

        self.ego_pub = self.create_publisher(ros_msgs.EgoControlCommand,
                                             f"avva/v1/sim/{SIM_ID}/ego/{EGO_ID}/control", RELIABLE_NO_DROP)
        self.npc_pub = self.create_publisher(ros_msgs.NpcControlBatch,
                                             f"avva/v1/sim/{SIM_ID}/npc_control", RELIABLE_NO_DROP)
        self.advance_pub = self.create_publisher(ros_msgs.AdvanceFrame,
                                                  f"avva/v1/sim/{SIM_ID}/advance", RELIABLE_NO_DROP)
        self.ack_pub = self.create_publisher(ros_msgs.FrameCompleteAck,
                                             f"avva/v1/sim/{SIM_ID}/frame_complete_ack", RELIABLE_NO_DROP)

    def _store(self, key, dc_cls):
        def cb(ros_msg):
            self.latest[key] = conv.from_ros_msg(ros_msg, dc_cls)
        return cb

    def _store_and_count(self, key, dc_cls):
        base = self._store(key, dc_cls)
        def cb(ros_msg):
            base(ros_msg)
            self.received_hop_count += 1
        return cb


def wait_for(executor, predicate, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def main() -> None:
    rclpy.init()
    backend = make_mock_adapter(REGISTRY)
    sim_node = SimulationLayerNode(backend, RUN_ID, EPOCH, SIM_ID, REGISTRY, RUN_CONFIG)
    mock = MockCoreEgoTraffic()

    executor = SingleThreadedExecutor()
    executor.add_node(sim_node)
    executor.add_node(mock)

    for _ in range(20):  # discovery: let both nodes see each other's topics before publishing
        executor.spin_once(timeout_sec=0.1)

    print("bootstrap: 실제 ROS 2 topic으로 WorldStateFrame(0)/EgoObservationFrame(0) 발행")
    sim_node.bootstrap()
    assert wait_for(executor, lambda: "observation" in mock.latest and "world_state" in mock.latest)
    ws0, obs0 = mock.latest["world_state"], mock.latest["observation"]
    print(f"mock Core/Ego/Traffic가 ROS 2로 수신: state_tick_id={ws0.state_tick_id}")

    print("\nEgoControlCommand + NpcControlBatch를 실제 ROS 2 topic으로 발행")
    mock.ego_pub.publish(conv.to_ros_msg(mock_ego_command(obs0), ros_msgs))
    mock.npc_pub.publish(conv.to_ros_msg(mock_npc_batch(ws0), ros_msgs))
    assert wait_for(executor, lambda: "command_ready" in mock.latest)
    ready = mock.latest["command_ready"]
    print(f"ROS 2로 수신한 CommandSetReady: target_tick_id={ready.target_tick_id}, "
         f"completeness={ready.completeness.name}, validity={ready.validity.name}")
    assert ready.target_tick_id == 1 and ready.validity == m.Validity.VALID

    print("\nAdvanceFrame(decision_id=D001)을 실제 ROS 2 topic으로 발행")
    native_before = int(backend.native_frame_id())
    mock.advance_pub.publish(conv.to_ros_msg(mock_advance(ready, D001), ros_msgs))
    assert wait_for(executor, lambda: mock.received_hop_count == 1)
    native_after = int(backend.native_frame_id())
    fc1 = mock.latest["frame_complete"]
    print(f"ROS 2로 수신한 FrameComplete: state_tick_id={fc1.state_tick_id}, "
         f"decision_id 일치={fc1.decision_id == D001}, native step {native_before}->{native_after}")
    assert native_after - native_before == 1 and fc1.decision_id == D001

    print("\n동일 AdvanceFrame(decision_id=D001) 재전송 (실제 ROS 2 topic)")
    mock.advance_pub.publish(conv.to_ros_msg(mock_advance(ready, D001), ros_msgs))
    assert wait_for(executor, lambda: mock.received_hop_count == 2)
    native_retry = int(backend.native_frame_id())
    print(f"native step: {native_after} -> {native_retry} "
         f"({'재실행 안 됨' if native_retry == native_after else '오류: 재실행됨'})")
    assert native_retry == native_after

    mock.ack_pub.publish(conv.to_ros_msg(mock_ack(fc1), ros_msgs))
    assert wait_for(executor, lambda: sim_node.layer.state == "WAITING_INPUTS")
    print(f"\nFrameCompleteAck(ACCEPTED)까지 실제 ROS 2 왕복 완료. sim_node.layer.state = "
         f"{sim_node.layer.state!r}, state_tick = {sim_node.layer.state_tick}")

    print("\nPASS — 진짜 rclpy publisher/subscriber + DDS topic으로 frame lifecycle 1회 통과")
    sim_node.destroy_node()
    mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
