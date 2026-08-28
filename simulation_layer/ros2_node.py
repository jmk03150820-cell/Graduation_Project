"""Simulation Layer를 ROS 2 topic에 올리는 rclpy 노드.

gate.py(SimulationLayer)의 상태머신·검증·hash 로직은 그대로 두고, 이 노드는
channel_registry.yaml의 논리 채널 <-> ROS 2 topic/QoS 매핑과
ros2_convert의 dataclass<->ROS msg 변환만 담당한다 (전제조건/큰 프레임 불변).

`import rclpy`/`import avva_interfaces`는 이 파일에만 있다 — carla_backend.py의
`import carla` 격리와 같은 패턴.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

import avva_interfaces.msg as ros_msgs
import avva_phase1 as m

from . import ros2_convert as conv
from .backend import RunConfig
from .gate import RegistryEntry, SimulationLayer

# channel_registry.yaml: RELIABLE_NO_DROP = Reliable + Volatile + KeepLast(depth)
# — Simulation Layer의 8개 채널 전부 이 delivery class (§5.4).
RELIABLE_NO_DROP = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=4)


class SimulationLayerNode(Node):
    def __init__(self, backend, run_id: bytes, run_epoch: int, sim_id: str,
                 registry: list[RegistryEntry], run_config: RunConfig) -> None:
        super().__init__(f"simulation_layer_{sim_id}")
        self.layer = SimulationLayer(backend, run_id, run_epoch, sim_id, registry,
                                     run_config, publish=self._publish)
        ego_ids = sorted(e.actor_id for e in registry if e.role == m.ActorRole.ACTOR_ROLE_EGO)

        self._channel_publishers = {
            f"avva/v1/sim/{sim_id}/world_state": self.create_publisher(
                ros_msgs.WorldStateFrame, f"avva/v1/sim/{sim_id}/world_state", RELIABLE_NO_DROP),
            f"avva/v1/sim/{sim_id}/command_ready": self.create_publisher(
                ros_msgs.CommandSetReady, f"avva/v1/sim/{sim_id}/command_ready", RELIABLE_NO_DROP),
            f"avva/v1/sim/{sim_id}/frame_complete": self.create_publisher(
                ros_msgs.FrameComplete, f"avva/v1/sim/{sim_id}/frame_complete", RELIABLE_NO_DROP),
        }
        for ego_id in ego_ids:
            topic = f"avva/v1/sim/{sim_id}/ego/{ego_id}/observation"
            self._channel_publishers[topic] = self.create_publisher(
                ros_msgs.EgoObservationFrame, topic, RELIABLE_NO_DROP)

        self.create_subscription(
            ros_msgs.NpcControlBatch, f"avva/v1/sim/{sim_id}/npc_control",
            self._on_npc_batch, RELIABLE_NO_DROP)
        self.create_subscription(
            ros_msgs.AdvanceFrame, f"avva/v1/sim/{sim_id}/advance",
            self._on_advance_frame, RELIABLE_NO_DROP)
        self.create_subscription(
            ros_msgs.FrameCompleteAck, f"avva/v1/sim/{sim_id}/frame_complete_ack",
            self._on_frame_complete_ack, RELIABLE_NO_DROP)
        for ego_id in ego_ids:
            self.create_subscription(
                ros_msgs.EgoControlCommand, f"avva/v1/sim/{sim_id}/ego/{ego_id}/control",
                self._on_ego_control, RELIABLE_NO_DROP)

    def bootstrap(self) -> None:
        self.layer.bootstrap()

    # ---- outbound: dataclass -> ROS msg -> publish -----------------------
    def _publish(self, channel: str, dc_msg) -> None:
        pub = self._channel_publishers.get(channel)
        if pub is None:
            raise KeyError(f"no publisher registered for channel {channel!r}")
        pub.publish(conv.to_ros_msg(dc_msg, ros_msgs))

    # ---- inbound: ROS msg -> dataclass -> gate ----------------------------
    def _on_ego_control(self, ros_msg) -> None:
        self.layer.on_ego_control(conv.from_ros_msg(ros_msg, m.EgoControlCommand))

    def _on_npc_batch(self, ros_msg) -> None:
        self.layer.on_npc_batch(conv.from_ros_msg(ros_msg, m.NpcControlBatch))

    def _on_advance_frame(self, ros_msg) -> None:
        self.layer.on_advance_frame(conv.from_ros_msg(ros_msg, m.AdvanceFrame))

    def _on_frame_complete_ack(self, ros_msg) -> None:
        self.layer.on_frame_complete_ack(conv.from_ros_msg(ros_msg, m.FrameCompleteAck))


def main() -> None:  # pragma: no cover — manual/launch entry point
    import sys
    import uuid

    rclpy.init(args=sys.argv)
    registry = [
        RegistryEntry("ego_0", m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    ]
    from .mock_backend import MockSimulatorAdapter
    run_config = RunConfig(target_rate_hz=20.0)
    backend = MockSimulatorAdapter()
    backend.initialize()
    backend.configure(run_config)
    for e in registry:
        backend.spawn_actor(e.actor_id)
    node = SimulationLayerNode(backend, uuid.uuid4().bytes, 1, "mock_0", registry, run_config)
    node.bootstrap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
