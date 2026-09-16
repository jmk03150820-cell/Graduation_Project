"""Simulation Layer를 ROS 2 topic에 올리는 rclpy 노드.

gate.py(SimulationLayer)의 상태머신·검증·hash 로직은 그대로 두고, 이 노드는
channel_registry.yaml의 논리 채널 <-> ROS 2 topic/QoS 매핑과
ros2_convert의 dataclass<->ROS msg 변환만 담당한다 (전제조건/큰 프레임 불변).

`import rclpy`/`import avva_interfaces`는 이 파일에만 있다 — carla_backend.py의
`import carla` 격리와 같은 패턴.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

import avva_interfaces.msg as ros_msgs
import avva_phase1 as m

from . import hashing
from . import ros2_convert as conv
from .backend import RunConfig
from .component import SimBackendComponent
from .gate import RegistryEntry, SimulationLayer

# channel_registry.yaml: RELIABLE_NO_DROP = Reliable + Volatile + KeepLast(depth)
# — Simulation Layer의 8개 채널 전부 이 delivery class (§5.4).
RELIABLE_NO_DROP = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=4)


def _reliable_no_drop(depth: int) -> QoSProfile:
    """channel_registry.yaml의 채널별 depth를 그대로 반영한 RELIABLE_NO_DROP 변형."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST, depth=depth)


# channel_registry.yaml kind 1~4(control channel) QoS — RunManifest만 RELIABLE_LATCHED
# (늦게 붙는 구독자도 마지막 값을 받아야 함), 나머지는 RELIABLE_NO_DROP + yaml이 정한 depth.
RELIABLE_LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_RUN_CONTROL_QOS = _reliable_no_drop(16)
_COMPONENT_READY_QOS = _reliable_no_drop(32)
_CONTROL_ACK_QOS = _reliable_no_drop(32)
_REJECT_NOTICE_QOS = _reliable_no_drop(32)

# channel_registry.yaml kind 30(ComponentHeartbeat): RELIABLE_LATEST = reliable+volatile+
# keep_last(depth 1) — 최신 상태 하나만 의미 있고 과거 하트비트는 쌓아둘 필요 없음.
_RELIABLE_LATEST = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
_HEARTBEAT_PERIOD_S = 1.0


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
            # gate.py가 REJECT/ABORT 시 RejectNotice를 실제로 발행하게 됐으므로
            # (5번 공지 2번: 출력 경계) 이 publisher가 없으면 첫 REJECT에서
            # _publish()가 KeyError로 죽는다 — channel_registry.yaml: 전용 채널,
            # sim_id 접두사 없음(evidence/reject는 컴포넌트 공통 채널).
            "avva/v1/evidence/reject": self.create_publisher(
                ros_msgs.RejectNotice, "avva/v1/evidence/reject", RELIABLE_NO_DROP),
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


class SimBackendRosNode(Node):
    """RunManifest/RunControlCommand(control channel, kind 1~4)부터 시작하는 실제
    운용 진입점.

    SimulationLayerNode(데모/수동 실행용, backend가 이미 준비된 채로 생성)와 달리
    이 노드는 control channel만 갖고 시작한다 — 어떤 backend를 쓸지·registry가
    뭔지는 RunManifest가 와야 정해지므로, startup.py의 공식 확장 지점인
    ``run_transport_factory``(startup.py 121~124줄 docstring: "실제 전송(ROS2 등)을
    쓰려면 같은 시그니처의 factory를 만들어 주입한다")를 그대로 쓴다.
    gate.py/component.py 로직은 전혀 안 바꾼다 — 이 노드는 채널<->토픽 매핑만 담당.
    """

    def __init__(self, sim_id: str) -> None:
        super().__init__(f"sim_backend_{sim_id}")
        self.sim_id = sim_id
        # frame data path(kind 10~23) publisher/subscription은 첫 READY manifest
        # 때 한 번만 만들고 run 교체(같은 sim_id) 시 재사용한다 — ponytail: registry의
        # ego 구성이 run마다 안 바뀐다고 가정(Phase 1 single-ego 제약과 일치). 여러
        # ego/동적 registry가 필요해지면 diff 기반 재생성으로 바꿔야 함.
        self._frame_publishers: dict[str, object] | None = None
        self.component = SimBackendComponent(sim_id, run_transport_factory=self._make_run_transport)

        self.create_subscription(
            ros_msgs.RunManifest, "avva/v1/run/manifest", self._on_run_manifest, RELIABLE_LATCHED)
        self.create_subscription(
            ros_msgs.RunControlCommand, "avva/v1/run/control", self._on_run_control, _RUN_CONTROL_QOS)
        self._ready_pub = self.create_publisher(
            ros_msgs.ComponentReady, "avva/v1/run/component_ready", _COMPONENT_READY_QOS)
        self._ack_pub = self.create_publisher(
            ros_msgs.RunControlAck, "avva/v1/run/control_ack", _CONTROL_ACK_QOS)

        # §10.6: 최소 하트비트 발행 — RTT/backpressure 등 정량 지표는 다음 단계.
        self._heartbeat_pub = self.create_publisher(
            ros_msgs.ComponentHeartbeat, "avva/v1/health/heartbeat", _RELIABLE_LATEST)
        self.create_timer(_HEARTBEAT_PERIOD_S, self._publish_heartbeat)

    def _publish_heartbeat(self) -> None:
        """ComponentHeartbeat(kind 30)을 주기 발행한다 — 존재 확인 수준(§10.6 B안).
        RunManifest 전이면 STARTING, layer는 있지만 아직 START 전이면
        COMPONENT_READY_STATE, RUNNING이면 COMPONENT_RUNNING으로만 구분한다."""
        layer = self.component.layer
        if self.component.run_state == m.RunState.RUNNING:
            state = m.ComponentState.COMPONENT_RUNNING
        elif layer is not None:
            state = m.ComponentState.COMPONENT_READY_STATE
        else:
            state = m.ComponentState.STARTING
        last_state_tick = (m.OptionalUint64(True, layer.state_tick) if layer is not None
                           else m.OptionalUint64(False, 0))
        hb = m.ComponentHeartbeat(
            header=None, component_state=state, health_status=m.HealthStatus.HEALTH_OK,
            last_state_tick_id=last_state_tick,
            # ponytail: target tick·native session은 존재 확인 단계엔 안 채움 — RTT/정량
            # 모니터링 도입 시(§10.6 A안) SimulationLayer에 노출 지점 추가해서 채울 것.
            last_target_tick_id=m.OptionalUint64(False, 0),
            native_session_id=m.OptionalUuid128(False, b"\x00" * 16),
            monotonic_time_ns=time.monotonic_ns())
        hb.header = self.component.build_header(hashing.component_heartbeat_hash(hb))
        self._heartbeat_pub.publish(conv.to_ros_msg(hb, ros_msgs))

    def _on_run_manifest(self, ros_msg) -> None:
        result = self.component.on_run_manifest(conv.from_ros_msg(ros_msg, m.RunManifest))
        self._ready_pub.publish(conv.to_ros_msg(result.ready, ros_msgs))

    def _on_run_control(self, ros_msg) -> None:
        ack = self.component.on_run_control(conv.from_ros_msg(ros_msg, m.RunControlCommand))
        self._ack_pub.publish(conv.to_ros_msg(ack, ros_msgs))

    def _make_run_transport(self, backend, run_id, run_epoch, sim_id, registry, run_config):
        """RunTransportFactory 구현체. SimulationLayer.publish를 실제 ROS 발행에 잇고,
        frame data path 구독을 component의 대응 메서드로 연결한다."""
        ego_ids = sorted(e.actor_id for e in registry if e.role == m.ActorRole.ACTOR_ROLE_EGO)
        if self._frame_publishers is None:
            publishers = {
                f"avva/v1/sim/{sim_id}/world_state": self.create_publisher(
                    ros_msgs.WorldStateFrame, f"avva/v1/sim/{sim_id}/world_state", RELIABLE_NO_DROP),
                f"avva/v1/sim/{sim_id}/command_ready": self.create_publisher(
                    ros_msgs.CommandSetReady, f"avva/v1/sim/{sim_id}/command_ready", RELIABLE_NO_DROP),
                f"avva/v1/sim/{sim_id}/frame_complete": self.create_publisher(
                    ros_msgs.FrameComplete, f"avva/v1/sim/{sim_id}/frame_complete", RELIABLE_NO_DROP),
                # channel_registry.yaml: evidence/reject는 depth 32 — Sim 파트 대조 §9에서
                # 확인된 기존 SimulationLayerNode의 depth 불일치(공용 depth=4 재사용)를
                # 여기서는 반복하지 않는다.
                "avva/v1/evidence/reject": self.create_publisher(
                    ros_msgs.RejectNotice, "avva/v1/evidence/reject", _REJECT_NOTICE_QOS),
            }
            for ego_id in ego_ids:
                topic = f"avva/v1/sim/{sim_id}/ego/{ego_id}/observation"
                publishers[topic] = self.create_publisher(ros_msgs.EgoObservationFrame, topic, RELIABLE_NO_DROP)

            self.create_subscription(
                ros_msgs.NpcControlBatch, f"avva/v1/sim/{sim_id}/npc_control",
                lambda ros_msg: self.component.on_npc_control_batch(
                    conv.from_ros_msg(ros_msg, m.NpcControlBatch)),
                RELIABLE_NO_DROP)
            self.create_subscription(
                ros_msgs.AdvanceFrame, f"avva/v1/sim/{sim_id}/advance",
                lambda ros_msg: self.component.on_advance_frame(
                    conv.from_ros_msg(ros_msg, m.AdvanceFrame)),
                RELIABLE_NO_DROP)
            self.create_subscription(
                ros_msgs.FrameCompleteAck, f"avva/v1/sim/{sim_id}/frame_complete_ack",
                lambda ros_msg: self.component.on_frame_complete_ack(
                    conv.from_ros_msg(ros_msg, m.FrameCompleteAck)),
                RELIABLE_NO_DROP)
            for ego_id in ego_ids:
                topic = f"avva/v1/sim/{sim_id}/ego/{ego_id}/control"
                self.create_subscription(
                    ros_msgs.EgoControlCommand, topic,
                    lambda ros_msg: self.component.on_ego_control(
                        conv.from_ros_msg(ros_msg, m.EgoControlCommand)),
                    RELIABLE_NO_DROP)
            self._frame_publishers = publishers

        def publish(channel: str, dc_msg) -> None:
            pub = self._frame_publishers.get(channel)
            if pub is None:
                raise KeyError(f"no publisher registered for channel {channel!r}")
            pub.publish(conv.to_ros_msg(dc_msg, ros_msgs))

        layer = SimulationLayer(backend, run_id, run_epoch, sim_id, registry, run_config, publish=publish)
        return layer, self._frame_publishers

    def shutdown(self) -> None:
        """프로세스 종료 시 호출 — component.shutdown()으로 활성 backend까지 정리한
        뒤 ROS node를 파괴한다. destroy_node()만 직접 부르면 backend(예: CARLA
        connection)가 안 닫혀서 리소스가 샌다."""
        self.component.shutdown()
        self.destroy_node()


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
