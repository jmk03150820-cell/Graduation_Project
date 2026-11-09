"""Simulation Layer를 ROS 2 topic에 올리는 rclpy 노드.

gate.py(SimulationLayer)의 상태머신·검증·hash 로직은 그대로 두고, 이 노드는
channel_registry.yaml의 논리 채널 <-> ROS 2 topic/QoS 매핑과
ros2_convert의 dataclass<->ROS msg 변환만 담당한다 (전제조건/큰 프레임 불변).

`import rclpy`/`import avva_interfaces`는 이 파일에만 있다 — carla_backend.py의
`import carla` 격리와 같은 패턴.
"""
from __future__ import annotations

import time
from queue import Queue
from threading import Lock, Thread
from typing import Callable

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


class _BackendWorker:
    """capacity-1 단일 backend worker thread (§ 추가 항목 2+3 통합).

    SingleThreadedExecutor 하나뿐인 ROS 콜백 스레드가 CARLA/SUMO native 호출로
    막히면 heartbeat 타이머를 포함한 spin 루프 전체가 멈춘다 — 실제 backend를
    ROS 노드에 물리기 전에 반드시 필요한 분리. ROS 콜백은 여기 submit만 하고
    바로 반환하고, native 호출은 이 전용 스레드에서만 일어난다.

    rclpy Publisher.publish()는 스레드 세이프하므로(rmw가 보장) worker thread가
    직접 publish까지 해도 안전하다 — avva-platform의 BoundedOperationQueue처럼
    결과를 다시 ROS 스레드로 마샬링하는 별도 scheduler/drain 타이머는 필요 없다.
    """

    def __init__(self, *, name: str) -> None:
        self._name = name
        self._queue: Queue = Queue(maxsize=1)
        self._lock = Lock()
        self._busy = False
        self._failure: BaseException | None = None
        self._thread = Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def try_submit(self, operation: Callable[[], None]) -> bool:
        """capacity 1 — 이미 하나 처리 중이면 콜백 스레드를 안 막고 바로 False."""
        with self._lock:
            if self._failure is not None:
                failure, self._failure = self._failure, None
                raise failure
            if self._busy:
                return False
            self._busy = True
            self._queue.put_nowait(operation)
            return True

    def _run(self) -> None:
        while True:
            operation = self._queue.get()
            try:
                operation()
            except BaseException as exc:  # noqa: BLE001 — 다음 try_submit 호출자에게 넘긴다
                with self._lock:
                    self._failure = exc
            finally:
                with self._lock:
                    self._busy = False

    def join(self, timeout: float) -> None:
        """in-flight 작업이 끝나길 최대 timeout초 기다린다(process shutdown 전용).

        ponytail: timeout을 넘기는 in-flight 작업과의 경합까지는 막지 않는다 —
        정상 종료 경로에서 그렇게 오래 걸리면 이미 backend 쪽 문제.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if not self._busy:
                    return
            time.sleep(0.01)


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
            # sim_id 접두사 없음(evidence/reject는 컴포넌트 공통 채널). depth는
            # yaml이 정한 32(_REJECT_NOTICE_QOS) — SimBackendRosNode와 통일.
            "avva/v1/evidence/reject": self.create_publisher(
                ros_msgs.RejectNotice, "avva/v1/evidence/reject", _REJECT_NOTICE_QOS),
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
        self._worker = _BackendWorker(name=f"sim_backend_{sim_id}_worker")
        # AVVA 공통 설계.docx 확정(2026-08-22): 에러 감지 시 즉시 발행 중단·tick
        # 금지·abort — "hold-last-and-proceed"는 1~2번도 허용 안 함. worker가
        # stuck이면 layer.state를 다른 스레드에서 안전하게 ABORTED로 못 바꾸므로,
        # 이 ROS 경계 자체가 새 RunManifest 전까지 모든 입력을 막는 circuit
        # breaker 역할을 한다(§_submit_or_reject).
        self._worker_stalled = False

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
        COMPONENT_READY_STATE, RUNNING이면 COMPONENT_RUNNING으로만 구분한다.
        worker가 stall된 상태(§_submit_or_reject)면 run_state/layer가 뭐라고
        하든 COMPONENT_ERROR/HEALTH_ERROR로 덮어쓴다 — RejectNotice를 놓쳤어도
        Core가 이 주기 채널 하나만 보고도 반드시 알 수 있게. 이 타이머와 콜백은
        같은 SingleThreadedExecutor 스레드라 _worker_stalled를 lock 없이 읽어도
        안전하다."""
        layer = self.component.layer
        if self._worker_stalled:
            state, health = m.ComponentState.COMPONENT_ERROR, m.HealthStatus.HEALTH_ERROR
        elif self.component.run_state == m.RunState.RUNNING:
            state, health = m.ComponentState.COMPONENT_RUNNING, m.HealthStatus.HEALTH_OK
        elif layer is not None:
            state, health = m.ComponentState.COMPONENT_READY_STATE, m.HealthStatus.HEALTH_OK
        else:
            state, health = m.ComponentState.STARTING, m.HealthStatus.HEALTH_OK
        last_state_tick = (m.OptionalUint64(True, layer.state_tick) if layer is not None
                           else m.OptionalUint64(False, 0))
        hb = m.ComponentHeartbeat(
            header=None, component_state=state, health_status=health,
            last_state_tick_id=last_state_tick,
            # ponytail: target tick·native session은 존재 확인 단계엔 안 채움 — RTT/정량
            # 모니터링 도입 시(§10.6 A안) SimulationLayer에 노출 지점 추가해서 채울 것.
            last_target_tick_id=m.OptionalUint64(False, 0),
            native_session_id=m.OptionalUuid128(False, b"\x00" * 16),
            monotonic_time_ns=time.monotonic_ns())
        hb.header = self.component.build_header(hashing.component_heartbeat_hash(hb))
        self._heartbeat_pub.publish(conv.to_ros_msg(hb, ros_msgs))

    def _publish_stall_reject(self, header: m.CommonHeader, rejected_kind: int) -> None:
        """worker가 stuck일 때 즉시 발행하는 RejectNotice — layer를 전혀 안 건드리고
        component.build_header()(lock-protected)만 써서 어느 스레드에서 불러도
        안전하다. retryable=False: 같은 run으로는 재시도해도 소용없다(abort)."""
        notice = m.RejectNotice(
            header=None, target_component_id=header.producer_id,
            rejected_message_kind=rejected_kind, related_hash=header.payload_hash,
            tick_refs=m.TickRefs(False, 0, False, 0, False, 0, False, b"\x00" * 16),
            reason_code=m.QUEUE_TIMEOUT, retryable=False,
            detail_data=m.DetailData(m.DetailKind.DETAIL_NONE, 0, 0,
                                     m.OptionalBoundedId(False, ""), m.OptionalHash256(False, b"\x00" * 32)),
            detail_message="")
        notice.header = self.component.build_header(hashing.reject_notice_hash(notice))
        pub = (self._frame_publishers or {}).get("avva/v1/evidence/reject")
        if pub is not None:
            pub.publish(conv.to_ros_msg(notice, ros_msgs))

    def _try_submit_or_log(self, label: str, operation: Callable[[], None]) -> bool:
        """try_submit()이 이전 operation의 저장된 예외를 다시 던지는 경우까지
        포함해 "제출 안 됨"으로 통일한다 — 이걸 안 잡으면 예외가 ROS 콜백 밖으로
        그대로 새어나가 stall/RejectNotice 처리를 건너뛰게 된다."""
        try:
            return self._worker.try_submit(operation)
        except BaseException as exc:  # noqa: BLE001 — 다음 시도부턴 worker가 다시 받는다(avva-platform과 동일)
            self.get_logger().error(f"{label}: sim backend worker raised on a previous operation: {exc!r}")
            return False

    def _submit_or_reject(self, label: str, rejected_kind: int, header: m.CommonHeader,
                          operation: Callable[[], None]) -> bool:
        """frame-path 공용 overflow 처리. AVVA 공통 설계.docx 확정: 에러는 즉시
        알리고 이 run은 더 못 쓴다 — worker가 한 번 stuck으로 확인되면 새
        RunManifest가 올 때까지 이 ROS 경계에서 조용히 계속 드롭한다(같은 stall을
        메시지마다 다시 알릴 필요는 없음: §immediate-react는 "첫 발생 즉시 반응"
        이지 "매번 반복 알림"이 아니다).
        """
        if self._worker_stalled:
            return False
        if self._try_submit_or_log(label, operation):
            return True
        self._worker_stalled = True
        self.get_logger().error(f"{label} dropped: sim backend worker stalled -- run aborted at ROS boundary")
        self._publish_stall_reject(header, rejected_kind)
        return False

    def _on_run_manifest(self, ros_msg) -> None:
        manifest = conv.from_ros_msg(ros_msg, m.RunManifest)
        self._worker_stalled = False  # 새 manifest = 새로 시작 시도, 이전 stall 해제

        def _process() -> None:
            result = self.component.on_run_manifest(manifest)
            self._ready_pub.publish(conv.to_ros_msg(result.ready, ros_msgs))

        self._submit_or_reject("RunManifest", m.RUN_MANIFEST, manifest.header, _process)

    def _on_run_control(self, ros_msg) -> None:
        cmd = conv.from_ros_msg(ros_msg, m.RunControlCommand)

        def _process() -> None:
            ack = self.component.on_run_control(cmd)
            self._ack_pub.publish(conv.to_ros_msg(ack, ros_msgs))

        if not self._worker_stalled and self._try_submit_or_log("RunControlCommand", _process):
            return
        # RunControlAck(ACK_REJECTED, QUEUE_TIMEOUT)가 이 채널의 정식 거부
        # 응답이라 RejectNotice를 추가로 발행하진 않는다 — 대신 frame-path도
        # 같이 막히도록 stall 플래그는 여기서도 세운다.
        self._worker_stalled = True
        # component.reject_busy()는 self.run_state만 읽는다(GIL 하 원자적 읽기) —
        # worker가 동시에 쓰고 있어도 죽지 않고 최악의 경우 살짝 오래된 상태만
        # 보고한다. _command_cache는 안 건드리므로 재전송하면 정상 재시도된다.
        ack = self.component.reject_busy(cmd.command_id, cmd.action)
        self._ack_pub.publish(conv.to_ros_msg(ack, ros_msgs))

    def _make_run_transport(self, backend, run_id, run_epoch, sim_id, registry, run_config):
        """RunTransportFactory 구현체. SimulationLayer.publish를 실제 ROS 발행에 잇고,
        frame data path 구독을 component의 대응 메서드로 연결한다.

        미검증 위험(2026-09-18 재검토에서 발견, 리눅스 네이티브 환경에서 재확인
        예정): 이 메서드는 on_run_manifest() 처리 경로를 통해 _BackendWorker
        스레드에서 실행되므로, 아래 create_publisher/create_subscription이
        메인 스레드가 rclpy.spin() 중인 도중 다른 스레드에서 호출된다. rclpy의
        Node.create_subscription()과 executor의 wait-set 구성 코드 둘 다 각자
        `.handle` context manager로 잠그긴 하지만(executors.py), 서로 다른
        handle 객체라 완전히 배타적인지는 소스만으로 확신 못 함 — 이번 세션
        WSL 테스트에서는 반복 실행해도 크래시가 없었고, run당 한 번(첫 READY
        manifest)만 타는 좁은 창이라 실전 위험은 낮아 보이지만 이론적으로
        증명된 건 아니다."""
        ego_ids = sorted(e.actor_id for e in registry if e.role == m.ActorRole.ACTOR_ROLE_EGO)
        if self._frame_publishers is None:
            publishers = {
                f"avva/v1/sim/{sim_id}/world_state": self.create_publisher(
                    ros_msgs.WorldStateFrame, f"avva/v1/sim/{sim_id}/world_state", RELIABLE_NO_DROP),
                f"avva/v1/sim/{sim_id}/command_ready": self.create_publisher(
                    ros_msgs.CommandSetReady, f"avva/v1/sim/{sim_id}/command_ready", RELIABLE_NO_DROP),
                f"avva/v1/sim/{sim_id}/frame_complete": self.create_publisher(
                    ros_msgs.FrameComplete, f"avva/v1/sim/{sim_id}/frame_complete", RELIABLE_NO_DROP),
                # channel_registry.yaml: evidence/reject는 depth 32.
                "avva/v1/evidence/reject": self.create_publisher(
                    ros_msgs.RejectNotice, "avva/v1/evidence/reject", _REJECT_NOTICE_QOS),
            }
            for ego_id in ego_ids:
                topic = f"avva/v1/sim/{sim_id}/ego/{ego_id}/observation"
                publishers[topic] = self.create_publisher(ros_msgs.EgoObservationFrame, topic, RELIABLE_NO_DROP)

            self.create_subscription(
                ros_msgs.NpcControlBatch, f"avva/v1/sim/{sim_id}/npc_control",
                lambda ros_msg: (lambda msg: self._submit_or_reject(
                    "NpcControlBatch", m.NPC_CONTROL_BATCH, msg.header,
                    lambda: self.component.on_npc_control_batch(msg))
                )(conv.from_ros_msg(ros_msg, m.NpcControlBatch)),
                RELIABLE_NO_DROP)
            self.create_subscription(
                ros_msgs.AdvanceFrame, f"avva/v1/sim/{sim_id}/advance",
                lambda ros_msg: (lambda msg: self._submit_or_reject(
                    "AdvanceFrame", m.ADVANCE_FRAME, msg.header,
                    lambda: self.component.on_advance_frame(msg))
                )(conv.from_ros_msg(ros_msg, m.AdvanceFrame)),
                RELIABLE_NO_DROP)
            self.create_subscription(
                ros_msgs.FrameCompleteAck, f"avva/v1/sim/{sim_id}/frame_complete_ack",
                lambda ros_msg: (lambda msg: self._submit_or_reject(
                    "FrameCompleteAck", m.FRAME_COMPLETE_ACK, msg.header,
                    lambda: self.component.on_frame_complete_ack(msg))
                )(conv.from_ros_msg(ros_msg, m.FrameCompleteAck)),
                RELIABLE_NO_DROP)
            for ego_id in ego_ids:
                topic = f"avva/v1/sim/{sim_id}/ego/{ego_id}/control"
                self.create_subscription(
                    ros_msgs.EgoControlCommand, topic,
                    lambda ros_msg: (lambda msg: self._submit_or_reject(
                        "EgoControlCommand", m.EGO_CONTROL_COMMAND, msg.header,
                        lambda: self.component.on_ego_control(msg))
                    )(conv.from_ros_msg(ros_msg, m.EgoControlCommand)),
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
        """프로세스 종료 시 호출 — worker의 in-flight 작업을 잠깐 기다린 뒤(최대 1초)
        component.shutdown()으로 활성 backend까지 정리하고 ROS node를 파괴한다.
        destroy_node()만 직접 부르면 backend(예: CARLA connection)가 안 닫혀서
        리소스가 샌다."""
        self._worker.join(timeout=1.0)
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
