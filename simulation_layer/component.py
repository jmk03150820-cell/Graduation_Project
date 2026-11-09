"""Public interface 파사드 (5번 공지 1번): 이 컴포넌트 밖에서 호출하는 진입점 전부를
여기 하나로 모은다 — on_run_manifest / on_ego_control / on_npc_control_batch /
on_advance_frame / on_run_control(+abort_run) / shutdown.

gate.py(frame 파이프라인)와 startup.py(RunManifest -> ComponentReady)는 그대로 두고,
이 파일은 그 둘을 이름이 맞는 메서드로 감싸기만 한다 — 새 로직 없음.

RunEndRestart_Rule.md §1: ABORT는 "현재 Run"만 정리하고 프로세스/컴포넌트 인스턴스는
살아남는다 — abort_run()은 backend.shutdown()+layer 폐기만 하고, 이 인스턴스는 그대로
남아 다음 on_run_manifest()를 다시 받을 수 있다(재시작).
"""
from __future__ import annotations

import time
import uuid
from threading import Lock

import avva_phase1 as m

from . import hashing
from .startup import RunTransportFactory, StartupResult, enter_running, start_run

_NO_ID = m.OptionalBoundedId(False, "")
_NO_UUID = m.OptionalUuid128(False, b"\x00" * 16)
_NO_U64 = m.OptionalUint64(False, 0)


def _control_ack_header(sim_id: str, run_id: bytes, run_epoch: int, payload_hash: bytes,
                        producer_instance_id: bytes, event_seq: int) -> m.CommonHeader:
    return m.CommonHeader(
        schema_major=1, schema_minor=0, run_id=run_id, run_epoch=run_epoch,
        scope_kind=m.ScopeKind.SIM, sim_id=m.OptionalBoundedId(True, sim_id), ego_id=_NO_ID,
        producer_id=m.ComponentId.SIM_BACKEND, producer_instance_id=producer_instance_id,
        event_seq=event_seq, correlation_id=_NO_UUID, sim_time_ns=_NO_U64,
        wall_time_unix_ns=time.time_ns(), payload_hash=payload_hash)


def _control_ack(sim_id: str, run_id: bytes, run_epoch: int, command_id: bytes,
                 action: m.RunControlAction, ack_status: m.AckStatus, run_state: m.RunState,
                 reason_code: int, producer_instance_id: bytes, event_seq: int) -> m.RunControlAck:
    ack = m.RunControlAck(header=None, command_id=command_id, action=action,
                          ack_status=ack_status, current_run_state=run_state, reason_code=reason_code)
    ack.header = _control_ack_header(sim_id, run_id, run_epoch, hashing.run_control_ack_hash(ack),
                                     producer_instance_id, event_seq)
    return ack


class SimBackendComponent:
    """1개 sim_id를 담당하는 Sim Backend 컴포넌트 — 외부(transport binding)는
    이 6개 메서드만 호출한다. Bootstrap Transport 준비는 이 클래스 밖(호출자) 책임."""

    def __init__(self, own_sim_id: str, run_transport_factory: RunTransportFactory | None = None) -> None:
        self.own_sim_id = own_sim_id
        self._run_transport_factory = run_transport_factory
        self.run_state = m.RunState.IDLE
        self.layer = None
        self.backend = None
        self.transport = None
        self._run_id = b"\x00" * 16
        self._run_epoch = 0
        # 재전송 안전장치(avva-platform ControlStatusChannel의 identity+digest dedup
        # 원리를 이 클래스 안에 최소 이식, Sim 파트 대조 §10.2): command_id -> (요청
        # payload_hash, 확정된 RunControlAck). 같은 command_id가 같은 내용으로 다시
        # 오면 재평가하지 않고 저장된 답을 그대로 반환 — enter_running()/abort_run()의
        # side effect가 재전송마다 중복 실행되는 걸 막는다.
        self._command_cache: dict[bytes, tuple[bytes, m.RunControlAck]] = {}
        # 기준서 §5.2: producer_instance_id는 프로세스(=이 컴포넌트 인스턴스) 시작마다
        # 새 값 하나, event_seq는 그 안에서 계속 단조 증가 — 메시지마다 새로 만들면
        # Core가 매번 "새 인스턴스"로 관측해서 SEQUENCE_ANOMALY 탐지가 무력화된다.
        self._producer_instance_id = uuid.uuid4().bytes
        self._event_seq = 0
        # ROS 경계에 worker thread를 두면(ros2_node.py) overflow 시 콜백 스레드가
        # worker와 동시에 header를 만들 수 있다 — event_seq 재사용/역전을 막는 lock.
        self._event_seq_lock = Lock()

    def _next_event_seq(self) -> int:
        with self._event_seq_lock:
            self._event_seq += 1
            return self._event_seq

    @property
    def producer_instance_id(self) -> bytes:
        """§11②: 이 컴포넌트 인스턴스의 고정 식별자 — heartbeat 등 외부에서 header를
        직접 만들어야 하는 경우에도 control channel과 같은 identity를 쓰게 한다."""
        return self._producer_instance_id

    def build_header(self, payload_hash: bytes) -> m.CommonHeader:
        """control channel 밖(예: heartbeat)에서도 같은 규칙(SIM scope, 같은
        producer_instance_id, 단조 event_seq)으로 header를 만들 수 있게 노출."""
        return _control_ack_header(self.own_sim_id, self._run_id, self._run_epoch, payload_hash,
                                   self._producer_instance_id, self._next_event_seq())

    def _ack(self, command_id: bytes, action: m.RunControlAction, ack_status: m.AckStatus,
             reason_code: int = m.UNKNOWN_REASON) -> m.RunControlAck:
        return _control_ack(self.own_sim_id, self._run_id, self._run_epoch, command_id, action,
                            ack_status, self.run_state, reason_code,
                            self._producer_instance_id, self._next_event_seq())

    # ---- 1) RunManifest -> ComponentReady (startup.py 위임) ----------------
    def on_run_manifest(self, manifest: m.RunManifest) -> StartupResult:
        kwargs = {} if self._run_transport_factory is None else {"run_transport_factory": self._run_transport_factory}
        result = start_run(manifest, self.own_sim_id, producer_instance_id=self._producer_instance_id,
                           event_seq=self._next_event_seq(), **kwargs)
        if result.ready.ready_status == m.ReadyStatus.READY:
            old_backend = self.backend  # candidate 성공 = 이전 active run을 새 run으로 교체
            self.layer, self.backend, self.transport = result.layer, result.backend, result.transport
            self._run_id, self._run_epoch = manifest.header.run_id, manifest.header.run_epoch
            self.run_state = m.RunState.IDLE  # RunControlCommand(START) 전까지는 RUNNING 아님
            self._command_cache = {}  # 새 run의 command_id는 이전 run 캐시와 무관 — 무한 누적 방지
            if old_backend is not None:
                old_backend.shutdown()  # 교체된 이전 run의 native 리소스 정리 (안 하면 leak)
        return result

    # ---- frame data path (gate.py 위임, 계약 이름으로 노출) -----------------
    # 실제 transport 위에서는 순서가 뒤집힐 수 있다(RunManifest/START 전에 frame
    # 메시지가 먼저 도착 등) — self.layer is None일 때 그냥 죽으면 안 되므로 무시.
    # RejectNotice를 못 만드는 이유: run context(run_id) 자체가 아직 없어 어느
    # run에 대한 거부인지 표현할 방법이 없음 — Bootstrap Transport 단에서 걸러야
    # 하는 문제일 수도 있어 팀 확인 대상으로 남긴다.
    def on_ego_control(self, msg: m.EgoControlCommand) -> None:
        if self.layer is not None:
            self.layer.on_ego_control(msg)

    def on_npc_control_batch(self, msg: m.NpcControlBatch) -> None:
        if self.layer is not None:
            self.layer.on_npc_batch(msg)

    def on_advance_frame(self, msg: m.AdvanceFrame) -> None:
        if self.layer is not None:
            self.layer.on_advance_frame(msg)

    def on_frame_complete_ack(self, msg: m.FrameCompleteAck) -> None:
        if self.layer is not None:
            self.layer.on_frame_complete_ack(msg)

    # ---- RunControlCommand -> RunControlAck --------------------------------
    def on_run_control(self, msg: m.RunControlCommand) -> m.RunControlAck:
        # gate.py._check_run과 동일한 원칙: run이 이미 떠 있는데 다른 run_id/epoch를
        # 향한 명령이면(stale/오배달) 절대 그대로 처리하면 안 된다 — 안 하면 지난
        # run 겨냥 RUN_ABORT가 현재 활성 run을 잘못 abort시킬 수 있음.
        if self.layer is not None and (msg.header.run_id != self._run_id
                                       or msg.header.run_epoch != self._run_epoch):
            reason = m.RUN_MISMATCH if msg.header.run_id != self._run_id else m.EPOCH_MISMATCH
            return self._ack(msg.command_id, msg.action, m.AckStatus.ACK_REJECTED, reason)
        cached = self._command_cache.get(msg.command_id)
        if cached is not None:
            cached_hash, cached_ack = cached
            if cached_hash == msg.header.payload_hash:
                # 재전송 — 재평가 없이 저장된 답 재확인. abort 이후(layer=None)에도 이전
                # command_id가 재전송되면 abort 전 상태를 담은 옛 ack가 그대로 나간다 —
                # 이건 라이브 상태 조회가 아니라 "그 command가 그때 어떻게 처리됐는지"의
                # 역사적 기록이라 의도된 동작이다(avva-platform ControlStatusChannel과 동일).
                return cached_ack
            # 같은 command_id를 다른 내용으로 재사용 — 모순이라 거부(gate.py._dedup()의
            # duplicate/conflict 구분과 동일 원칙)
            return self._ack(msg.command_id, msg.action, m.AckStatus.ACK_REJECTED, m.PAYLOAD_CONFLICT)
        if msg.action == m.RunControlAction.START:
            if self.layer is None or self.run_state != m.RunState.IDLE:
                ack = self._ack(msg.command_id, msg.action, m.AckStatus.ACK_REJECTED, m.INVALID_STATE)
            else:
                enter_running(self.layer)
                self.run_state = m.RunState.RUNNING
                ack = self._ack(msg.command_id, msg.action, m.AckStatus.ACCEPTED)
        elif msg.action == m.RunControlAction.RUN_ABORT:
            ack = self.abort_run(msg.command_id, msg.reason_code or m.CORE_ABORTED)
        else:
            # PREPARE/PAUSE/RESUME: 기준서 §6.2엔 정의돼 있지만(PREPARE=resource 준비 시작,
            # PAUSE/RESUME=native tick 경계에서 진행/재개) gate.py FSM엔 그 상태 전이가
            # 아직 없다 — Phase 1 축소 구현. 이 상황에 맞는 전용 ReasonCode도 계약에
            # 없어 INVALID_STATE로 대체 — 둘 다 팀 확인/후속 구현 대상.
            ack = self._ack(msg.command_id, msg.action, m.AckStatus.ACK_REJECTED, m.INVALID_STATE)
        self._command_cache[msg.command_id] = (msg.header.payload_hash, ack)
        return ack

    def reject_busy(self, command_id: bytes, action: m.RunControlAction) -> m.RunControlAck:
        """ros2_node.py의 worker queue가 가득 찼을 때 콜백 스레드에서 안전하게 부르는
        거부 ack — QUEUE_TIMEOUT(4002, 계약에 이미 정의돼 있었지만 미사용이던 코드)을
        쓴다. _command_cache에는 넣지 않는다: 이건 이 command_id에 대한 최종 판정이
        아니라 "지금은 못 받았다"이므로, 같은 command_id로 재전송되면 worker가 비었을
        때 정상적으로 다시 시도될 수 있어야 한다."""
        return self._ack(command_id, action, m.AckStatus.ACK_REJECTED, m.QUEUE_TIMEOUT)

    def abort_run(self, command_id: bytes = b"\x00" * 16, reason_code: int = m.UNKNOWN_REASON) -> m.RunControlAck:
        """RunEndRestart_Rule.md §1: 현재 Run만 정리, 컴포넌트 인스턴스는 살아남는다."""
        if self.backend is not None:
            self.backend.shutdown()
        self.layer = self.backend = self.transport = None
        self.run_state = m.RunState.IDLE
        return self._ack(command_id, m.RunControlAction.RUN_ABORT, m.AckStatus.ACCEPTED, reason_code)

    # ---- shutdown -----------------------------------------------------------
    def shutdown(self) -> None:
        """프로세스 종료 직전 정리. Run이 떠 있으면 backend까지 정리한다."""
        if self.backend is not None:
            self.backend.shutdown()
        self.layer = self.backend = self.transport = None
        self.run_state = m.RunState.RUN_UNKNOWN
