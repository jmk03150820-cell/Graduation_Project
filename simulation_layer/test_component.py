"""SimBackendComponent 공개 인터페이스(5번 공지 1번) + RunControlCommand/ABORT/restart
(5번) + RejectNotice 발행(2번) 계약 시험.

Runs with `python -m simulation_layer.test_component` (assert-based) or pytest.
"""
from __future__ import annotations

import uuid

import simulation_layer  # noqa: F401  (interfaces path shim)
import avva_phase1 as m
from simulation_layer import hashing
from simulation_layer.component import SimBackendComponent
from simulation_layer.test_frame_lifecycle import by_channel
from simulation_layer.test_startup import SIM_ID, _manifest


def _run_control_cmd(manifest: m.RunManifest, action: m.RunControlAction, reason_code: int = 0) -> m.RunControlCommand:
    """실제 Core라면 RunManifest와 같은 run_id/epoch를 담아 보낼 RunControlCommand.
    on_run_control()이 이제 이 header의 run_id/epoch를 활성 run과 대조하므로
    (test_run_control_rejects_stale_run_id_or_epoch), 시험용 명령도 유효한 header가 필요."""
    return m.RunControlCommand(
        header=m.CommonHeader(
            schema_major=1, schema_minor=0, run_id=manifest.header.run_id, run_epoch=manifest.header.run_epoch,
            scope_kind=m.ScopeKind.GLOBAL, sim_id=m.OptionalBoundedId(False, ""),
            ego_id=m.OptionalBoundedId(False, ""), producer_id=m.ComponentId.CORE,
            producer_instance_id=b"\x02" * 16, event_seq=1,
            correlation_id=m.OptionalUuid128(False, b"\x00" * 16), sim_time_ns=m.OptionalUint64(False, 0),
            wall_time_unix_ns=0, payload_hash=b"\x00" * 32),
        command_id=uuid.uuid4().bytes, action=action, reason_code=reason_code)


def test_run_manifest_then_run_control_start_reaches_running():
    """on_run_manifest() -> READY, on_run_control(START) -> RunControlAck(ACCEPTED,
    RUNNING) 이후 실제 RUNNING(최초 관측 발행)까지 — 인터페이스 6개 중 3개 연결 확인."""
    comp = SimBackendComponent(SIM_ID)
    manifest = _manifest()
    result = comp.on_run_manifest(manifest)
    assert result.ready.ready_status == m.ReadyStatus.READY
    assert comp.run_state == m.RunState.IDLE  # START 전

    ack = comp.on_run_control(_run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON))
    assert ack.ack_status == m.AckStatus.ACCEPTED and ack.current_run_state == m.RunState.RUNNING
    assert ack.header.payload_hash == hashing.run_control_ack_hash(ack)
    assert comp.run_state == m.RunState.RUNNING
    ws = by_channel(comp.transport, "world_state")
    assert len(ws) == 1 and ws[0].state_tick_id == 0


def test_abort_run_tears_down_and_allows_restart():
    """RUN_ABORT -> RunControlAck(ACCEPTED, IDLE) + layer/backend 완전 폐기, 그러나
    컴포넌트 인스턴스 자체는 살아남아 새 epoch RunManifest로 재시작 가능해야 한다
    (RunEndRestart_Rule.md §1: 프로세스/Bootstrap Transport는 유지)."""
    comp = SimBackendComponent(SIM_ID)
    manifest = _manifest()
    comp.on_run_manifest(manifest)
    comp.on_run_control(_run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON))
    assert comp.run_state == m.RunState.RUNNING

    abort_ack = comp.on_run_control(_run_control_cmd(manifest, m.RunControlAction.RUN_ABORT, m.CORE_ABORTED))
    assert abort_ack.ack_status == m.AckStatus.ACCEPTED
    assert abort_ack.current_run_state == m.RunState.IDLE
    assert comp.layer is None and comp.backend is None, "ABORT는 현재 Run(layer/backend)을 완전히 폐기해야 함"
    assert comp.run_state == m.RunState.IDLE

    manifest2 = _manifest()
    manifest2.header.run_epoch = manifest.header.run_epoch + 1
    result2 = comp.on_run_manifest(manifest2)
    assert result2.ready.ready_status == m.ReadyStatus.READY, "abort 이후 같은 컴포넌트로 재시작이 안 됨"
    assert comp.layer is not None and comp.backend is not None


def test_reject_notice_published_on_bad_ego_control():
    """§2 출력 경계: 잘못된 메시지를 받으면 내부 로그(events)뿐 아니라 실제
    RejectNotice가 avva/v1/evidence/reject로 발행돼야 한다 — 지금까지는 events에만
    남고 외부로는 한 번도 안 나갔던 gap."""
    comp = SimBackendComponent(SIM_ID)
    manifest = _manifest()
    comp.on_run_manifest(manifest)
    comp.on_run_control(_run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON))

    bad = m.EgoControlCommand(
        header=None, based_on_tick_id=0, target_tick_id=1, source_stack=m.SourceStack.MODULE_CHAIN,
        control_time_sim_ns=0, command_status=m.CommandStatus.COMMAND_OK,
        control=m.OptionalNeutralControl(False, None), failure_reason=m.UNKNOWN_REASON)
    bad.header = m.CommonHeader(
        schema_major=1, schema_minor=0, run_id=manifest.header.run_id, run_epoch=manifest.header.run_epoch,
        scope_kind=m.ScopeKind.SIM, sim_id=m.OptionalBoundedId(True, SIM_ID),
        ego_id=m.OptionalBoundedId(False, ""),  # scope 위반: ego_id 없음 -> INVALID_HEADER_SCOPE
        producer_id=m.ComponentId.EGO_ADAPTER, producer_instance_id=b"\x09" * 16, event_seq=1,
        correlation_id=m.OptionalUuid128(False, b"\x00" * 16), sim_time_ns=m.OptionalUint64(False, 0),
        wall_time_unix_ns=0, payload_hash=hashing.ego_control_command_hash(bad))

    comp.on_ego_control(bad)
    rejects = by_channel(comp.transport, "evidence/reject")
    assert len(rejects) == 1, "REJECT 이벤트가 RejectNotice로 발행되지 않음"
    rn = rejects[0]
    assert rn.reason_code == m.INVALID_HEADER_SCOPE
    assert rn.rejected_message_kind == m.EGO_CONTROL_COMMAND
    assert rn.target_component_id == m.ComponentId.EGO_ADAPTER, "거부 대상은 원 메시지의 producer"
    assert rn.retryable is True, "state 안 바뀌는 REJECT는 고쳐서 재전송 가능"
    assert rn.related_hash == bad.header.payload_hash
    assert rn.header.payload_hash == hashing.reject_notice_hash(rn)


def test_frame_messages_before_run_do_not_crash():
    """실제 transport에서는 메시지 순서가 뒤집힐 수 있다 — RunManifest/START 전에
    frame 데이터 경로 메시지가 먼저 도착해도 AttributeError로 죽으면 안 된다."""
    comp = SimBackendComponent(SIM_ID)
    dummy = object()  # self.layer is None이면 msg를 건드리지도 않아야 함
    comp.on_ego_control(dummy)
    comp.on_npc_control_batch(dummy)
    comp.on_advance_frame(dummy)
    comp.on_frame_complete_ack(dummy)
    assert comp.layer is None  # 아무 상태도 안 생김


def test_run_control_rejects_stale_run_id_or_epoch():
    """다른 run_id/epoch를 향한 RunControlCommand는 활성 run에 영향을 주면 안 된다
    — stale RUN_ABORT가 새로 시작한(또는 계속 도는) 활성 run을 잘못 꺼뜨리는 사고 방지
    (gate.py._check_run과 같은 원칙을 RunControlCommand에도 적용)."""
    comp = SimBackendComponent(SIM_ID)
    manifest = _manifest()
    comp.on_run_manifest(manifest)
    comp.on_run_control(_run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON))
    assert comp.run_state == m.RunState.RUNNING

    stale_abort = m.RunControlCommand(
        header=m.CommonHeader(
            schema_major=1, schema_minor=0, run_id=manifest.header.run_id,
            run_epoch=manifest.header.run_epoch + 99,  # 존재하지 않는 미래 epoch
            scope_kind=m.ScopeKind.GLOBAL, sim_id=m.OptionalBoundedId(False, ""),
            ego_id=m.OptionalBoundedId(False, ""), producer_id=m.ComponentId.CORE,
            producer_instance_id=b"\x02" * 16, event_seq=1,
            correlation_id=m.OptionalUuid128(False, b"\x00" * 16), sim_time_ns=m.OptionalUint64(False, 0),
            wall_time_unix_ns=0, payload_hash=b"\x00" * 32),
        command_id=uuid.uuid4().bytes, action=m.RunControlAction.RUN_ABORT, reason_code=m.CORE_ABORTED)

    ack = comp.on_run_control(stale_abort)
    assert ack.ack_status == m.AckStatus.ACK_REJECTED and ack.reason_code == m.EPOCH_MISMATCH
    assert comp.run_state == m.RunState.RUNNING, "stale epoch의 RUN_ABORT가 활성 run을 꺼뜨리면 안 됨"
    assert comp.layer is not None and comp.backend is not None


def test_new_manifest_shuts_down_replaced_backend():
    """RunManifest가 새로 와서 이미 떠 있는 run을 대체하면 이전 backend가 leak되지
    않고 shutdown()이 호출돼야 한다."""
    from simulation_layer.mock_backend import MockSimulatorAdapter

    shutdown_calls: list = []
    original_shutdown = MockSimulatorAdapter.shutdown

    def tracked_shutdown(self):
        shutdown_calls.append(self)
        original_shutdown(self)

    MockSimulatorAdapter.shutdown = tracked_shutdown
    try:
        comp = SimBackendComponent(SIM_ID)
        manifest1 = _manifest()
        comp.on_run_manifest(manifest1)
        old_backend = comp.backend
        assert shutdown_calls == []

        manifest2 = _manifest()
        manifest2.header.run_epoch = manifest1.header.run_epoch + 1
        comp.on_run_manifest(manifest2)
        assert shutdown_calls == [old_backend], "교체된 이전 backend가 shutdown되지 않음(leak)"
        assert comp.backend is not old_backend
    finally:
        MockSimulatorAdapter.shutdown = original_shutdown


def test_tick_refs_encoding_is_conditional_on_has_flags():
    """TickRefs.has_X는 OptionalXxx 타입은 아니지만 presence flag 역할은 동일 —
    has_X가 False면 그 값은 canonical bytes에 전혀 나타나면 안 된다. 예전 버전은
    has_decision_id만 조건부고 나머지 3개는 항상 u64를 썼던 버그였음(팀의 다른
    컴포넌트 인코더와 hash가 어긋날 수 있었음)."""
    from avva_hash_v1 import CanonicalWriter
    from simulation_layer.hashing import _tick_refs

    all_false = m.TickRefs(False, 999, False, 999, False, 999, False, b"\xff" * 16)
    w = CanonicalWriter()
    _tick_refs(w, all_false)
    assert w.finish() == b"\x00\x00\x00\x00", "has_X=False인데 쓰레기 값이 인코딩에 섞여 들어감"

    only_state = m.TickRefs(True, 7, False, 0, False, 0, False, b"\x00" * 16)
    w2 = CanonicalWriter()
    _tick_refs(w2, only_state)
    assert w2.finish() == b"\x01" + (7).to_bytes(8, "little") + b"\x00\x00\x00"


def test_run_control_retransmit_is_idempotent_conflict_rejected():
    """재전송 안전장치(Sim 파트 대조 §10.2 이식): 같은 command_id+같은 payload_hash
    재전송은 재평가 없이 저장된 RunControlAck를 그대로 반환한다(enter_running()이
    두 번 안 불림). 같은 command_id를 다른 payload_hash로 재사용하면 모순으로 보고
    PAYLOAD_CONFLICT로 거부하며 실제 run 상태는 안 건드린다."""
    comp = SimBackendComponent(SIM_ID)
    manifest = _manifest()
    comp.on_run_manifest(manifest)

    fixed_command_id = uuid.uuid4().bytes
    first = _run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON)
    first.command_id = fixed_command_id
    ack1 = comp.on_run_control(first)
    assert ack1.ack_status == m.AckStatus.ACCEPTED
    assert comp.run_state == m.RunState.RUNNING

    retry = _run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON)
    retry.command_id = fixed_command_id  # _run_control_cmd()의 payload_hash 기본값은 first와 동일
    ack2 = comp.on_run_control(retry)
    assert ack2 is ack1, "동일 재전송은 재평가 없이 저장된 답 그대로 반환해야 함"

    conflict = _run_control_cmd(manifest, m.RunControlAction.RUN_ABORT, m.CORE_ABORTED)
    conflict.command_id = fixed_command_id
    conflict.header.payload_hash = b"\xff" * 32  # 같은 command_id, 다른 내용
    ack3 = comp.on_run_control(conflict)
    assert ack3.ack_status == m.AckStatus.ACK_REJECTED and ack3.reason_code == m.PAYLOAD_CONFLICT
    assert comp.run_state == m.RunState.RUNNING, "충돌 재전송이 실제 run 상태를 건드리면 안 됨(abort 실행 안 됨)"


def test_command_cache_resets_on_new_run():
    """새 RunManifest가 활성 run을 교체하면 이전 run의 _command_cache도 같이
    비워져야 한다 — 안 그러면 프로세스 수명 동안 run을 재시작할 때마다 캐시가
    무한정 쌓인다(재전송 안전장치 자체의 메모리 누수)."""
    comp = SimBackendComponent(SIM_ID)
    manifest1 = _manifest()
    comp.on_run_manifest(manifest1)
    comp.on_run_control(_run_control_cmd(manifest1, m.RunControlAction.START, m.UNKNOWN_REASON))
    assert len(comp._command_cache) == 1

    manifest2 = _manifest()
    manifest2.header.run_epoch = manifest1.header.run_epoch + 1
    comp.on_run_manifest(manifest2)
    assert comp._command_cache == {}, "새 run 시작 후에도 이전 run의 command 캐시가 남아있음"


def test_component_ready_and_ack_share_stable_producer_instance():
    """기준서 §5.2: producer_instance_id는 프로세스(=이 컴포넌트 인스턴스) 시작마다
    새 값 하나, event_seq는 그 안에서 계속 단조 증가 — 메시지마다 새로 만들면 Core가
    매번 "새 인스턴스"로 관측해서 SEQUENCE_ANOMALY 탐지가 무력화된다. 예전엔
    startup.py/component.py가 호출마다 uuid4()를 새로 만들고 event_seq=1을
    하드코딩했다."""
    comp = SimBackendComponent(SIM_ID)
    manifest = _manifest()
    result = comp.on_run_manifest(manifest)
    ready_pid = result.ready.header.producer_instance_id
    ready_seq = result.ready.header.event_seq

    ack = comp.on_run_control(_run_control_cmd(manifest, m.RunControlAction.START, m.UNKNOWN_REASON))
    assert ack.header.producer_instance_id == ready_pid, "같은 컴포넌트 인스턴스인데 producer_instance_id가 바뀜"
    assert ack.header.event_seq > ready_seq, "event_seq가 단조 증가하지 않음"


TESTS = [
    (test_run_manifest_then_run_control_start_reaches_running,
     "on_run_manifest() -> ComponentReady(READY) -> on_run_control(START) -> "
     "RunControlAck(ACCEPTED, RUNNING) -> 실제 최초 관측 발행까지, 공개 인터페이스 이름으로 연결 확인"),
    (test_abort_run_tears_down_and_allows_restart,
     "RUN_ABORT -> RunControlAck(ACCEPTED, IDLE) + layer/backend 완전 폐기, 컴포넌트 인스턴스는 "
     "살아남아 새 epoch RunManifest로 재시작(restart) 가능"),
    (test_reject_notice_published_on_bad_ego_control,
     "잘못된 EgoControlCommand(scope 위반) -> RejectNotice가 avva/v1/evidence/reject로 실제 발행, "
     "target_component_id=원 producer, retryable=True, hash 자체 검증"),
    (test_frame_messages_before_run_do_not_crash,
     "RunManifest/START 전에 frame 데이터 경로 메시지가 먼저 도착해도 AttributeError 없이 무시됨"),
    (test_run_control_rejects_stale_run_id_or_epoch,
     "다른(존재하지 않는) epoch를 향한 RUN_ABORT는 ACK_REJECTED+EPOCH_MISMATCH로 거부되고 "
     "활성 run은 전혀 영향받지 않음"),
    (test_new_manifest_shuts_down_replaced_backend,
     "이미 떠 있는 run을 새 RunManifest가 대체하면 이전 backend.shutdown()이 실제로 호출됨(leak 방지)"),
    (test_run_control_retransmit_is_idempotent_conflict_rejected,
     "같은 command_id+payload_hash 재전송 -> 저장된 RunControlAck 그대로 반환(재평가 없음), "
     "같은 command_id+다른 payload_hash -> PAYLOAD_CONFLICT 거부, run 상태 불변"),
    (test_command_cache_resets_on_new_run,
     "새 RunManifest로 run이 교체되면 이전 run의 _command_cache가 비워짐(무한 누적 방지)"),
    (test_tick_refs_encoding_is_conditional_on_has_flags,
     "TickRefs has_X=False -> 그 필드 값은 canonical bytes에 전혀 안 나타남(Optional 패턴과 동일 취급)"),
    (test_component_ready_and_ack_share_stable_producer_instance,
     "기준서 §5.2: ComponentReady와 그 뒤 RunControlAck가 같은 producer_instance_id를 "
     "공유하고 event_seq가 단조 증가 — 예전엔 호출마다 새 uuid4()+event_seq=1 하드코딩"),
]

if __name__ == "__main__":
    for i, (fn, desc) in enumerate(TESTS, 1):
        fn()
        print(f"[{i}/{len(TESTS)}] PASS {fn.__name__}\n         검증: {desc}")
    print(f"\n전체 {len(TESTS)}개 component 계약 시험 PASS")
