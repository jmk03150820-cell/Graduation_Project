"""startup.py 4개 시나리오 계약 시험 (오늘 공지 RunEndRestart_Rule.md 반영):

1) 정상 Manifest -> Run Transport READY -> ComponentReady(READY) -> RUNNING
2) 지원하지 않는 simulator/rate -> READY_REJECTED + MANIFEST_MISMATCH
3) Run Transport 생성 실패 -> READY_REJECTED + TRANSPORT_ERROR
4) READY 전에 AdvanceFrame -> native step 0

Runs with `python -m simulation_layer.test_startup` (assert-based) or pytest.
"""
from __future__ import annotations

import uuid

import simulation_layer  # noqa: F401  (interfaces path shim)
import avva_phase1 as m
from simulation_layer import hashing
from simulation_layer.startup import enter_running, start_run
from simulation_layer.test_frame_lifecycle import by_channel

RUN_ID = uuid.uuid4().bytes
EPOCH = 1
SIM_ID = "mock_0"
FIXED_STEP_NS = 50_000_000  # 20 Hz


def _manifest(sim_id: str = SIM_ID, adapter_type=m.NativeAdapterType.NATIVE_ADAPTER_UNSPECIFIED,
             fixed_step_ns: int = FIXED_STEP_NS) -> m.RunManifest:
    """최소 필드로 채운 RunManifest. header.payload_hash는 §4.6 RunManifest
    canonical encoder가 아직 없어(오늘 구현 범위 밖) 자리표시값을 쓴다 —
    start_run()은 그 값을 검증하지 않고 ComponentReady.manifest_hash로 그대로
    echo하기만 하므로 이 시험 목적엔 영향 없다(팀 확인 대상으로 플래그).
    """
    header = m.CommonHeader(
        schema_major=1, schema_minor=0, run_id=RUN_ID, run_epoch=EPOCH, scope_kind=m.ScopeKind.GLOBAL,
        sim_id=m.OptionalBoundedId(False, ""), ego_id=m.OptionalBoundedId(False, ""),
        producer_id=m.ComponentId.CORE, producer_instance_id=b"\x02" * 16, event_seq=1,
        correlation_id=m.OptionalUuid128(False, b"\x00" * 16), sim_time_ns=m.OptionalUint64(False, 0),
        wall_time_unix_ns=0, payload_hash=hashing.sha256(b"manifest-placeholder"))
    return m.RunManifest(
        header=header, manifest_version=m.SemanticVersion(1, 0, 0),
        execution_mode=m.ExecutionMode.SYNC_FIXED_STEP, fixed_step_ns=fixed_step_ns,
        warmup_ticks=0, seed=0,
        map_ref=m.ContentRef("town_0", b"\x00" * 32), scenario_ref=m.ContentRef("scenario_0", b"\x00" * 32),
        sim_instances=[m.SimInstanceProfile(
            sim_id=sim_id, adapter_type=adapter_type, adapter_version="0.1",
            capability=m.AdapterCapability(control_mode_mask=0, supports_physics=True,
                                           supports_spawn=True, supports_despawn=True, max_actor_count=10),
            determinism=m.DeterminismCapability.DETERMINISM_UNKNOWN)],
        required_sim_ids=[sim_id],
        ego_profiles=[m.EgoProfile(ego_id="ego_0", source_stack=m.SourceStack.MODULE_CHAIN,
                                   plant_type=m.PlantType.VIRTUAL, vehicle_profile_id="default")],
        traffic_profile=m.TrafficProfile(engine_type=m.TrafficEngineType.TRAFFIC_ENGINE_UNSPECIFIED,
                                         binary_version="", fixed_step_ns=fixed_step_ns,
                                         derived_seed=0, demand_hash=b"\x00" * 32),
        max_actor_count=1200, max_report_count=32, max_prediction_points=32,
        deadline_budget_ns=fixed_step_ns,
        timeouts=m.TimeoutConfig(*([1_000_000_000] * 6)),
        transport_profile=m.TransportProfile.INPROC_TEST, software_versions=[],
        logging_profile=m.LoggingProfile(critical_depth=0, general_depth=0, flush_period_ns=0, retention_days=0))


def test_normal_manifest_reaches_running():
    """1) 정상 Manifest -> Run Transport READY -> ComponentReady(READY) -> RUNNING."""
    manifest = _manifest()
    result = start_run(manifest, SIM_ID)

    assert result.ready.ready_status == m.ReadyStatus.READY
    assert result.ready.reason_codes == []
    assert result.ready.manifest_hash == manifest.header.payload_hash  # echo
    assert result.ready.header.payload_hash == hashing.component_ready_hash(result.ready)
    assert result.layer is not None and result.backend is not None
    assert result.ready.capability_digest != b"\x00" * 32

    # RUNNING 진입 전에는 아직 관측이 없다 (bootstrap 전)
    assert result.transport == [] or result.transport is not None
    assert result.layer.state == "WAITING_INPUTS"

    enter_running(result.layer)  # RunControlCommand(START) 상당
    ws = by_channel(result.transport, "world_state")
    assert len(ws) == 1 and ws[0].state_tick_id == 0
    result.backend.shutdown()


def test_unsupported_simulator_or_rate_rejected():
    """2) 지원하지 않는 simulator/rate -> READY_REJECTED + MANIFEST_MISMATCH."""
    # 2a) 이 컴포넌트가 지원 안 하는 simulator_type (create_simulator_adapter가 모르는 타입)
    bad_sim = _manifest(adapter_type=m.NativeAdapterType.MORAI)
    result = start_run(bad_sim, SIM_ID)
    assert result.ready.ready_status == m.ReadyStatus.READY_REJECTED
    assert result.ready.reason_codes == [m.MANIFEST_MISMATCH]
    assert result.layer is None and result.backend is None

    # 2b) mock capability 범위(1~1000Hz) 밖의 rate — fixed_step_ns를 극단적으로 크게(0.1Hz)
    bad_rate = _manifest(fixed_step_ns=10_000_000_000)
    result2 = start_run(bad_rate, SIM_ID)
    assert result2.ready.ready_status == m.ReadyStatus.READY_REJECTED
    assert result2.ready.reason_codes == [m.MANIFEST_MISMATCH]
    assert result2.layer is None and result2.backend is None


def test_run_transport_construction_failure_rejected():
    """3) Run Transport 생성 실패 -> READY_REJECTED + TRANSPORT_ERROR.

    Manifest 자체(simulator_type/rate)는 완전히 유효하고 capability도 통과하지만,
    Run Transport(엔드포인트) 구성 단계에서만 인위적으로 실패시켜 —
    MANIFEST_MISMATCH가 아니라 TRANSPORT_ERROR로 분류되는지 확인한다.
    """
    def failing_run_transport_factory(*args, **kwargs):
        raise RuntimeError("simulated create_publisher() failure")

    manifest = _manifest()
    result = start_run(manifest, SIM_ID, run_transport_factory=failing_run_transport_factory)

    assert result.ready.ready_status == m.ReadyStatus.READY_REJECTED
    assert result.ready.reason_codes == [m.TRANSPORT_ERROR]
    assert result.layer is None and result.backend is None


def test_advance_frame_before_running_does_nothing():
    """4) READY 전에 AdvanceFrame -> native step 0.

    start_run()으로 ComponentReady(READY)까지는 갔지만 enter_running()(=RUNNING
    진입)을 아직 안 부른 상태 — 이 시점에 AdvanceFrame이 와도 gate 상태가 READY가
    아니므로(§8.2, 초기 상태 WAITING_INPUTS) native step이 0에 머물러야 한다.
    """
    manifest = _manifest()
    result = start_run(manifest, SIM_ID)
    assert result.layer.state == "WAITING_INPUTS"  # enter_running() 호출 전
    assert result.backend.native_frame_id() == "0"

    fake_advance = m.AdvanceFrame(
        header=None, decision_id=uuid.uuid4().bytes, target_tick_id=1,
        snapshot_hash=b"\x11" * 32, action=m.AdvanceAction.ADVANCE, abort_reason_code=m.UNKNOWN_REASON)
    fake_advance.header = m.CommonHeader(
        schema_major=1, schema_minor=0, run_id=RUN_ID, run_epoch=EPOCH, scope_kind=m.ScopeKind.SIM,
        sim_id=m.OptionalBoundedId(True, SIM_ID), ego_id=m.OptionalBoundedId(False, ""),
        producer_id=m.ComponentId.CORE, producer_instance_id=b"\x03" * 16, event_seq=1,
        correlation_id=m.OptionalUuid128(False, b"\x00" * 16), sim_time_ns=m.OptionalUint64(True, 0),
        wall_time_unix_ns=0, payload_hash=hashing.advance_frame_hash(fake_advance))

    result.layer.on_advance_frame(fake_advance)
    assert result.backend.native_frame_id() == "0", "READY(RUNNING) 전인데 native step이 발생함"
    assert result.layer.events[-1]["reason_code"] == m.INVALID_STATE
    result.backend.shutdown()


TESTS = [
    (test_normal_manifest_reaches_running,
     "정상 Manifest -> RunContext 구성 -> simulator 선택 -> Run Transport 구성 -> "
     "ComponentReady(READY) -> enter_running()으로 RUNNING 진입, 최초 관측 발행까지 확인"),
    (test_unsupported_simulator_or_rate_rejected,
     "지원 안 하는 simulator_type(MORAI) / 지원 범위 밖 rate 각각 READY_REJECTED+MANIFEST_MISMATCH, "
     "candidate backend/layer 전부 폐기(None)"),
    (test_run_transport_construction_failure_rejected,
     "Manifest·capability는 유효하지만 Run Transport 구성 자체가 실패 -> READY_REJECTED+TRANSPORT_ERROR "
     "(MANIFEST_MISMATCH와 구분됨)"),
    (test_advance_frame_before_running_does_nothing,
     "ComponentReady(READY) 이후, enter_running() 전에 AdvanceFrame이 와도 native step 0 유지, "
     "INVALID_STATE로 거부"),
]

if __name__ == "__main__":
    for i, (fn, desc) in enumerate(TESTS, 1):
        fn()
        print(f"[{i}/{len(TESTS)}] PASS {fn.__name__}\n         검증: {desc}")
    print(f"\n전체 {len(TESTS)}개 startup 계약 시험 PASS")
