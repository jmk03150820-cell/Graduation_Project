"""Mock Core/Ego/Traffic 기준 정상 경로 1 frame + 중복/충돌 방어 계약 시험.

Runs with plain `python -m simulation_layer.test_frame_lifecycle` (assert-based)
or with pytest.
"""
from __future__ import annotations

import time
import uuid

import simulation_layer  # noqa: F401  (interfaces path shim)
import avva_phase1 as m
from simulation_layer import hashing
from simulation_layer.backend import RunConfig
from simulation_layer.gate import RegistryEntry, SimulationLayer
from simulation_layer.mock_backend import MockSimulatorAdapter

RUN_ID = uuid.uuid4().bytes
EPOCH = 1
SIM_ID = "mock_0"
EGO_ID = "ego_0"
RUN_CONFIG = RunConfig(target_rate_hz=20.0)  # fixed delta는 rate에서 유도, 하드코딩 금지
STEP_NS = RUN_CONFIG.fixed_step_ns

_NO_ID = m.OptionalBoundedId(False, "")
_NO_NC = m.OptionalNeutralControl(False, None)


def _header(producer, scope, payload_hash, sim_time, ego_id=None, correlation=None, seq=[0]):
    seq[0] += 1
    return m.CommonHeader(
        schema_major=1, schema_minor=0, run_id=RUN_ID, run_epoch=EPOCH, scope_kind=scope,
        sim_id=m.OptionalBoundedId(True, SIM_ID),
        ego_id=m.OptionalBoundedId(True, ego_id) if ego_id else _NO_ID,
        producer_id=producer, producer_instance_id=b"\x01" * 16, event_seq=seq[0],
        correlation_id=m.OptionalUuid128(True, correlation) if correlation else m.OptionalUuid128(False, b"\x00" * 16),
        sim_time_ns=m.OptionalUint64(True, sim_time),
        wall_time_unix_ns=time.time_ns(), payload_hash=payload_hash)


def make_layer():
    registry = [
        RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
        RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
        RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.TRUCK),
    ]
    backend = make_mock_adapter(registry)
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, SIM_ID, registry, RUN_CONFIG,
                            publish=lambda ch, msg: published.append((ch, msg)))
    return layer, backend, published


def make_mock_adapter(registry) -> MockSimulatorAdapter:
    adapter = MockSimulatorAdapter()
    adapter.initialize()
    adapter.configure(RUN_CONFIG)
    for e in registry:
        adapter.spawn_actor(e.actor_id)
    return adapter


def mock_ego_command(obs: m.EgoObservationFrame, throttle=0.3) -> m.EgoControlCommand:
    ctrl = m.NeutralControl(
        control_mode=m.ControlMode.DIRECT_ACTUATION,
        valid_fields_mask=(1 << 0) | (1 << 5) | (1 << 6),
        steering_tire_angle_rad=0.0, steering_tire_rotation_rate_rad_s=0.0,
        velocity_mps=0.0, acceleration_mps2=0.0, jerk_mps3=0.0,
        throttle=throttle, brake=0.0, gear=m.Gear.GEAR_UNKNOWN, hand_brake=False)
    cmd = m.EgoControlCommand(
        header=None, based_on_tick_id=obs.state_tick_id, target_tick_id=obs.state_tick_id + 1,
        source_stack=m.SourceStack.MODULE_CHAIN, control_time_sim_ns=obs.state_tick_id * STEP_NS,
        command_status=m.CommandStatus.COMMAND_OK,
        control=m.OptionalNeutralControl(True, ctrl), failure_reason=m.UNKNOWN_REASON)
    cmd.header = _header(m.ComponentId.EGO_ADAPTER, m.ScopeKind.EGO,
                         hashing.ego_control_command_hash(cmd), obs.state_tick_id * STEP_NS,
                         ego_id=EGO_ID, correlation=obs.header.correlation_id.value)
    return cmd


def mock_npc_batch(ws: m.WorldStateFrame) -> m.NpcControlBatch:
    apply_ctrl = m.NeutralControl(
        control_mode=m.ControlMode.VELOCITY_TARGET, valid_fields_mask=(1 << 0) | (1 << 2),
        steering_tire_angle_rad=0.0, steering_tire_rotation_rate_rad_s=0.0,
        velocity_mps=5.0, acceleration_mps2=0.0, jerk_mps3=0.0,
        throttle=0.0, brake=0.0, gear=m.Gear.GEAR_UNKNOWN, hand_brake=False)
    items = [  # actor_id 정렬, expected set과 1:1 (§6.7)
        m.NpcControlItem("npc_1", m.CommandAction.APPLY, m.ItemStatus.ITEM_OK,
                         m.OptionalNeutralControl(True, apply_ctrl), m.UNKNOWN_REASON),
        m.NpcControlItem("npc_2", m.CommandAction.HOLD, m.ItemStatus.ITEM_OK, _NO_NC, m.UNKNOWN_REASON),
    ]
    batch = m.NpcControlBatch(
        header=None, based_on_tick_id=ws.state_tick_id, target_tick_id=ws.state_tick_id + 1,
        control_set_meta=ws.control_set_meta, items=items, lifecycle_intents=[],
        engine_step_id=ws.state_tick_id + 1, engine_sim_time_ns=(ws.state_tick_id + 1) * STEP_NS,
        engine_step_duration_ns=1_000_000)
    batch.header = _header(m.ComponentId.TRAFFIC_ADAPTER, m.ScopeKind.SIM,
                           hashing.npc_control_batch_hash(batch), ws.state_tick_id * STEP_NS,
                           correlation=ws.header.correlation_id.value)
    return batch


def mock_advance(ready: m.CommandSetReady, decision_id: bytes) -> m.AdvanceFrame:
    adv = m.AdvanceFrame(header=None, decision_id=decision_id,
                         target_tick_id=ready.target_tick_id, snapshot_hash=ready.snapshot_hash,
                         action=m.AdvanceAction.ADVANCE, abort_reason_code=m.UNKNOWN_REASON)
    adv.header = _header(m.ComponentId.CORE, m.ScopeKind.SIM,
                         hashing.advance_frame_hash(adv), ready.header.sim_time_ns.value)
    return adv


def mock_ack(fc: m.FrameComplete) -> m.FrameCompleteAck:
    ack = m.FrameCompleteAck(header=None, decision_id=fc.decision_id, state_tick_id=fc.state_tick_id,
                             applied_snapshot_hash=fc.applied_snapshot_hash,
                             ack_status=m.AckStatus.ACCEPTED, reason_code=m.UNKNOWN_REASON)
    ack.header = _header(m.ComponentId.CORE, m.ScopeKind.SIM,
                         hashing.frame_complete_ack_hash(ack), fc.header.sim_time_ns.value)
    return ack


def by_channel(published, suffix):
    return [msg for ch, msg in published if ch.endswith(suffix)]


def test_normal_path_one_frame():
    layer, backend, published = make_layer()

    # bootstrap: seed 관측 k=0 (§7.3)
    layer.bootstrap()
    ws0 = by_channel(published, "world_state")[0]
    obs0 = by_channel(published, "observation")[0]
    assert ws0.state_tick_id == 0 and obs0.state_tick_id == 0
    assert ws0.expected_traffic_actor_ids == ["npc_1", "npc_2"]
    assert [a.actor_id for a in ws0.actors] == sorted(a.actor_id for a in ws0.actors)

    # Ego/Traffic → target 1 명령, 도착 순서 무관
    layer.on_npc_batch(mock_npc_batch(ws0))
    assert layer.state == "COLLECTING"
    assert by_channel(published, "command_ready") == []
    layer.on_ego_control(mock_ego_command(obs0))

    # READY: CommandSetReady 1회, FULL+VALID, snapshot hash 재계산 일치
    readies = by_channel(published, "command_ready")
    assert len(readies) == 1 and layer.state == "READY"
    ready = readies[0]
    assert ready.completeness == m.Completeness.FULL and ready.validity == m.Validity.VALID
    assert ready.counts.valid_ego == 1 and ready.counts.valid_traffic == 2
    assert ready.header.payload_hash == hashing.command_set_ready_hash(ready)

    # Core 승인 → apply + native step 정확히 1회 → state 선발행, FrameComplete 후발행
    decision = uuid.uuid4().bytes
    layer.on_advance_frame(mock_advance(ready, decision))
    assert int(backend.native_frame_id()) == 1
    ws1 = by_channel(published, "world_state")[1]
    completes = by_channel(published, "frame_complete")
    assert len(completes) == 1
    fc = completes[0]
    assert fc.state_tick_id == 1 and fc.decision_id == decision
    assert fc.applied_snapshot_hash == ready.snapshot_hash
    channels_after_ready = [ch for ch, _ in published[published.index(("avva/v1/sim/mock_0/command_ready", ready)) + 1:]]
    assert channels_after_ready.index("avva/v1/sim/mock_0/world_state") \
         < channels_after_ready.index("avva/v1/sim/mock_0/frame_complete"), "state must publish first (§7.2)"

    # 물리 반영 확인: npc_1 velocity target 5.0 적용
    npc1 = next(a for a in ws1.actors if a.actor_id == "npc_1")
    assert npc1.velocity_mps.x == 5.0 and npc1.position_m.x > 0.0

    # 동일 decision_id 재수신 → 재실행 없이 동일 FrameComplete 재전송 (방어 1)
    layer.on_advance_frame(mock_advance(ready, decision))
    assert int(backend.native_frame_id()) == 1, "duplicate AdvanceFrame must not re-tick"
    assert len(by_channel(published, "frame_complete")) == 2
    assert by_channel(published, "frame_complete")[1] is fc
    assert len(by_channel(published, "world_state")) == 2, "no re-publish of state"

    # 새 decision_id지만 상태가 PUBLISHING → state gate가 거부 (방어 2)
    layer.on_advance_frame(mock_advance(ready, uuid.uuid4().bytes))
    assert int(backend.native_frame_id()) == 1
    assert any(e["event"] == "REJECT" and e["reason_code"] == m.INVALID_STATE for e in layer.events)

    # FrameCompleteAck(ACCEPTED) → 다음 frame으로
    layer.on_frame_complete_ack(mock_ack(fc))
    assert layer.state == "WAITING_INPUTS" and layer.state_tick == 1


def test_duplicate_and_conflict_inputs():
    layer, backend, published = make_layer()
    layer.bootstrap()
    ws0 = by_channel(published, "world_state")[0]
    obs0 = by_channel(published, "observation")[0]

    cmd = mock_ego_command(obs0)
    layer.on_ego_control(cmd)
    # 동일 key + 동일 hash → idempotent duplicate, side effect 없음
    layer.on_ego_control(cmd)
    assert layer.state == "COLLECTING"
    assert any(e["reason_code"] == m.DUPLICATE_IDEMPOTENT for e in layer.events)

    # 동일 key + 다른 hash → PAYLOAD_CONFLICT, frame ABORTED (§8.4)
    layer.on_ego_control(mock_ego_command(obs0, throttle=0.9))
    assert layer.state == "ABORTED"
    assert layer.events[-1]["reason_code"] == m.PAYLOAD_CONFLICT

    # ABORTED 이후에는 어떤 입력도 진행시키지 못한다
    layer.on_npc_batch(mock_npc_batch(ws0))
    assert by_channel(published, "command_ready") == []
    assert int(backend.native_frame_id()) == 0


def test_tick_relation_rejects():
    layer, _, published = make_layer()
    layer.bootstrap()
    obs0 = by_channel(published, "observation")[0]

    stale = mock_ego_command(obs0)
    stale.based_on_tick_id, stale.target_tick_id = 5, 6  # 미래 tick
    stale.header.payload_hash = hashing.ego_control_command_hash(stale)
    layer.on_ego_control(stale)
    assert layer.ego_slot is None
    assert layer.events[-1]["reason_code"] == m.FUTURE_TICK

    bad = mock_ego_command(obs0)
    bad.target_tick_id = bad.based_on_tick_id + 2  # target != based_on+1
    bad.header.payload_hash = hashing.ego_control_command_hash(bad)
    layer.on_ego_control(bad)
    assert layer.events[-1]["reason_code"] == m.TICK_RELATION_INVALID


def test_actor_reasons_sorted_by_actor_id():
    """§6.8 "실제 정렬 report의 앞 32개" — item 순회 실패분과 missing분이 섞여도
    actor_reasons 전체가 actor_id로 정렬돼야 truncation이 올바른 32개를 자른다."""
    registry = [
        RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
        RegistryEntry("npc_a", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
        RegistryEntry("npc_m", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
        RegistryEntry("npc_z", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
    ]
    backend = make_mock_adapter(registry)
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, SIM_ID, registry, RUN_CONFIG,
                            publish=lambda ch, msg: published.append((ch, msg)))
    layer.bootstrap()
    ws0 = by_channel(published, "world_state")[0]
    obs0 = by_channel(published, "observation")[0]

    # npc_z만 FAILED item 제출(item-loop에서 먼저 실패로 잡힘), npc_a는 아예 미제출(missing,
    # 나중에 정렬돼 붙음) — 정렬 안 하면 [npc_z, npc_a] 순서(z가 a보다 앞)로 나가는 버그가 재현됨
    items = [m.NpcControlItem("npc_z", m.CommandAction.COMMAND_ACTION_UNSPECIFIED,
                              m.ItemStatus.ITEM_FAILED, _NO_NC, m.COMPUTE_FAILED)]
    batch = m.NpcControlBatch(
        header=None, based_on_tick_id=0, target_tick_id=1, control_set_meta=ws0.control_set_meta,
        items=items, lifecycle_intents=[], engine_step_id=1, engine_sim_time_ns=STEP_NS,
        engine_step_duration_ns=1_000_000)
    batch.header = _header(m.ComponentId.TRAFFIC_ADAPTER, m.ScopeKind.SIM,
                           hashing.npc_control_batch_hash(batch), 0, correlation=ws0.header.correlation_id.value)

    layer.on_npc_batch(batch)
    layer.on_ego_control(mock_ego_command(obs0))
    ready = by_channel(published, "command_ready")[0]

    ids = [r.actor_id for r in ready.actor_reasons]
    assert ids == sorted(ids), f"actor_reasons not sorted by actor_id: {ids}"
    assert ids == ["npc_a", "npc_m", "npc_z"]  # a=missing, m=missing, z=failed — 전부 섞여 정렬


def test_advance_frame_rejects_unspecified_action():
    """AdvanceAction.ADVANCE가 아니면(ABORT도 아닌 UNSPECIFIED 등) 거부해야지 진행시키면 안 됨."""
    layer, backend, published = make_layer()
    layer.bootstrap()
    ws0 = by_channel(published, "world_state")[0]
    obs0 = by_channel(published, "observation")[0]
    layer.on_ego_control(mock_ego_command(obs0))
    layer.on_npc_batch(mock_npc_batch(ws0))
    ready = by_channel(published, "command_ready")[0]

    bad_adv = mock_advance(ready, uuid.uuid4().bytes)
    bad_adv.action = m.AdvanceAction.ADVANCE_UNSPECIFIED
    bad_adv.header.payload_hash = hashing.advance_frame_hash(bad_adv)

    layer.on_advance_frame(bad_adv)
    assert layer.state == "READY", "UNSPECIFIED action이 승인으로 취급되면 안 됨"
    assert int(backend.native_frame_id()) == 0
    assert layer.events[-1]["reason_code"] == m.OUT_OF_RANGE


def test_capability_and_config_gates():
    """capability fail-fast(§2-7)와 execution policy 경계: 실행 전에 실패해야 한다."""
    import dataclasses

    # async execution은 Phase 1에서 gate 진입 자체가 거부됨
    async_cfg = RunConfig(target_rate_hz=20.0, execution_mode=m.ExecutionMode.EXECUTION_UNSPECIFIED)
    try:
        SimulationLayer(MockSimulatorAdapter(), RUN_ID, EPOCH, SIM_ID, [], async_cfg,
                        publish=lambda ch, msg: None)
        assert False, "non-sync execution mode must be rejected"
    except NotImplementedError:
        pass

    # target_rate가 capability 범위 밖이면 configure에서 거부
    try:
        a = MockSimulatorAdapter()
        a.initialize()
        a.configure(RunConfig(target_rate_hz=999999.0))
        assert False, "over-capability rate must be rejected"
    except ValueError:
        pass

    # 시나리오 NPC 수 > max_npc면 bootstrap(실행 전)에서 거부
    class TinyAdapter(MockSimulatorAdapter):
        def capabilities(self):
            return dataclasses.replace(super().capabilities(), configured_max_npc=1)

    registry = [
        RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
        RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
    ]
    tiny = TinyAdapter()
    tiny.initialize()
    tiny.configure(RUN_CONFIG)
    for e in registry:
        tiny.spawn_actor(e.actor_id)
    layer = SimulationLayer(tiny, RUN_ID, EPOCH, SIM_ID, registry, RUN_CONFIG,
                            publish=lambda ch, msg: None)
    try:
        layer.bootstrap()
        assert False, "2 NPCs on max_npc=1 must fail before running"
    except ValueError:
        pass


def test_encoder_provenance_and_swap_seam():
    """capability_hash 아키텍처가 실제로 common/layer 분리인지 코드로 증명
    (오늘 아침 공지: "필드 순서/width/enum/string/list 정렬 등 canonical byte
    encoding 규칙은 common capability encoder가 담당, Layer는 실제 값만 구성").

    1) canonical byte encoding 전체(CAPABILITY_PREFIX, 필드 순서/폭/enum/bool/
       list 정렬을 다 아는 simulator_capability_bytes())가 Simulator Layer
       사본이 아니라 common 계약 번들(avva_hash_v1.py)에 정의돼 있는지 —
       동일성(is) 비교 + 실제 파일 경로 확인
    2) backend.py 어디에도 인코딩 로직(u8/u16/u32/f64/sequence 호출)이 남아있지
       않은지 — 있으면 "값만 구성"이 아니라 여전히 규칙을 쥐고 있다는 뜻
    3) common의 simulator_capability_bytes()를 바꿔치기하면 backend.py의 digest
       파이프라인이 재배포 없이 자동으로 그걸 타는지 — "규칙은 common 소유,
       layer는 값 전달만" 증명
    """
    import pathlib
    import avva_hash_v1
    from simulation_layer import backend

    # 1) 인코딩 규칙 출처: simulator_capability_bytes는 common에만 있고
    #    backend가 쓰는 것도 바로 그 common 함수(사본 아님)
    common_path = pathlib.Path(avva_hash_v1.__file__).resolve()
    assert "interfaces" in common_path.parts and "generated" in common_path.parts, \
        f"avva_hash_v1이 계약 번들 밖에서 로드됨: {common_path}"
    assert backend.simulator_capability_bytes is avva_hash_v1.simulator_capability_bytes, \
        "backend가 common simulator_capability_bytes가 아닌 사본을 씀"
    assert backend.sha256 is avva_hash_v1.sha256

    # 2) backend.py에 인코딩 규칙(u8/u16/u32/f64/bool8/sequence 직접 호출)이
    #    남아있지 않은지 — "값만 구성"이라면 CanonicalWriter 메서드를 직접
    #    호출할 일이 없다
    pkg = pathlib.Path(backend.__file__).parent
    backend_src = (pkg / "backend.py").read_text(encoding="utf-8")
    for method in (".u8(", ".u16(", ".u32(", ".u64(", ".f64(", ".bool8(", ".sequence("):
        assert method not in backend_src, \
            f"backend.py가 여전히 CanonicalWriter 인코딩({method!r})을 직접 호출함 — " \
            "규칙이 common으로 완전히 안 옮겨짐"
    for py in pkg.glob("*.py"):
        for line in py.read_text(encoding="utf-8").splitlines():
            assert not line.startswith("CAPABILITY_PREFIX ="), \
                f"{py.name}: domain prefix가 layer 쪽에 재정의됨 — common(avva_hash_v1) 소유여야 함"
            assert not line.startswith("def simulator_capability_bytes"), \
                f"{py.name}: 필드 인코딩 규칙이 layer 쪽에 재정의됨 — common 소유여야 함"

    # 3) 교체 seam: common의 simulator_capability_bytes()를 '팀이 배포한 새
    #    common encoder'로 바꿔치기하면 backend.py의 digest가 재배포 없이
    #    자동으로 그걸 타야 함
    cap = MockSimulatorAdapter().capabilities()
    original = backend.simulator_capability_bytes
    calls: list = []

    def fake_common_encoder(**kwargs):
        calls.append(kwargs)
        return original(**kwargs)

    backend.simulator_capability_bytes = fake_common_encoder
    try:
        d = backend.capability_digest(cap)
        assert len(calls) == 1, "digest가 common simulator_capability_bytes() 교체 seam을 거치지 않음"
        assert calls[0]["simulator_type"] == int(cap.simulator_type)  # 값 전달 확인
    finally:
        backend.simulator_capability_bytes = original
    assert backend.capability_digest(cap) == d  # 원복 후에도 동일 digest


def test_new_run_starts_clean_and_rejects_old_epoch():
    """reset/new Run 정책 (§8.5): 새 RunKey = 새 SimulationLayer 인스턴스.

    snapshot/dedup/decision_cache/ticked는 전부 인스턴스 필드라 새 run에서
    구조적으로 빈 상태로 시작하고(같은 epoch 재사용 API는 의도적으로 없음 —
    same-epoch resume 금지), 이전 epoch의 메시지는 EPOCH_MISMATCH로 거부된다.
    """
    # run 1 (epoch 1): frame 1개를 끝까지 진행해 캐시를 채운다
    layer1, backend, published1 = make_layer()
    layer1.bootstrap()
    ws0 = by_channel(published1, "world_state")[0]
    obs0 = by_channel(published1, "observation")[0]
    old_ego_cmd = mock_ego_command(obs0)
    layer1.on_ego_control(old_ego_cmd)
    layer1.on_npc_batch(mock_npc_batch(ws0))
    ready = by_channel(published1, "command_ready")[0]
    decision = uuid.uuid4().bytes
    layer1.on_advance_frame(mock_advance(ready, decision))
    layer1.on_frame_complete_ack(mock_ack(by_channel(published1, "frame_complete")[0]))
    assert layer1.decision_cache and layer1.ticked, "run 1 캐시가 채워져 있어야 전제 성립"

    # run 2 (새 epoch): 같은 backend를 재사용해도 새 인스턴스는 깨끗하게 시작
    backend.reset()
    published2: list[tuple[str, object]] = []
    registry = list(layer1.registry.values())
    layer2 = SimulationLayer(backend, RUN_ID, EPOCH + 1, SIM_ID, registry, RUN_CONFIG,
                             publish=lambda ch, msg: published2.append((ch, msg)))
    assert not layer2.decision_cache and not layer2.ticked and layer2.snapshot is None
    assert layer2.ego_slot is None and layer2.npc_slot is None and layer2.state_tick == 0
    layer2.bootstrap()
    assert by_channel(published2, "world_state")[0].state_tick_id == 0

    # 이전 epoch(1)의 메시지가 새 run(epoch 2)에 오면 EPOCH_MISMATCH 거부 (§8.4)
    layer2.on_ego_control(old_ego_cmd)
    assert layer2.ego_slot is None
    assert layer2.events[-1]["reason_code"] == m.EPOCH_MISMATCH


def test_capability_digest_rule():
    """capability_digest_rule.md 6개 케이스: 결정성 / 필드 민감성 / runtime metric
    미포함 / 무순서 list 정렬 / max_npc vs schema capacity 구분 / 공통 encoding 규칙."""
    import dataclasses
    import struct
    from avva_hash_v1 import CAPABILITY_PREFIX  # common — capability_digest_rule.md §3
    from simulation_layer.backend import capability_canonical_bytes, capability_digest

    cap = MockSimulatorAdapter().capabilities()

    # 1) 동일 capability를 여러 번 hash -> 항상 같은 digest (identity/consistency)
    digests = {capability_digest(dataclasses.replace(cap)) for _ in range(100)}
    assert len(digests) == 1 and len(digests.pop()) == 32  # SHA-256, 100회 전부 동일

    # 2) field 하나 변경 -> digest 변경 (모든 필드에 대해 확인)
    base = capability_digest(cap)
    changed = [
        dataclasses.replace(cap, component_type=m.ComponentId.CORE),
        dataclasses.replace(cap, simulator_type=m.NativeAdapterType.CARLA),
        dataclasses.replace(cap, max_ego=cap.max_ego + 1),
        dataclasses.replace(cap, schema_max_npc=cap.schema_max_npc + 1),
        dataclasses.replace(cap, configured_max_npc=cap.configured_max_npc + 1),
        dataclasses.replace(cap, validated_max_npc=1),
        dataclasses.replace(cap, supports_sync=not cap.supports_sync),
        dataclasses.replace(cap, supports_async=not cap.supports_async),
        dataclasses.replace(cap, supported_rate_min_hz=cap.supported_rate_min_hz + 1.0),
        dataclasses.replace(cap, supported_rate_max_hz=cap.supported_rate_max_hz + 1.0),
        dataclasses.replace(cap, supported_control_modes=(m.ControlMode.DIRECT_ACTUATION,)),
    ]
    variants = [capability_digest(c) for c in changed]
    assert base not in variants and len(set(variants)) == len(variants), \
        "어떤 필드를 바꿔도 digest가 달라져야 하고 서로 충돌도 없어야 함"

    # 3) runtime metric 변경 -> digest 불변 (§4: 현재 NPC 수/tick/FPS 등은 capability 아님)
    adapter = MockSimulatorAdapter()
    d_before = capability_digest(adapter.capabilities())
    adapter.initialize()
    adapter.configure(RUN_CONFIG)
    for i in range(5):
        adapter.spawn_actor(f"npc_{i}")  # 현재 NPC 수 변화
    adapter.reset()
    for _ in range(7):
        adapter.tick()                   # 현재 tick/native frame 변화
    assert capability_digest(adapter.capabilities()) == d_before, \
        "runtime 상태(NPC 수/tick)가 digest에 새어 들어감"
    runtime_metric_names = {"current_npc", "current_tick", "cpu", "gpu", "latency", "fps"}
    assert not runtime_metric_names & {f.name for f in dataclasses.fields(cap)}, \
        "runtime metric은 capability struct에 두지 않는다"

    # 4) 순서 의미 없는 list의 순서만 변경 -> digest 불변 (정렬 후 encoding 규칙)
    shuffled = dataclasses.replace(cap, supported_control_modes=tuple(
        reversed(cap.supported_control_modes)))
    assert shuffled.supported_control_modes != cap.supported_control_modes  # 순서는 실제로 다름
    assert capability_digest(shuffled) == base, "무순서 list는 정렬 후 hash되어야 함"

    # 5) max_npc는 schema capacity와 구분 (§2)
    assert cap.configured_max_npc < cap.schema_max_npc == 1200, \
        "schema capacity(1200)를 max_npc로 선언하면 안 됨"
    assert cap.validated_max_npc == 0 and cap.effective_max_npc == cap.configured_max_npc
    assert dataclasses.replace(cap, validated_max_npc=150).effective_max_npc == 150

    # 6) enum/field order/endian/bool/list ordering이 공통 규칙(§4.6)을 따르는지 —
    #    기대 byte stream을 struct.pack으로 독립 재구성해 바이트 단위 대조
    expected = (CAPABILITY_PREFIX
                + struct.pack("<B", int(cap.component_type))      # enum = numeric u8
                + struct.pack("<B", int(cap.simulator_type))
                + struct.pack("<H", cap.max_ego)                  # 고정폭 little-endian
                + struct.pack("<I", cap.schema_max_npc)
                + struct.pack("<I", cap.configured_max_npc)
                + struct.pack("<I", cap.validated_max_npc)
                + struct.pack("<B", 1 if cap.supports_sync else 0)   # bool = 1 byte
                + struct.pack("<B", 1 if cap.supports_async else 0)
                + struct.pack("<d", cap.supported_rate_min_hz)       # f64 LE
                + struct.pack("<d", cap.supported_rate_max_hz)
                + struct.pack("<I", len(cap.supported_control_modes))  # u32 count
                + bytes(sorted(int(v) for v in cap.supported_control_modes)))  # 정렬된 원소
    assert capability_canonical_bytes(cap) == expected, \
        "canonical byte stream이 공통 encoding 규칙과 다름"


def test_import_boundaries():
    """경계 규칙 자동 강제 (§2-7 + 과제 8 transport 분리 + MetaDrive 2nd backend):
    - `import carla`는 carla_backend.py에만
    - `import metadrive`는 metadrive_backend.py에만 (Sim backend는 어느 native
      시뮬레이터 API도 직접 사용 금지 — 과제 5 selectable execution의 전제)
    - `import rclpy`/`avva_interfaces`는 ROS 2 binding 파일에만 — ROS 2를 떼어내도
      Sim backend(gate/backend/adapter/hashing/transforms)는 수정 0
    """
    import pathlib
    native_only = {"carla": "carla_backend.py", "metadrive": "metadrive_backend.py"}
    ros2_binding_files = {"ros2_node.py", "ros2_convert.py",
                          "demo_ros2_lifecycle.py", "test_ros2_convert.py"}
    pkg = pathlib.Path(__file__).parent
    for py in pkg.glob("*.py"):
        for line in py.read_text(encoding="utf-8").splitlines():
            code = line.split("#")[0].strip()
            for mod, owner in native_only.items():
                if py.name != owner:
                    assert not (code.startswith(f"import {mod}") or code.startswith(f"from {mod}")), \
                        f"{py.name}: {mod} import leaked outside {owner}"
            if py.name not in ros2_binding_files:
                for mod in ("rclpy", "avva_interfaces"):
                    assert not (code.startswith(f"import {mod}") or code.startswith(f"from {mod}")), \
                        f"{py.name}: {mod} import leaked outside the ROS 2 binding layer"


TESTS = [
    (test_normal_path_one_frame,
     "정상 경로 1 frame: 입력 2종 -> READY -> AdvanceFrame -> native step 1회 -> "
     "state 선발행/FrameComplete 후발행 -> 중복 승인 시 재tick 없음 -> ACK로 다음 frame"),
    (test_duplicate_and_conflict_inputs,
     "동일 key+동일 hash = DUPLICATE_IDEMPOTENT(진행 계속) / "
     "동일 key+다른 hash = PAYLOAD_CONFLICT(ABORTED, 이후 어떤 입력도 진행 불가)"),
    (test_tick_relation_rejects,
     "미래 tick = FUTURE_TICK 거부 / target != based_on+1 = TICK_RELATION_INVALID 거부"),
    (test_actor_reasons_sorted_by_actor_id,
     "CommandSetReady.actor_reasons가 actor_id 정렬(§6.8) — FAILED item과 missing이 섞여도"),
    (test_advance_frame_rejects_unspecified_action,
     "AdvanceAction이 ADVANCE가 아니면(UNSPECIFIED 등) 거부, native step 0회 유지"),
    (test_capability_and_config_gates,
     "async 실행 거부(Phase 1 sync only) / rate 초과 configure 거부 / "
     "NPC 수 > max_npc면 bootstrap(실행 전)에서 실패"),
    (test_capability_digest_rule,
     "digest 6종: 100회 반복 동일 / 전 필드 각각 변경 시 상이 / runtime metric 무영향 / "
     "무순서 list 순서 무영향(정렬 encoding) / max_npc vs schema capacity 구분 / "
     "canonical bytes를 struct.pack으로 독립 재구성해 §4.6 규칙 바이트 대조"),
    (test_encoder_provenance_and_swap_seam,
     "capability_hash 아키텍처: 필드 순서/폭/enum/bool/list 정렬까지 전부 common "
     "simulator_capability_bytes()가 소유(is 비교+경로), backend.py엔 CanonicalWriter 인코딩 "
     "호출이 하나도 없음(값만 전달) / common encoder를 바꿔치기하면 digest가 자동 위임"),
    (test_new_run_starts_clean_and_rejects_old_epoch,
     "새 Run(새 epoch) = 새 인스턴스로 snapshot/dedup/decision_cache/ticked 전부 초기화 / "
     "이전 epoch 메시지는 EPOCH_MISMATCH 거부"),
    (test_import_boundaries,
     "import carla는 carla_backend.py에만 / import metadrive는 metadrive_backend.py에만 / "
     "rclpy·avva_interfaces는 ROS2 binding 파일에만"),
]


def _mutation_check() -> None:
    """불량 주입 자가검증: 구현을 일부러 망가뜨렸을 때 테스트가 실제로 잡는지.

    MockSimulatorAdapter.tick을 '한 번 호출에 native step 2회'로 바꿔서
    exactly-one-tick 보장을 깨뜨린다. 정상 suite가 이걸 통과시키면 그 suite는
    허수라는 뜻 — 반드시 AssertionError로 검출돼야 한다.
    """
    original_tick = MockSimulatorAdapter.tick

    def double_tick(self):  # 불량: native step이 두 번 실행되는 결함
        original_tick(self)
        original_tick(self)

    MockSimulatorAdapter.tick = double_tick
    try:
        test_normal_path_one_frame()
        raise SystemExit("불량(이중 tick)을 주입했는데 테스트가 통과함 — suite가 허수!")
    except AssertionError:
        import traceback
        frame = traceback.extract_tb(__import__("sys").exc_info()[2])[-1]
        print(f"[불량 주입] MockSimulatorAdapter.tick을 2회 실행되게 조작"
              f" -> 테스트가 즉시 검출:")
        print(f"           {frame.name} line {frame.lineno}: {frame.line}")
        print("[불량 주입] 검출 성공 — suite는 이중 native step 결함을 놓치지 않음")
    finally:
        MockSimulatorAdapter.tick = original_tick


if __name__ == "__main__":
    for i, (fn, desc) in enumerate(TESTS, 1):
        fn()
        print(f"[{i}/{len(TESTS)}] PASS {fn.__name__}\n         검증: {desc}")
    print()
    _mutation_check()
    print(f"\n전체 {len(TESTS)}개 계약 시험 PASS + 불량 주입 검출 확인")
