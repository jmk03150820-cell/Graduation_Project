"""Bootstrap Transport 수신 이후의 기동 시퀀스 — 1번 체크리스트에서 빠졌던
3단계(Bootstrap Transport 수신/RunManifest 적용/Run Transport 준비)를 채운다.

RunManifest 수신 → RunContext 구성/검증 → simulator_type 선택 →
execution_mode/target_rate_hz 적용 → Run Transport 구성(candidate) →
mandatory endpoint READY 확인 → SimulatorCapability 확정 → ComponentReady →
RUNNING.

Bootstrap Transport 자체(어떻게 RunManifest를 물리적으로 수신하는지)는 이 파일의
관심사가 아니다 — INPROC이든 ROS2든 "RunManifest 객체를 손에 쥔 시점"부터
시작한다(Capability_runtrasport_readyreject.md: Transport Interface 분리 원칙,
"Layer logic에 transport 구현이 직접 들어가지 않도록").

candidate 방식(RunEndRestart_Rule.md §3): RunManifest → candidate backend/
RunContext/Run Transport 생성 → validation 성공 시에만 active로 commit. 중간에
실패하면 candidate를 전부 폐기(backend.shutdown())하고 프로세스에는 새 run의
흔적이 하나도 안 남는다 — 이전 active run(있었다면)도 그대로 유지되고 rollback
하지 않는다(§2 "이전 Run의 값을 그대로 유지하거나 rollback하지 않습니다"는
"이전 run을 새 값으로 덮어쓰지 않는다"는 뜻 — candidate가 실패해도 이전 active
run 객체 자체는 호출자가 별도로 들고 있는 한 안 건드린다는 의미로 구현했다).
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Callable

import avva_phase1 as m

from . import hashing
from .backend import RunConfig, create_simulator_adapter
from .backend import capability_digest as _capability_digest
from .gate import RegistryEntry, SimulationLayer

_NO_ID = m.OptionalBoundedId(False, "")
_NO_UUID = m.OptionalUuid128(False, b"\x00" * 16)
_NO_U64 = m.OptionalUint64(False, 0)
_ZERO_HASH = b"\x00" * 32

RunTransportFactory = Callable[
    [object, bytes, int, str, list[RegistryEntry], RunConfig],
    tuple[SimulationLayer, object],
]


@dataclass
class RunContext:
    """RunManifest에서 뽑아낸, 이 컴포넌트(Sim Backend)에 필요한 최소 구성."""
    run_id: bytes
    run_epoch: int
    sim_id: str
    adapter_type: m.NativeAdapterType
    run_config: RunConfig
    registry: list[RegistryEntry]


@dataclass
class StartupResult:
    ready: m.ComponentReady
    layer: SimulationLayer | None   # READY일 때만 존재
    backend: object | None          # READY일 때만 존재 — shutdown 책임은 호출자
    transport: object | None        # run_transport_factory가 반환한 두 번째 값(기본: publish sink)


def _find_sim_instance(manifest: m.RunManifest, sim_id: str) -> m.SimInstanceProfile | None:
    for p in manifest.sim_instances:
        if p.sim_id == sim_id:
            return p
    return None


def _build_registry(manifest: m.RunManifest) -> list[RegistryEntry]:
    # Phase 1: registry는 ego만으로 시작한다. NPC는 런타임에 ActorLifecycleIntent로
    # 추가되는 게 맞는 설계(§4.4)지만, gate.py가 아직 registry 동적 갱신을 지원하지
    # 않는 건 이미 별도로 플래그된 갭이고 이 함수 책임 밖이다.
    return [
        RegistryEntry(p.ego_id, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                      m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR)
        for p in manifest.ego_profiles
    ]


def _software_version() -> m.SoftwareVersion:
    # BoundedId는 빈 문자열 금지(README 규칙 6) — 아직 실제 빌드 메타데이터
    # 파이프라인이 없어 placeholder를 쓴다(팀 확인 대상 — 실제 git rev/build id로 교체 예정).
    return m.SoftwareVersion(
        component_id=m.ComponentId.SIM_BACKEND, semantic_version=m.SemanticVersion(0, 1, 0),
        git_commit="unknown", build_id="dev", image_digest=m.OptionalHash256(False, b"\x00" * 32))


def _component_ready_header(manifest: m.RunManifest, sim_id: str, payload_hash: bytes,
                            producer_instance_id: bytes, event_seq: int) -> m.CommonHeader:
    # §5.2.1: ComponentReady는 scope에 따름 — Sim Backend는 SIM, sim_id 필수·ego_id 부재.
    return m.CommonHeader(
        schema_major=1, schema_minor=0, run_id=manifest.header.run_id, run_epoch=manifest.header.run_epoch,
        scope_kind=m.ScopeKind.SIM, sim_id=m.OptionalBoundedId(True, sim_id), ego_id=_NO_ID,
        producer_id=m.ComponentId.SIM_BACKEND, producer_instance_id=producer_instance_id,
        event_seq=event_seq, correlation_id=_NO_UUID, sim_time_ns=_NO_U64,
        wall_time_unix_ns=time.time_ns(), payload_hash=payload_hash)


def _component_ready(manifest: m.RunManifest, sim_id: str, status: m.ReadyStatus,
                     capability_digest: bytes, reason_codes: list[int],
                     producer_instance_id: bytes, event_seq: int) -> m.ComponentReady:
    ready = m.ComponentReady(
        header=None, manifest_hash=manifest.header.payload_hash, ready_status=status,
        capability_digest=capability_digest, software_version=_software_version(),
        reason_codes=reason_codes)
    ready.header = _component_ready_header(manifest, sim_id, hashing.component_ready_hash(ready),
                                           producer_instance_id, event_seq)
    return ready


def _rejected(manifest: m.RunManifest, sim_id: str, reason_code: int,
             producer_instance_id: bytes, event_seq: int) -> StartupResult:
    ready = _component_ready(manifest, sim_id, m.ReadyStatus.READY_REJECTED, _ZERO_HASH, [reason_code],
                             producer_instance_id, event_seq)
    return StartupResult(ready, None, None, None)


def _default_run_transport_factory(backend, run_id, run_epoch, sim_id, registry, run_config):
    """Run Transport = INPROC_TEST publish sink. 실제 전송(ROS2 등)을 쓰려면 같은
    시그니처의 factory를 만들어 start_run(..., run_transport_factory=...)로 주입한다
    — Sim Backend 로직(SimulationLayer)은 그대로, transport만 교체된다."""
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, run_id, run_epoch, sim_id, registry, run_config,
                            publish=lambda ch, msg: published.append((ch, msg)))
    return layer, published


def start_run(manifest: m.RunManifest, own_sim_id: str,
              run_transport_factory: RunTransportFactory = _default_run_transport_factory,
              producer_instance_id: bytes | None = None, event_seq: int = 1) -> StartupResult:
    """RunManifest 1개를 받아 ComponentReady까지 진행한다 (candidate 방식).

    실패 지점에 따라 reason_code를 구분한다(Capability_runtrasport_readyreject.md §4):
    - Manifest에 필요한 sim_id가 없음 / 선택한 adapter_type을 이 컴포넌트가 지원 안 함 /
      execution_mode·rate가 capability 범위 밖  -> MANIFEST_MISMATCH
    - Manifest 자체는 유효하지만 Run Transport(엔드포인트) 구성 자체가 실패 -> TRANSPORT_ERROR

    producer_instance_id/event_seq: 기준서 5.2 "producer_instance_id는 프로세스 시작마다
    새 값, event_seq는 그 안에서 단조 증가" — 이 컴포넌트를 살아있는 동안 감싸는
    쪽(component.py의 SimBackendComponent)이 프로세스 수명 동안 하나씩 만들어서
    넘겨야 한다. 안 넘기면(단독 호출/테스트) 이 함수 호출마다 새로 만든다 — 예전엔
    이 fallback이 유일한 경로였어서 실제로 Core가 매 ComponentReady를 "새 인스턴스"로
    관측하는 버그였다.
    """
    if producer_instance_id is None:
        producer_instance_id = uuid.uuid4().bytes

    def rejected(reason_code: int) -> StartupResult:
        return _rejected(manifest, own_sim_id, reason_code, producer_instance_id, event_seq)

    profile = _find_sim_instance(manifest, own_sim_id)
    if profile is None:
        return rejected(m.MANIFEST_MISMATCH)
    # 기준서 line 55: "Single Ego(ego_0)로 시작" — Phase 1은 단일 ego. wire schema는
    # multi-ego 필드(BoundedSeq<EgoProfile,8>)를 유지하지만 이 레이어의 gate.py는
    # ego_slot이 1개뿐이라 2개 이상은 조용히 받았다가 PAYLOAD_CONFLICT로 run이
    # 죽는다 — Manifest 단계에서 명시적으로 거부하는 게 맞다.
    if len(manifest.ego_profiles) > 1:
        return rejected(m.MANIFEST_MISMATCH)

    run_config = RunConfig(target_rate_hz=1e9 / manifest.fixed_step_ns, execution_mode=manifest.execution_mode)
    registry = _build_registry(manifest)

    backend = None
    try:
        backend = create_simulator_adapter(profile.adapter_type)  # 지원 안 하는 타입 -> ValueError
        backend.initialize()
        backend.configure(run_config)  # capability rate/mode 재검증 -> ValueError/NotImplementedError
    except (ValueError, NotImplementedError):
        if backend is not None:
            backend.shutdown()
        return rejected(m.MANIFEST_MISMATCH)
    except Exception:
        if backend is not None:
            backend.shutdown()
        return rejected(m.TRANSPORT_ERROR)

    try:
        # vehicle_profile_id를 spawn spec으로 사용 — CARLA blueprint 문자열과
        # 정확히 매핑되려면 별도 변환표가 필요할 수 있음(팀 확인 대상, 지금은
        # mock/MetaDrive처럼 spec을 사실상 무시하는 어댑터에서만 검증됨).
        for p in manifest.ego_profiles:
            backend.spawn_actor(p.ego_id, p.vehicle_profile_id)
    except Exception:
        backend.shutdown()
        return rejected(m.TRANSPORT_ERROR)

    # Run Transport 구성 + mandatory endpoint READY 확인 (candidate)
    try:
        layer, transport = run_transport_factory(
            backend, manifest.header.run_id, manifest.header.run_epoch, own_sim_id, registry, run_config)
    except Exception:
        backend.shutdown()
        return rejected(m.TRANSPORT_ERROR)

    # SimulatorCapability 확정 + ComponentReady(READY)
    digest = _capability_digest(backend.capabilities())
    ready = _component_ready(manifest, own_sim_id, m.ReadyStatus.READY, digest, [],
                             producer_instance_id, event_seq)
    return StartupResult(ready, layer, backend, transport)


def enter_running(layer: SimulationLayer) -> None:
    """RunControlCommand(START) 수신에 대응하는 RUNNING 진입 — 최초 관측을 발행한다.
    이 호출 전에는 gate 상태가 READY가 아니므로 AdvanceFrame이 와도 tick 안 된다."""
    layer.bootstrap()
