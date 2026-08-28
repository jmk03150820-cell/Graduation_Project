"""Simulator Adapter 공통 인터페이스 — Sim Backend 공통 로직과 simulator-specific
코드의 경계 (기준서 §2-7/§2-8).

경계 규칙: 이 파일과 gate.py는 공통/계약 타입만 다룬다. `import carla`는
carla_backend.py 안에만, `import rclpy`는 ros2_node.py 안에만 존재한다
(test_frame_lifecycle의 boundary test로 강제).

인터페이스는 특정 시뮬레이터의 호출 구조가 아니라 의미만 계약한다:
apply_control(적용) / tick(정확히 1 native fixed step) / get_actor_state(상태 추출)
+ lifecycle(initialize/configure/spawn/remove/reset/shutdown) + capabilities().
gym 스타일 시뮬레이터(MetaDrive)는 apply_control=action 저장, tick=env.step,
get_actor_state=관측 파싱으로 흡수한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields as dc_fields
from typing import Protocol

import avva_phase1 as m
from avva_hash_v1 import sha256, simulator_capability_bytes

# capability_hash의 domain prefix·wrap 규칙 + SimulatorCapability의 canonical
# byte encoding(필드 순서/폭/enum/bool/list 정렬)까지 전부 avva_hash_v1(common)의
# simulator_capability_bytes()가 소유한다 — 오늘 아침 공지: "Layer에서는 실제
# 값만 구성하고, canonical byte encoding 규칙은 common capability encoder가
# 담당". 이 파일은 dataclass 값을 그 함수 인자로 풀어 넘기기만 한다 — 인코딩
# 로직(u8/u16/u32/f64/sequence 호출)이 이 파일에 없다.
# prefix 값 자체(V1 유무)는 avva_hash_v1.CAPABILITY_PREFIX 쪽에서 팀 확인
# 대상으로 남아있다(capability_digest_rule.md §3 "AVVA-CAPABILITY-V1\0" vs
# Capability_runtrasport_readyreject.md §1 "AVVA-CAPABILITY\0" 표기 차이).


@dataclass(frozen=True)
class SimulatorCapability:
    """Simulator Layer의 ComponentCapability (capability_digest_rule.md §1).

    공통 필드(component_type/adapter type/max_npc/max_ego/sync/async/rate) +
    Simulator-specific(simulator_type). max_npc는 §2 규칙대로 3단 구분:
    schema capacity(1200)를 그대로 선언하지 않고, stress test 전에는
    configured(provisional) 값을 쓰며 검증 후 validated로 확정한다.
    runtime metric(현재 NPC 수/tick/FPS 등)은 여기 넣지 않는다 (§4).
    """
    component_type: m.ComponentId          # SIM_BACKEND 고정
    simulator_type: m.NativeAdapterType    # layer-specific: CARLA/MetaDrive/...
    max_ego: int
    schema_max_npc: int                    # 메시지 구조가 표현 가능한 최대치 (=1200)
    configured_max_npc: int                # 현재 설정상 허용 최대치 (provisional)
    validated_max_npc: int                 # stress test로 검증된 값; 0 = 미검증
    supports_sync: bool
    supports_async: bool
    # rate는 "이번 Run의 실행 rate"가 아니라 "지원 가능한 범위"
    # (Capability_runtrasport_readyreject.md §2 — target_rate_hz는 RunConfig 소관)
    supported_rate_min_hz: float
    supported_rate_max_hz: float
    supported_control_modes: tuple[m.ControlMode, ...]  # 순서 의미 없는 집합 — hash 시 정렬

    @property
    def effective_max_npc(self) -> int:
        """Phase 1: 검증 전엔 configured, 검증 후엔 validated (§2)."""
        return self.validated_max_npc or self.configured_max_npc


def capability_canonical_bytes(cap: SimulatorCapability) -> bytes:
    """cap의 실제 값만 common encoder에 넘긴다 — 인코딩 규칙(순서/폭/정렬)은 여기 없음."""
    assert len(dc_fields(SimulatorCapability)) == 11, \
        "필드가 추가/삭제됨 — common avva_hash_v1.simulator_capability_bytes()도 함께 갱신할 것"
    return simulator_capability_bytes(
        component_type=int(cap.component_type), simulator_type=int(cap.simulator_type),
        max_ego=cap.max_ego, schema_max_npc=cap.schema_max_npc,
        configured_max_npc=cap.configured_max_npc, validated_max_npc=cap.validated_max_npc,
        supports_sync=cap.supports_sync, supports_async=cap.supports_async,
        supported_rate_min_hz=cap.supported_rate_min_hz,
        supported_rate_max_hz=cap.supported_rate_max_hz,
        supported_control_modes=[int(v) for v in cap.supported_control_modes])


def capability_digest(cap: SimulatorCapability) -> bytes:
    """capability_digest = SHA256(canonical_encode(ComponentCapability)) (§3)."""
    return sha256(capability_canonical_bytes(cap))


@dataclass(frozen=True)
class RunConfig:
    """Run 단위 실행 설정. fixed delta는 하드코딩하지 않고 target_rate_hz에서 유도."""
    target_rate_hz: float = 20.0
    execution_mode: m.ExecutionMode = m.ExecutionMode.SYNC_FIXED_STEP

    @property
    def fixed_step_ns(self) -> int:
        return round(1e9 / self.target_rate_hz)


def check_configuration(cap: SimulatorCapability, config: RunConfig) -> None:
    """capability fail-fast (§2-7): 시나리오 요구를 못 채우면 실행 전에 실패."""
    if config.execution_mode == m.ExecutionMode.SYNC_FIXED_STEP:
        if not cap.supports_sync:
            raise ValueError(f"{cap.simulator_type.name}: sync fixed-step unsupported")
    else:
        # ponytail: Phase 1은 sync lockstep만 구현. async는 Phase 2 Execution
        # Policy 확장점 — 이 분기가 그 경계이며, supports_async 시뮬레이터라도
        # 정책 구현 전까지는 거부한다.
        raise NotImplementedError("Phase 1 supports SYNC_FIXED_STEP only (async = Phase 2)")
    if not cap.supported_rate_min_hz <= config.target_rate_hz <= cap.supported_rate_max_hz:
        raise ValueError(f"target_rate_hz {config.target_rate_hz} out of supported range "
                         f"[{cap.supported_rate_min_hz}, {cap.supported_rate_max_hz}] "
                         f"for {cap.simulator_type.name}")


@dataclass
class ActorKinematics:
    """공통(ENU, SI, radian) 좌표계의 per-actor 동적 상태. simulator native 값 금지."""
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    acceleration: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    angular_velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    dimensions_lwh: tuple[float, float, float] | None = None  # 정적, spawn 시 1회 캐싱


class SimulatorAdapter(Protocol):
    """한 native simulator를 감싸는 어댑터. logical actor_id만 노출한다.

    Sim Backend(gate.py)는 이 인터페이스만 호출한다 — 새 시뮬레이터는
    이 Protocol 구현 클래스 하나로 추가되고 Sim Backend는 수정 0.
    호출 순서: initialize() → configure() → spawn_actor()* → [reset() →
    apply_control()/tick()/get_actor_state() 루프] → shutdown().
    """
    native_session_id: bytes
    native_generation: int

    def initialize(self) -> None:
        """native 연결 수립 (CARLA: client+world 접속, session id 발급)."""

    def configure(self, config: RunConfig) -> None:
        """capability 검증 후 실행 모드/fixed delta 설정. 위반 시 즉시 예외."""

    def capabilities(self) -> SimulatorCapability:
        """이 시뮬레이터의 능력 서술자."""

    def spawn_actor(self, actor_id: str, spec: str) -> None:
        """actor 1개 생성. spec은 어댑터별 모델 지정자 (CARLA: blueprint id)."""

    def remove_actor(self, actor_id: str) -> None:
        """actor 1개 제거."""

    def reset(self) -> None:
        """run 시작 상태로 정착 (warm-up 포함). native frame 기준점 재설정."""

    def apply_control(self, controls: dict[str, m.NeutralControl]) -> None:
        """이번 tick에 적용할 제어 일괄 등록. 실패는 예외로 (gate가 NATIVE_APPLY_ERROR 처리)."""

    def tick(self) -> None:
        """native fixed step 정확히 1회. 실패는 예외로 (gate가 NATIVE_TICK_ERROR 처리)."""

    def get_actor_state(self) -> dict[str, ActorKinematics]:
        """공통 좌표계로 변환된 전체 actor 상태."""

    def native_frame_id(self) -> str:
        """native simulator frame counter (BoundedId, 선행 0 없는 10진수)."""

    def shutdown(self) -> None:
        """native 연결 정리 (sync mode 해제, actor destroy 등)."""


def create_simulator_adapter(simulator_type: str | m.NativeAdapterType, **kwargs) -> "SimulatorAdapter":
    """simulator_type으로 어댑터 선택 (CARLA/MetaDrive/mock).

    RunManifest.SimInstanceProfile.adapter_type이 여기로 이어질 예정 —
    import는 함수 안에서만 해서 미설치 시뮬레이터 패키지를 요구하지 않는다.
    """
    if isinstance(simulator_type, m.NativeAdapterType):
        name = simulator_type.name.lower()
    else:
        name = simulator_type.strip().lower()
    if name == "carla":
        from .carla_backend import CarlaSimulatorAdapter
        return CarlaSimulatorAdapter(**kwargs)
    if name == "metadrive":
        from .metadrive_backend import MetaDriveSimulatorAdapter
        return MetaDriveSimulatorAdapter(**kwargs)
    if name in ("mock", "native_adapter_unspecified"):
        from .mock_backend import MockSimulatorAdapter
        return MockSimulatorAdapter(**kwargs)
    raise ValueError(f"unknown simulator_type: {simulator_type!r}")
