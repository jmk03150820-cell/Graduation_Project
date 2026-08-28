"""MetaDriveSimulatorAdapter — SimulatorAdapter 구현 (2nd backend, portability 증명용).

경계 규칙: `import metadrive`는 저장소 전체에서 이 파일에만 존재한다.

CARLA와의 핵심 차이 (전부 실측 확인, 문서만 보고 가정하지 않음 — 2026-08-27
headless smoke test, metadrive-simulator==0.4.3):
- 좌표계: right-handed, Z-up (CARLA는 left-handed) — Y negate 불필요.
  heading_theta 부호도 우리 §4.3(0=+X, +Z축 반시계 양수)과 그대로 일치
  (steer=+1.0 → heading_theta 증가 실측 확인). CARLA처럼 축 반전 코드가 없다.
- position/velocity가 2D (x, y)뿐 — 지면 높이 개념이 없어 z/vz는 항상 0으로 채운다.
- traffic_density=0으로 두면 IDM 내장 traffic이 전혀 안 생김 — NPC는 MetaDrive의
  "traffic"이 아니라 MultiAgentEnv의 "controlled agent"로 다룬다(연구 메모 결론).
  각 agent가 EnvInputPolicy로 매 tick 우리 action을 받는다.
- native frame 카운터가 없다 — CARLA의 WorldSnapshot.frame 대응은 env.episode_step
  (env.step() 1회당 정확히 +1, 실측 확인).
- num_agents는 env 생성 시점에 고정된다(엔진이 "agent0","agent1"... id를 자체
  부여) — CARLA처럼 spawn_actor() 즉시 생성이 안 되고, 우리 logical actor_id는
  reset()에서 실제 env를 만들 때 엔진 id와 순서대로 매핑한다(gym 스타일 3-call
  흡수 패턴, backend.py 문서에 이미 예고된 것).
- angular_velocity를 직접 주는 API가 없어 heading_theta 차분/dt로 근사한다(ponytail).
- vehicle.get_state()에 등가/정밀 angular velocity가 필요해지면 물리 body handle을
  더 파야 하며, 이건 검증 필요 항목으로 남긴다.
"""
from __future__ import annotations

import math

from metadrive.envs.marl_envs.multi_agent_metadrive import MultiAgentMetaDrive

import avva_phase1 as m

from .backend import ActorKinematics, RunConfig, SimulatorCapability, check_configuration
from . import transforms as tf

_CAPABILITIES = SimulatorCapability(
    component_type=m.ComponentId.SIM_BACKEND,
    simulator_type=m.NativeAdapterType.NATIVE_ADAPTER_UNSPECIFIED,  # MetaDrive는 계약 enum에 없음(팀 확인 필요)
    max_ego=8,
    # ponytail: 미실측 provisional — MultiAgentEnv 대규모 agent 수 stress test 전.
    schema_max_npc=1200, configured_max_npc=50, validated_max_npc=0,
    supports_sync=True, supports_async=False,
    supported_rate_min_hz=1.0, supported_rate_max_hz=50.0,  # physics_world_step_size 하한 0.02s 기준
    # DIRECT_ACTUATION만 native, VELOCITY_TARGET/ACCELERATION_TARGET은 P-controller 근사(CARLA와 동일 패턴)
    supported_control_modes=(m.ControlMode.DIRECT_ACTUATION, m.ControlMode.VELOCITY_TARGET,
                             m.ControlMode.ACCELERATION_TARGET))


class MetaDriveSimulatorAdapter:
    """MultiAgentMetaDrive를 lockstep으로 구동한다.

    engine agent id("agent0" 등)는 이 클래스 밖으로 노출하지 않는다.
    """

    def __init__(self) -> None:
        self.native_session_id = b"\x00" * 16
        self.native_generation = 0
        self._config: RunConfig | None = None
        self._env: MultiAgentMetaDrive | None = None
        self._pending_spawns: list[tuple[str, str]] = []  # (actor_id, spec) — reset()에서 일괄 생성
        self._actor_to_engine: dict[str, str] = {}
        self._pending_actions: dict[str, list[float]] = {}
        self._last_heading: dict[str, float] = {}  # angular_velocity 근사용
        self._episode_step = 0

    # ---- lifecycle -------------------------------------------------------
    def initialize(self) -> None:
        import uuid
        self.native_session_id = uuid.uuid4().bytes
        self.native_generation = 1

    def configure(self, config: RunConfig) -> None:
        check_configuration(_CAPABILITIES, config)
        self._config = config

    def capabilities(self) -> SimulatorCapability:
        return _CAPABILITIES

    def spawn_actor(self, actor_id: str, spec: str = "") -> None:
        """num_agents가 env 생성 시점에 고정되므로 실제 spawn은 reset()으로 미룬다."""
        if self._env is not None:
            raise RuntimeError("MetaDrive는 reset() 이후 동적 spawn을 지원하지 않는다 (Phase 1 범위)")
        if any(a == actor_id for a, _ in self._pending_spawns):
            raise ValueError(f"actor {actor_id} already spawned")
        self._pending_spawns.append((actor_id, spec))

    def remove_actor(self, actor_id: str) -> None:
        # ponytail: MultiAgentEnv의 respawn/removal API는 Phase 1 범위 밖(Registry
        # 동적 갱신 자체가 아직 gate에 안 붙어 있음, CARLA도 동일 상태).
        raise NotImplementedError("MetaDrive adapter: runtime remove_actor는 Phase 1 미지원")

    def reset(self) -> None:
        if self._config is None:
            raise RuntimeError("configure() must run before reset()")
        if not self._pending_spawns:
            raise RuntimeError("spawn_actor()로 최소 1개 actor를 등록한 뒤 reset() 호출")
        if self._env is None:
            step_size = self._config.fixed_step_ns / 1e9
            self._env = MultiAgentMetaDrive(config=dict(
                num_agents=len(self._pending_spawns), allow_respawn=False,
                use_render=False, traffic_density=0.0,  # NPC는 agent로 직접 제어(§ 상단 설명)
                crash_done=False, out_of_road_done=False,  # Phase 1: registry 정적 — 임의 제거 방지
                physics_world_step_size=step_size, decision_repeat=1,  # 1 env.step() = 정확히 1 native step
            ))
        obs, _info = self._env.reset()
        engine_ids = sorted(self._env.agents, key=lambda s: int(s.replace("agent", "")))
        if len(engine_ids) != len(self._pending_spawns):
            raise RuntimeError(f"engine agent 수({len(engine_ids)}) != 요청 actor 수"
                               f"({len(self._pending_spawns)})")
        self._actor_to_engine = {a: e for (a, _), e in zip(self._pending_spawns, engine_ids)}
        self._last_heading = {a: self._env.agents[e].heading_theta for a, e in self._actor_to_engine.items()}
        self._episode_step = 0
        self._pending_actions.clear()

    def shutdown(self) -> None:
        if self._env is not None:
            self._env.close()

    # ---- per-tick --------------------------------------------------------
    def apply_control(self, controls: dict[str, m.NeutralControl]) -> None:
        unknown = set(controls) - set(self._actor_to_engine)
        if unknown:
            raise KeyError(f"unknown actors: {sorted(unknown)}")
        self._pending_actions = {a: self._to_action(a, c) for a, c in controls.items()}

    def _to_action(self, actor_id: str, c: m.NeutralControl) -> list[float]:
        mask = c.valid_fields_mask
        # tf.steering_rad_to_native는 CARLA의 부호 반전(우회전 양수)을 전제로 한
        # 함수라 여기서 재사용하지 않는다 — MetaDrive는 실측상 우리 부호(좌회전
        # 양수)와 그대로 같아 단순 정규화만 한다.
        steer = c.steering_tire_angle_rad / _MAX_STEER_RAD if mask & 1 else 0.0
        if c.control_mode == m.ControlMode.DIRECT_ACTUATION:
            throttle = c.throttle if mask & (1 << 5) else 0.0
            brake = c.brake if mask & (1 << 6) else 0.0
            throttle_brake = throttle if throttle > 0.0 else -brake
        elif c.control_mode == m.ControlMode.VELOCITY_TARGET:
            # ponytail: P-controller 근사 — MetaDrive에 native velocity control이
            # 없어 CARLA 어댑터와 동일 패턴으로 throttle_brake 단일 축에 매핑.
            v = self._env.agents[self._actor_to_engine[actor_id]]
            err = c.velocity_mps - v.speed
            throttle_brake = max(-1.0, min(1.0, err * 0.5))
        elif c.control_mode == m.ControlMode.ACCELERATION_TARGET:
            throttle_brake = max(-1.0, min(1.0, c.acceleration_mps2 / 3.0))
        else:
            raise ValueError(f"unsupported control mode {c.control_mode} for {actor_id}")
        return [max(-1.0, min(1.0, steer)), max(-1.0, min(1.0, throttle_brake))]

    def tick(self) -> None:
        action = {self._actor_to_engine[a]: v for a, v in self._pending_actions.items()}
        for engine_id in self._actor_to_engine.values():
            action.setdefault(engine_id, [0.0, 0.0])  # 미제출 actor는 중립(§6.6 0-normalize와 별개, 물리 유지 목적)
        step_before = self._episode_step
        self._env.step(action)
        self._episode_step = self._env.episode_step
        if self._episode_step != step_before + 1:
            raise RuntimeError(f"native step이 정확히 +1이 아님: {step_before} -> {self._episode_step}")
        self._pending_actions = {}

    def get_actor_state(self) -> dict[str, ActorKinematics]:
        out: dict[str, ActorKinematics] = {}
        dt = self._config.fixed_step_ns / 1e9
        for actor_id, engine_id in self._actor_to_engine.items():
            v = self._env.agents[engine_id]
            heading = v.heading_theta
            angular_z = (heading - self._last_heading[actor_id]) / dt  # ponytail: 근사, 직접 API 없음
            self._last_heading[actor_id] = heading
            out[actor_id] = ActorKinematics(
                position=[v.position[0], v.position[1], 0.0],       # 2D sim — 고도 없음
                velocity=[v.velocity[0], v.velocity[1], 0.0],
                acceleration=[0.0, 0.0, 0.0],  # MetaDrive가 가속도를 직접 안 줌 — 필요해지면 속도 차분으로 유도
                angular_velocity=[0.0, 0.0, angular_z],
                orientation_xyzw=tf.rpy_to_quaternion(0.0, 0.0, heading),
                dimensions_lwh=(v.LENGTH, v.WIDTH, v.HEIGHT))
        return out

    def native_frame_id(self) -> str:
        return str(self._episode_step)


_MAX_STEER_RAD = math.radians(40.0)  # ponytail: MetaDrive 기본 차량 근사값, 실제 차량별 상이 가능 — 검증 필요
