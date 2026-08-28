"""CarlaSimulatorAdapter — SimulatorAdapter 구현.

경계 규칙: `import carla`는 저장소 전체에서 이 파일에만 존재한다. client 연결,
synchronous mode 설정, spawn/destroy, apply_batch_sync, world.tick(),
snapshot 읽기, native ID 매핑, 정적 속성 캐싱 전부 여기 격리 —
Sim Backend(gate.py)는 CARLA API를 직접 호출하지 않는다.

전제 (§2-5): synchronous_mode=True + fixed_delta_seconds 고정이어야
world.tick() lockstep이 성립한다. fixed delta는 하드코딩하지 않고
RunConfig.target_rate_hz에서 유도한다. no-teleport — 제어는
ApplyVehicleControl만 사용.
"""
from __future__ import annotations

import math
import uuid

import carla

import avva_phase1 as m

from .backend import ActorKinematics, RunConfig, SimulatorCapability, check_configuration
from . import transforms as tf

_CAPABILITIES = SimulatorCapability(
    component_type=m.ComponentId.SIM_BACKEND,
    simulator_type=m.NativeAdapterType.CARLA,
    max_ego=1,  # Phase 1 single ego; multi-ego spawn은 검증 후 확대
    # capability_digest_rule.md §2: configured=미실측 provisional 값 —
    # 1000 NPC stress 단계(기준서 §14.3) 후 validated_max_npc로 확정.
    schema_max_npc=1200, configured_max_npc=100, validated_max_npc=0,
    supports_sync=True, supports_async=True,  # async 모드는 존재하나 Phase 1은 거부(§2-5 전제)
    max_rate_hz=100.0,  # CARLA 권장 fixed_delta >= 0.01s
    supported_control_modes=(m.ControlMode.DIRECT_ACTUATION, m.ControlMode.VELOCITY_TARGET,
                             m.ControlMode.ACCELERATION_TARGET))  # 뒤 2개는 P-controller 근사

_WARMUP_TICKS = 10  # 스폰 직후 지면 안착(낙하/서스펜션)용 (§7.3)


class CarlaSimulatorAdapter:
    """CARLA 세계를 lockstep으로 구동한다.

    native actor ID(carla actor.id)는 이 클래스 밖으로 노출하지 않는다.
    """

    def __init__(self, host: str = "localhost", port: int = 2000, timeout_s: float = 10.0) -> None:
        self._host, self._port, self._timeout_s = host, port, timeout_s
        self._client: carla.Client | None = None
        self._world: carla.World | None = None
        self._original_settings = None
        self.native_session_id = b"\x00" * 16
        self.native_generation = 0
        self._dt = 0.0
        self._actors: dict[str, carla.Actor] = {}          # logical id -> native handle
        self._max_steer_rad: dict[str, float] = {}         # spawn 시 1회 캐싱 (§2-5 ⑤)
        self._dimensions: dict[str, tuple[float, float, float]] = {}  # 정적, spawn 시 캐싱
        self._last_frame = 0
        self._pending: list[tuple[str, carla.VehicleControl]] = []

    # ---- lifecycle -------------------------------------------------------
    def initialize(self) -> None:
        self._client = carla.Client(self._host, self._port)
        self._client.set_timeout(self._timeout_s)
        self._world = self._client.get_world()
        self._original_settings = self._world.get_settings()
        # native session은 연결 단위 (§9.2: 연결마다 발급, apply/tick 전 일치 검사)
        self.native_session_id = uuid.uuid4().bytes
        self.native_generation = 1

    def configure(self, config: RunConfig) -> None:
        if self._world is None:
            raise RuntimeError("initialize() must run before configure()")
        check_configuration(_CAPABILITIES, config)
        self._dt = config.fixed_step_ns / 1e9
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self._dt
        self._world.apply_settings(settings)

    def capabilities(self) -> SimulatorCapability:
        return _CAPABILITIES

    def spawn_actor(self, actor_id: str, spec: str) -> None:
        """spec = CARLA blueprint id (예: 'vehicle.tesla.model3')."""
        if actor_id in self._actors:
            raise ValueError(f"actor {actor_id} already spawned")
        spawn_points = self._world.get_map().get_spawn_points()
        if len(self._actors) >= len(spawn_points):
            raise RuntimeError("not enough spawn points on this map")
        bp = self._world.get_blueprint_library().find(spec)
        bp.set_attribute("role_name", actor_id)
        actor = self._world.spawn_actor(bp, spawn_points[len(self._actors)])
        self._actors[actor_id] = actor
        # 정적 속성 캐싱: bounding box는 half-extent라 x2 (§2-6)
        ext = actor.bounding_box.extent
        self._dimensions[actor_id] = (ext.x * 2, ext.y * 2, ext.z * 2)
        wheels = actor.get_physics_control().wheels
        self._max_steer_rad[actor_id] = math.radians(max(w.max_steer_angle for w in wheels) or 70.0)

    def remove_actor(self, actor_id: str) -> None:
        actor = self._actors.pop(actor_id)
        self._max_steer_rad.pop(actor_id, None)
        self._dimensions.pop(actor_id, None)
        actor.destroy()

    def reset(self) -> None:
        if self._dt <= 0.0:
            raise RuntimeError("configure() must run before reset()")
        # sync mode에서 spawn 반영 + 물리 안착까지 warm-up tick (§7.3)
        for _ in range(_WARMUP_TICKS):
            self._last_frame = self._world.tick()
        self._pending.clear()

    def shutdown(self) -> None:
        if self._actors and self._client is not None:
            self._client.apply_batch_sync(
                [carla.command.DestroyActor(a) for a in self._actors.values()], False)
            self._actors.clear()
            self._max_steer_rad.clear()
            self._dimensions.clear()
        if self._world is not None and self._original_settings is not None:
            self._world.apply_settings(self._original_settings)

    # ---- control conversion (공통 NeutralControl -> carla.VehicleControl) --
    def _to_vehicle_control(self, actor_id: str, c: m.NeutralControl) -> carla.VehicleControl:
        mask = c.valid_fields_mask
        steer = 0.0
        if mask & 1:
            steer = tf.steering_rad_to_native(c.steering_tire_angle_rad, self._max_steer_rad[actor_id])
        if c.control_mode == m.ControlMode.DIRECT_ACTUATION:
            throttle = c.throttle if mask & (1 << 5) else 0.0
            brake = c.brake if mask & (1 << 6) else 0.0
        elif c.control_mode == m.ControlMode.VELOCITY_TARGET:
            # ponytail: P-controller 근사 — CARLA에 native velocity control이 없어
            # no-teleport 원칙 하에 throttle/brake로 추종. 정밀 추종 필요해지면 PI로 교체.
            v = self._actors[actor_id].get_velocity()
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            err = c.velocity_mps - speed
            throttle = min(1.0, max(0.0, err * 0.5))
            brake = min(1.0, max(0.0, -err * 0.3))
        elif c.control_mode == m.ControlMode.ACCELERATION_TARGET:
            # ponytail: 정적 근사 매핑 — capability matrix 확정 전 임시.
            throttle = min(1.0, max(0.0, c.acceleration_mps2 / 3.0))
            brake = min(1.0, max(0.0, -c.acceleration_mps2 / 8.0))
        else:
            raise ValueError(f"unsupported control mode {c.control_mode} for {actor_id}")
        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake,
                                    hand_brake=bool(c.hand_brake) if mask & (1 << 8) else False)

    # ---- per-tick --------------------------------------------------------
    def apply_control(self, controls: dict[str, m.NeutralControl]) -> None:
        unknown = set(controls) - set(self._actors)
        if unknown:
            raise KeyError(f"unknown actors: {sorted(unknown)}")
        self._pending = [(a, self._to_vehicle_control(a, c)) for a, c in controls.items()]

    def tick(self) -> None:
        # ① 전 명령을 배치 한 번으로 적용 (tick 안 함) ② Response로 실패 검사
        # ③ 실패 없을 때만 world.tick() 1회 (§2-5)
        if self._pending:
            cmds = [carla.command.ApplyVehicleControl(self._actors[a], vc)
                    for a, vc in self._pending]
            responses = self._client.apply_batch_sync(cmds, False)
            failed = [(self._pending[i][0], r.error) for i, r in enumerate(responses) if r.has_error()]
            if failed:
                raise RuntimeError(f"apply failed: {failed}")
            self._pending = []
        frame = self._world.tick()
        if frame != self._last_frame + 1:
            raise RuntimeError(f"native frame jumped {self._last_frame} -> {frame}")
        self._last_frame = frame

    def get_actor_state(self) -> dict[str, ActorKinematics]:
        # 동적 상태는 snapshot 1회로 일괄 — actor별 RPC 금지 (§2-6)
        snapshot = self._world.get_snapshot()
        out: dict[str, ActorKinematics] = {}
        for actor_id, actor in self._actors.items():
            s = snapshot.find(actor.id)
            if s is None:
                raise RuntimeError(f"actor {actor_id} missing from snapshot (despawned?)")
            t, v, a, w = s.get_transform(), s.get_velocity(), s.get_acceleration(), s.get_angular_velocity()
            rot = t.rotation
            out[actor_id] = ActorKinematics(
                position=list(tf.location_to_common(t.location.x, t.location.y, t.location.z)),
                velocity=list(tf.velocity_to_common(v.x, v.y, v.z)),
                acceleration=list(tf.acceleration_to_common(a.x, a.y, a.z)),
                angular_velocity=list(tf.angular_velocity_to_common(w.x, w.y, w.z)),
                orientation_xyzw=tf.rotation_to_common_quaternion(rot.roll, rot.pitch, rot.yaw),
                dimensions_lwh=self._dimensions[actor_id])
        return out

    def native_frame_id(self) -> str:
        return str(self._last_frame)
