"""Mockup SimulatorAdapter — no simulator, just enough state for one frame lifecycle.

Implements the SimulatorAdapter interface in backend.py; CARLA implementation is
carla_backend.py.
"""
from __future__ import annotations

import uuid

import avva_phase1 as m

from .backend import (ActorKinematics, RunConfig, SimulatorAdapter,  # noqa: F401
                      SimulatorCapability, check_configuration)

_CAPABILITIES = SimulatorCapability(
    component_type=m.ComponentId.SIM_BACKEND,
    simulator_type=m.NativeAdapterType.NATIVE_ADAPTER_UNSPECIFIED,  # mock은 native 아님
    max_ego=8,  # 계약 AVVA_MAX_EGO_COUNT
    # capability_digest_rule.md §2: schema capacity(1200)를 max로 선언하지 않는다.
    schema_max_npc=1200, configured_max_npc=200, validated_max_npc=0,  # stress test 전 provisional
    supports_sync=True, supports_async=False,
    supported_rate_min_hz=1.0, supported_rate_max_hz=1000.0,
    supported_control_modes=(m.ControlMode.VELOCITY_TARGET, m.ControlMode.ACCELERATION_TARGET,
                             m.ControlMode.DIRECT_ACTUATION))


class MockSimulatorAdapter:
    """Point-mass actors on the +X axis. Deterministic, no physics engine.

    ponytail: 1-D kinematics only — enough to see state change per tick.
    """

    def __init__(self) -> None:
        self.native_session_id = b"\x00" * 16
        self.native_generation = 0
        self._dt = 0.0
        self._frame = 0
        self._actors: dict[str, ActorKinematics] = {}
        self._pending: dict[str, m.NeutralControl] = {}

    # ---- lifecycle -------------------------------------------------------
    def initialize(self) -> None:
        self.native_session_id = uuid.uuid4().bytes
        self.native_generation = 1

    def configure(self, config: RunConfig) -> None:
        check_configuration(_CAPABILITIES, config)
        self._dt = config.fixed_step_ns / 1e9

    def capabilities(self) -> SimulatorCapability:
        return _CAPABILITIES

    def spawn_actor(self, actor_id: str, spec: str = "") -> None:
        if actor_id in self._actors:
            raise ValueError(f"actor {actor_id} already spawned")
        self._actors[actor_id] = ActorKinematics()

    def remove_actor(self, actor_id: str) -> None:
        del self._actors[actor_id]

    def reset(self) -> None:
        if self._dt <= 0.0:
            raise RuntimeError("configure() must run before reset()")
        self._frame = 0
        for a in self._actors.values():
            a.position[:] = [0.0, 0.0, 0.0]
            a.velocity[:] = [0.0, 0.0, 0.0]
        self._pending.clear()

    def shutdown(self) -> None:
        pass

    # ---- per-tick --------------------------------------------------------
    def apply_control(self, controls: dict[str, m.NeutralControl]) -> None:
        unknown = set(controls) - set(self._actors)
        if unknown:
            raise KeyError(f"unknown actors: {sorted(unknown)}")
        self._pending = dict(controls)

    def tick(self) -> None:
        for actor_id, kin in self._actors.items():
            ctrl = self._pending.get(actor_id)
            if ctrl is not None:
                mask = ctrl.valid_fields_mask
                if ctrl.control_mode == m.ControlMode.VELOCITY_TARGET and mask & (1 << 2):
                    kin.velocity[0] = ctrl.velocity_mps
                elif ctrl.control_mode == m.ControlMode.DIRECT_ACTUATION:
                    throttle = ctrl.throttle if mask & (1 << 5) else 0.0
                    brake = ctrl.brake if mask & (1 << 6) else 0.0
                    kin.velocity[0] = max(0.0, kin.velocity[0] + (throttle * 3.0 - brake * 8.0) * self._dt)
                elif ctrl.control_mode == m.ControlMode.ACCELERATION_TARGET and mask & (1 << 3):
                    kin.velocity[0] += ctrl.acceleration_mps2 * self._dt
            kin.position[0] += kin.velocity[0] * self._dt
        self._pending.clear()
        self._frame += 1

    def get_actor_state(self) -> dict[str, ActorKinematics]:
        return self._actors

    def native_frame_id(self) -> str:
        return str(self._frame)
