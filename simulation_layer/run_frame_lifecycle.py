"""단일 entry point — simulator_type 하나만 바꿔서 CARLA/MetaDrive/mock 중 실행.

Sim Backend(gate.py/hashing.py) 코드는 simulator_type과 완전히 무관 — 바뀌는
줄은 `create_simulator_adapter(simulator_type)` 호출 한 줄뿐임을 이 파일
자체로 증명한다 (다른 simulator_type별 분기가 존재하는 곳은 spawn spec 딕셔너리
하나뿐 — CARLA만 blueprint 문자열이 필요해서 생기는 불가피한 차이).

실행: python -m simulation_layer.run_frame_lifecycle [carla|metadrive|mock] [N_FRAMES]
"""
from __future__ import annotations

import sys
import uuid

import avva_phase1 as m
from simulation_layer.backend import RunConfig, create_simulator_adapter
from simulation_layer.gate import RegistryEntry, SimulationLayer
from simulation_layer.test_frame_lifecycle import (
    EPOCH, RUN_ID, by_channel, mock_ack, mock_advance, mock_ego_command, mock_npc_batch,
)

EGO_ID = "ego_0"
REGISTRY = [
    RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.TRUCK),
]
# CARLA만 blueprint 문자열이 필요 — simulator_type별로 다른 유일한 지점
_CARLA_SPAWN = {EGO_ID: "vehicle.tesla.model3", "npc_1": "vehicle.audi.tt", "npc_2": "vehicle.bmw.grandtourer"}


def run(simulator_type: str, n_frames: int) -> None:
    sim_id = f"{simulator_type.lower()}_0"
    run_config = RunConfig(target_rate_hz=20.0)

    backend = create_simulator_adapter(simulator_type)  # <-- simulator_type이 바뀌는 유일한 줄
    backend.initialize()
    backend.configure(run_config)
    specs = _CARLA_SPAWN if simulator_type.lower() == "carla" else {}
    for e in REGISTRY:
        backend.spawn_actor(e.actor_id, specs.get(e.actor_id, ""))

    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, sim_id, REGISTRY, run_config,
                            publish=lambda ch, msg: published.append((ch, msg)))

    def retag(msg):
        msg.header.sim_id = m.OptionalBoundedId(True, sim_id)
        return msg

    layer.bootstrap()
    print(f"[{simulator_type}] backend={type(backend).__name__}, "
         f"bootstrap OK, native_frame_id={backend.native_frame_id()}")

    for i in range(n_frames):
        ws = by_channel(published, "world_state")[-1]
        obs = by_channel(published, "observation")[-1]
        layer.on_ego_control(retag(mock_ego_command(obs)))
        layer.on_npc_batch(retag(mock_npc_batch(ws)))
        ready = by_channel(published, "command_ready")[-1]
        assert ready.validity == m.Validity.VALID
        layer.on_advance_frame(retag(mock_advance(ready, uuid.uuid4().bytes)))
        fc = by_channel(published, "frame_complete")[-1]
        layer.on_frame_complete_ack(retag(mock_ack(fc)))
        assert layer.state == "WAITING_INPUTS"

    print(f"[{simulator_type}] {n_frames} frame 완료, state_tick={layer.state_tick}, "
         f"native_step={backend.native_frame_id()}")
    backend.shutdown()


if __name__ == "__main__":
    sim_type = sys.argv[1] if len(sys.argv) > 1 else "mock"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    run(sim_type, n)
