"""Mockup 기반 Sim Backend 독립 실행 harness.

다른 실제 레이어(Core/Traffic/Ego, ROS 2, CARLA/MetaDrive) 없이 이 저장소
하나만으로 완결 실행된다 — mock Core(AdvanceFrame+FrameCompleteAck 발급),
mock Traffic(NpcControlBatch), mock Ego(EgoControlCommand) + MockSimulatorAdapter
로 N frame lockstep을 끝까지 돌린다.

실행: python -m simulation_layer.sim_local_harness [N_FRAMES]
"""
from __future__ import annotations

import sys
import uuid

import avva_phase1 as m
from simulation_layer.gate import RegistryEntry, SimulationLayer
from simulation_layer.test_frame_lifecycle import (
    EGO_ID, EPOCH, RUN_CONFIG, RUN_ID, SIM_ID, by_channel, make_mock_adapter,
    mock_ack, mock_advance, mock_ego_command, mock_npc_batch,
)

REGISTRY = [
    RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.ACTIVE_PROXY, m.ActorClass.TRUCK),
]


def run(n_frames: int) -> None:
    backend = make_mock_adapter(REGISTRY)
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, SIM_ID, REGISTRY, RUN_CONFIG,
                            publish=lambda ch, msg: published.append((ch, msg)))
    layer.bootstrap()
    print(f"harness: mock Core/Traffic/Ego + MockSimulatorAdapter, {n_frames} frame 목표")
    print(f"{'tick':>4} {'state':<14} {'validity':<9} {'native_step':>11} {'npc_1.v_x':>10}")

    for _ in range(n_frames):
        ws = by_channel(published, "world_state")[-1]
        obs = by_channel(published, "observation")[-1]
        layer.on_ego_control(mock_ego_command(obs))
        layer.on_npc_batch(mock_npc_batch(ws))
        ready = by_channel(published, "command_ready")[-1]
        assert layer.state == "READY"

        layer.on_advance_frame(mock_advance(ready, uuid.uuid4().bytes))
        assert layer.state == "PUBLISHING"
        fc = by_channel(published, "frame_complete")[-1]

        new_ws = by_channel(published, "world_state")[-1]
        npc1 = next(a for a in new_ws.actors if a.actor_id == "npc_1")
        print(f"{fc.state_tick_id:>4} {layer.state:<14} {ready.validity.name:<9} "
              f"{backend.native_frame_id():>11} {npc1.velocity_mps.x:>10.2f}")

        layer.on_frame_complete_ack(mock_ack(fc))
        assert layer.state == "WAITING_INPUTS"

    backend.shutdown()
    print(f"\n harness 완료: {n_frames} frame, 최종 state_tick={layer.state_tick}, "
         f"native_step={backend.native_frame_id()} — 외부 레이어/프로세스 없이 독립 실행됨")


if __name__ == "__main__":
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 10)
