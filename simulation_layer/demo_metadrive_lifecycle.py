"""실제 MetaDrive 기준 frame lifecycle 데모 (CARLA 서버 불필요, 순수 파이썬).

demo_carla_lifecycle.py와 완전히 같은 구조 — MetaDriveSimulatorAdapter만 꽂았을
뿐 gate/FSM/메시지 코드는 동일함을 보여주는 것이 목적(과제 4: "CARLA에서 이미
구현한 로직을 복제하지 않도록").
"""
from __future__ import annotations

import uuid

import avva_phase1 as m
from simulation_layer.backend import RunConfig
from simulation_layer.metadrive_backend import MetaDriveSimulatorAdapter
from simulation_layer.gate import RegistryEntry, SimulationLayer
from simulation_layer.test_frame_lifecycle import (
    EGO_ID, EPOCH, RUN_ID, by_channel, mock_ack, mock_advance,
    mock_ego_command, mock_npc_batch,
)

SIM_ID = "metadrive_0"
RUN_CONFIG = RunConfig(target_rate_hz=20.0)  # MetaDrive 지원범위(1~50Hz) 안, CARLA 데모와 다른 값 사용 가능함을 보임
STEP_NS = RUN_CONFIG.fixed_step_ns
REGISTRY = [
    RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
]
N_FRAMES = 20


def retag(msg):
    msg.header.sim_id = m.OptionalBoundedId(True, SIM_ID)
    return msg


def main() -> None:
    print(f"MetaDrive 초기화 중 (target_rate={RUN_CONFIG.target_rate_hz}Hz "
          f"-> fixed_delta={STEP_NS / 1e9}s)...")
    backend = MetaDriveSimulatorAdapter()
    backend.initialize()
    backend.configure(RUN_CONFIG)
    for e in REGISTRY:
        backend.spawn_actor(e.actor_id)
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, SIM_ID, REGISTRY, RUN_CONFIG,
                            publish=lambda ch, msg: published.append((ch, msg)))
    try:
        layer.bootstrap()
        ws = by_channel(published, "world_state")[-1]
        print(f"스폰 완료. native_frame_id={ws.native_frame_ref.native_frame_id}, "
              f"actors={[a.actor_id for a in ws.actors]}")

        for _ in range(N_FRAMES):
            ws = by_channel(published, "world_state")[-1]
            obs = by_channel(published, "observation")[-1]
            layer.on_ego_control(retag(mock_ego_command(obs, throttle=0.6)))
            layer.on_npc_batch(retag(mock_npc_batch(ws)))
            ready = by_channel(published, "command_ready")[-1]
            assert ready.validity == m.Validity.VALID

            decision = uuid.uuid4().bytes
            native_before = int(backend.native_frame_id())
            layer.on_advance_frame(retag(mock_advance(ready, decision)))
            native_after = int(backend.native_frame_id())
            assert native_after == native_before + 1, "native step이 정확히 1회가 아님"

            fc = by_channel(published, "frame_complete")[-1]
            assert fc.decision_id == decision and fc.applied_snapshot_hash == ready.snapshot_hash

            if fc.state_tick_id == 3:  # 중복 AdvanceFrame 방어 확인 (CARLA 데모와 동일 검증)
                layer.on_advance_frame(retag(mock_advance(ready, decision)))
                assert int(backend.native_frame_id()) == native_after, "중복 승인에서 재tick 발생"
                print("        -> 동일 AdvanceFrame 재전송: MetaDrive step() 재실행 없음 확인")

            layer.on_frame_complete_ack(retag(mock_ack(fc)))
            assert layer.state == "WAITING_INPUTS"

            ego = next(a for a in by_channel(published, "world_state")[-1].actors if a.actor_id == EGO_ID)
            print(f"tick {fc.state_tick_id}: native {native_before}->{native_after} | "
                  f"ego pos=({ego.position_m.x:.2f},{ego.position_m.y:.2f}) "
                  f"v=({ego.velocity_mps.x:.2f},{ego.velocity_mps.y:.2f}) m/s")

        ego_states = [next(a for a in w.actors if a.actor_id == EGO_ID)
                      for ch, w in published if ch.endswith("world_state")]
        moved = ((ego_states[-1].position_m.x - ego_states[0].position_m.x) ** 2
                + (ego_states[-1].position_m.y - ego_states[0].position_m.y) ** 2) ** 0.5
        print(f"\n{N_FRAMES} frame 완료: ego 이동거리 {moved:.3f} m "
              f"(throttle 0.6 실제 물리 반영 확인)")
        assert moved > 0.1
        print("PASS — 실제 MetaDrive 기준 frame lifecycle + 중복 승인 방어 + state 추출 동작")
        print("       (gate.py/hashing.py/RegistryEntry — CARLA 데모와 완전히 동일 코드, adapter만 교체)")
    finally:
        backend.shutdown()


if __name__ == "__main__":
    main()
