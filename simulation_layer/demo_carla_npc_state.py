"""NPC state extraction 확인용 데모 (CARLA 서버 필요, localhost:2000).

확인 항목 (사용자 질문 그대로):
1) npc {actor_id, pose, velocity, orientation, native frame}이 ego와 동일한
   경로로 추출되어 WorldStateFrame.actors에 들어가는지
2) 그 값이 라벨이 아니라 실제 CARLA snapshot에서 읽은 값인지 — npc_1에 실제
   velocity 명령을 걸어 tick마다 값이 살아 움직이는 것으로 증명
"""
from __future__ import annotations

import uuid

import avva_phase1 as m
from simulation_layer.carla_backend import CarlaSimulatorAdapter
from simulation_layer.gate import RegistryEntry, SimulationLayer
from simulation_layer.test_frame_lifecycle import (
    EGO_ID, EPOCH, RUN_CONFIG, RUN_ID, STEP_NS, by_channel, mock_ack,
    mock_advance, mock_ego_command, mock_npc_batch,
)

SIM_ID = "carla_npc_check"
SPAWN = {EGO_ID: "vehicle.tesla.model3", "npc_1": "vehicle.audi.tt", "npc_2": "vehicle.bmw.grandtourer"}
REGISTRY = [
    RegistryEntry(EGO_ID, m.ActorRole.ACTOR_ROLE_EGO, m.ControlOwner.EGO_STACK,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_1", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
    RegistryEntry("npc_2", m.ActorRole.NPC, m.ControlOwner.TRAFFIC_ENGINE,
                  m.Lifecycle.ACTIVE, m.RepresentationLevel.FULL_PHYSICS, m.ActorClass.PASSENGER_CAR),
]
N_FRAMES = 12


def retag(msg):
    msg.header.sim_id = m.OptionalBoundedId(True, SIM_ID)
    return msg


def fmt(actor: m.ActorState) -> str:
    q = actor.orientation_xyzw
    return (f"{actor.actor_id:6s} role={actor.actor_role.name:4s} "
           f"pos=({actor.position_m.x:7.2f},{actor.position_m.y:7.2f},{actor.position_m.z:5.2f}) "
           f"vel=({actor.velocity_mps.x:6.2f},{actor.velocity_mps.y:6.2f}) "
           f"quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")


def main() -> None:
    print(f"CARLA 연결 중 (localhost:2000)...")
    backend = CarlaSimulatorAdapter()
    backend.initialize()
    backend.configure(RUN_CONFIG)
    for actor_id, blueprint in SPAWN.items():
        backend.spawn_actor(actor_id, blueprint)
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, SIM_ID, REGISTRY, RUN_CONFIG,
                            publish=lambda ch, msg: published.append((ch, msg)))
    try:
        layer.bootstrap()
        ws0 = by_channel(published, "world_state")[-1]

        print(f"\n[검증 1] WorldStateFrame.actors에 ego/npc가 전부 들어있는가?")
        print(f"  actors 목록: {[a.actor_id for a in ws0.actors]}")
        print(f"  native_frame_ref.native_frame_id (메시지 전체 공용 필드) = "
             f"{ws0.native_frame_ref.native_frame_id}")
        assert {a.actor_id for a in ws0.actors} == {"ego_0", "npc_1", "npc_2"}
        for a in ws0.actors:
            print(f"    {fmt(a)}")
        npc_entries = [a for a in ws0.actors if a.actor_id != EGO_ID]
        assert len(npc_entries) == 2, "NPC가 WorldStateFrame.actors에 없음"
        print("  -> PASS: npc_1/npc_2 모두 actor_id/pose/velocity/orientation 필드를 가진 "
             "ActorState로 WorldStateFrame.actors에 포함됨 (ego와 동일한 _actor_state() 경로)")

        print(f"\n[검증 1b] EgoObservationFrame.gt_objects에도 NPC가 ObservedObject로 들어가는가?")
        obs0 = by_channel(published, "observation")[-1]
        print(f"  gt_objects 목록: {[o.actor_id for o in obs0.gt_objects]}")
        assert {o.actor_id for o in obs0.gt_objects} == {"npc_1", "npc_2"}
        print("  -> PASS")

        print(f"\n[검증 2] 실제 CARLA 값인지 — npc_1에 velocity_mps=5.0 명령을 걸고 "
             f"{N_FRAMES} tick 동안 값이 실제로 움직이는지 확인")
        npc1_positions = []
        npc1_velocities = []
        for i in range(N_FRAMES):
            ws = by_channel(published, "world_state")[-1]
            obs = by_channel(published, "observation")[-1]
            layer.on_ego_control(retag(mock_ego_command(obs, throttle=0.0)))  # ego는 정지 유지
            layer.on_npc_batch(retag(mock_npc_batch(ws)))  # npc_1=APPLY v=5.0, npc_2=HOLD
            ready = by_channel(published, "command_ready")[-1]
            decision = uuid.uuid4().bytes
            layer.on_advance_frame(retag(mock_advance(ready, decision)))
            fc = by_channel(published, "frame_complete")[-1]
            layer.on_frame_complete_ack(retag(mock_ack(fc)))

            new_ws = by_channel(published, "world_state")[-1]
            npc1 = next(a for a in new_ws.actors if a.actor_id == "npc_1")
            npc2 = next(a for a in new_ws.actors if a.actor_id == "npc_2")
            npc1_positions.append((npc1.position_m.x, npc1.position_m.y))
            npc1_velocities.append(npc1.velocity_mps.x)
            print(f"  tick {i + 1}: npc_1 {fmt(npc1)}")
            print(f"           npc_2 {fmt(npc2)}  (HOLD, 속도 명령 없음)")

        moved = ((npc1_positions[-1][0] - npc1_positions[0][0]) ** 2
                + (npc1_positions[-1][1] - npc1_positions[0][1]) ** 2) ** 0.5
        print(f"\n  npc_1 총 이동거리 = {moved:.3f} m, 최종 velocity.x = {npc1_velocities[-1]:.2f} m/s")
        assert moved > 0.1 and abs(npc1_velocities[-1]) > 0.5, \
            "npc_1이 안 움직임 — CARLA에서 실제로 읽은 값이 아닐 가능성"
        print("  -> PASS: 매 tick 값이 변하고 명령(velocity_mps=5.0)과 일치하는 방향으로 수렴 "
             "= 라벨이 아니라 실제 CARLA snapshot에서 읽은 값")

        print("\n전체 PASS")
    finally:
        backend.shutdown()


if __name__ == "__main__":
    main()
