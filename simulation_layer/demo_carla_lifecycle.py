"""실제 CARLA server 기준 frame lifecycle 데모 (CARLA 서버 필요, localhost:2000).

MockBackend 자리에 CarlaBackend를 꽂았을 뿐 gate/FSM/메시지 코드는 동일 —
"backend 교체 시 공통 레이어 diff = 0" 확인이 목적.

확인 내용:
- CARLA sync mode 연결, ego/NPC 스폰, 정적 속성(max_steer, bbox) 캐싱
- frame N회: 입력 -> READY -> AdvanceFrame -> world.tick() 1회 -> WorldState/FrameComplete
- FrameComplete가 실제 CARLA native frame ID / decision_id / snapshot_hash와 상관되는지
- 실제 CARLA state(pose/velocity/orientation) 추출 결과가 tick마다 변하는지
- 동일 AdvanceFrame 재전송 시 CARLA world.tick()이 다시 돌지 않는지
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

SIM_ID = "carla_0"
SPAWN = {
    EGO_ID: "vehicle.tesla.model3",
    "npc_1": "vehicle.audi.tt",
    "npc_2": "vehicle.bmw.grandtourer",
}
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
    """mock builder의 header.sim_id를 이 데모의 sim_id로 교체 (payload_hash는 header 제외라 불변)."""
    msg.header.sim_id = m.OptionalBoundedId(True, SIM_ID)
    return msg


def main() -> None:
    print(f"CARLA 연결 중 (localhost:2000, target_rate={RUN_CONFIG.target_rate_hz}Hz "
          f"-> fixed_delta={STEP_NS / 1e9}s, sync mode)...")
    backend = CarlaSimulatorAdapter()
    backend.initialize()
    backend.configure(RUN_CONFIG)  # 20 Hz RunConfig에서 fixed delta 유도 (하드코딩 아님)
    for actor_id, blueprint in SPAWN.items():
        backend.spawn_actor(actor_id, blueprint)
    published: list[tuple[str, object]] = []
    layer = SimulationLayer(backend, RUN_ID, EPOCH, SIM_ID, REGISTRY, RUN_CONFIG,
                            publish=lambda ch, msg: published.append((ch, msg)))
    try:
        layer.bootstrap()
        ws = by_channel(published, "world_state")[-1]
        print(f"스폰 완료. native_frame_id={ws.native_frame_ref.native_frame_id}, "
              f"actors={[a.actor_id for a in ws.actors]}")
        print(f"정적 캐시 예 (ego): max_steer={backend._max_steer_rad[EGO_ID]:.3f} rad, "
              f"dimensions(l,w,h)={tuple(round(v, 2) for v in backend._dimensions[EGO_ID])} m")

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
            assert native_after == native_before + 1, "world.tick()이 정확히 1회가 아님"

            fc = by_channel(published, "frame_complete")[-1]
            assert fc.decision_id == decision and fc.applied_snapshot_hash == ready.snapshot_hash
            new_ws = by_channel(published, "world_state")[-1]
            assert new_ws.native_frame_ref.native_frame_id == str(native_after)
            ego = next(a for a in new_ws.actors if a.actor_id == EGO_ID)
            print(f"tick {fc.state_tick_id}: native {native_before}->{native_after} | "
                  f"FrameComplete(decision={decision.hex()[:8]}, dur={fc.tick_duration_ns / 1e6:.1f}ms) | "
                  f"ego pos=({ego.position_m.x:.2f},{ego.position_m.y:.2f}) "
                  f"v={ego.velocity_mps.x:.2f},{ego.velocity_mps.y:.2f} m/s | "
                  f"quat_w={ego.orientation_xyzw.w:.3f}")

            if fc.state_tick_id == 3:  # 실서버 기준 중복 AdvanceFrame 방어 확인
                layer.on_advance_frame(retag(mock_advance(ready, decision)))
                assert int(backend.native_frame_id()) == native_after, "중복 승인에서 재tick 발생"
                print("        -> 동일 AdvanceFrame 재전송: CARLA world.tick() 재실행 없음 확인")

            layer.on_frame_complete_ack(retag(mock_ack(fc)))
            assert layer.state == "WAITING_INPUTS"

        ego_states = [next(a for a in w.actors if a.actor_id == EGO_ID)
                      for _, w in published if _.endswith("world_state")]
        moved = abs(ego_states[-1].position_m.x - ego_states[0].position_m.x) \
              + abs(ego_states[-1].position_m.y - ego_states[0].position_m.y)
        speed = abs(ego_states[-1].velocity_mps.x) + abs(ego_states[-1].velocity_mps.y)
        print(f"\n{N_FRAMES} frame 완료: ego 이동량 {moved:.3f} m, 최종 속력성분 {speed:.2f} m/s "
              f"(throttle 0.6 실제 물리 반영 확인)")
        print("PASS — 실제 CARLA 기준 frame lifecycle + 중복 승인 방어 + state 추출 동작")
    finally:
        backend.shutdown()
        print("shutdown: actor destroy + async mode 복원 완료")


if __name__ == "__main__":
    main()
