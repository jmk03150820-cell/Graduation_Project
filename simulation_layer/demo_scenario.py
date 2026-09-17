"""요청 시나리오를 그대로 실행하는 데모: 구현 검증용, pytest 대상 아님.

1) EgoControlCommand(target=1) + NpcControlBatch(target=1) -> CommandSetReady(target=1)
2) AdvanceFrame(target=1, decision_id=D001) -> native step 1회 -> FrameComplete(target=1, decision_id=D001)
3) 동일 AdvanceFrame(decision_id=D001) 재전송 -> native step이 두 번 실행되지 않는지 확인
"""
from __future__ import annotations

import uuid

from simulation_layer.test_frame_lifecycle import (
    by_channel, make_layer, mock_advance, mock_ego_command, mock_npc_batch,
)

D001 = uuid.uuid5(uuid.NAMESPACE_OID, "D001").bytes  # decision_id는 Uuid128; "D001"에서 결정론적으로 파생

layer, backend, published = make_layer()

layer.bootstrap()
ws0 = by_channel(published, "world_state")[0]
obs0 = by_channel(published, "observation")[0]
print(f"[bootstrap] WorldStateFrame state_tick_id={ws0.state_tick_id}, "
      f"EgoObservationFrame state_tick_id={obs0.state_tick_id}")

print("\n=== 1) Mockup에 입력: EgoControlCommand + NpcControlBatch (target_tick=1) ===")
ego_cmd = mock_ego_command(obs0)
npc_batch = mock_npc_batch(ws0)
print(f"EgoControlCommand.target_tick_id = {ego_cmd.target_tick_id}")
print(f"NpcControlBatch.target_tick_id   = {npc_batch.target_tick_id}")

layer.on_ego_control(ego_cmd)
layer.on_npc_batch(npc_batch)

ready = by_channel(published, "command_ready")[0]
print(f"-> CommandSetReady.target_tick_id = {ready.target_tick_id}  "
      f"(completeness={ready.completeness.name}, validity={ready.validity.name})")
assert ready.target_tick_id == 1

print("\n=== 2) Mockup Core에 입력: AdvanceFrame (target_tick=1, decision_id=D001) ===")
adv1 = mock_advance(ready, D001)
print(f"AdvanceFrame.target_tick_id = {adv1.target_tick_id}, decision_id = D001 ({D001.hex()})")

native_before = int(backend.native_frame_id())
layer.on_advance_frame(adv1)
native_after = int(backend.native_frame_id())
print(f"NativeAdapter.step() 호출 후 native_frame_id: {native_before} -> {native_after} "
      f"({'정확히 1회' if native_after - native_before == 1 else '오류'})")

fc1 = by_channel(published, "frame_complete")[-1]
print(f"-> FrameComplete.state_tick_id = {fc1.state_tick_id}, decision_id = D001 "
      f"({fc1.decision_id == D001})")
assert fc1.state_tick_id == 1 and fc1.decision_id == D001 and native_after - native_before == 1

print("\n=== 3) 동일 AdvanceFrame(decision_id=D001) 재전송 ===")
adv2 = mock_advance(ready, D001)  # 같은 target_tick_id, 같은 decision_id
layer.on_advance_frame(adv2)
native_after_retry = int(backend.native_frame_id())
fc_count = len(by_channel(published, "frame_complete"))
fc2 = by_channel(published, "frame_complete")[-1]
print(f"native_frame_id: {native_after} -> {native_after_retry} "
      f"({'변화 없음, step 재실행 안 됨' if native_after_retry == native_after else '오류: step이 다시 실행됨'})")
print(f"FrameComplete 발행 횟수: {fc_count} (재전송이 포함되어 2여야 정상), "
      f"재전송 객체가 최초와 동일한가: {fc2 is fc1}")
assert native_after_retry == native_after, "native step이 두 번째 AdvanceFrame에서 다시 실행됨"
assert fc2 is fc1, "재전송된 FrameComplete가 최초 결과와 다름 (재실행됐을 가능성)"

print("\n=== 결과: PASS — decision_id=D001 두 번 수신에도 native step은 정확히 1회만 실행됨 ===")
