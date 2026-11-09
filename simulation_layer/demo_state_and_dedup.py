"""요청 시나리오 3종 데모. pytest 대상 아님.

1) EgoControlCommand + NpcControlBatch 입력 -> 상태머신 변화(state transition) 확인
2) EgoControlCommand를 "동일 내용"으로 2회 연속 전송 -> PAYLOAD_CONFLICT가 나면 안 됨
   (내용이 같으면 §8.4 DUPLICATE_IDEMPOTENT — conflict는 "다른 내용"일 때만)
3) AdvanceFrame(decision_id=D001, snapshot_hash=A) -> native step 1회
   -> 동일 AdvanceFrame 재전송 -> native step count가 그대로인지 확인
"""
from __future__ import annotations

import uuid

import avva_phase1 as m
from simulation_layer.test_frame_lifecycle import (
    by_channel, make_layer, mock_advance, mock_ego_command, mock_npc_batch,
)

SEP = "=" * 78
D001 = uuid.uuid5(uuid.NAMESPACE_OID, "D001").bytes

layer, backend, published = make_layer()

print(SEP)
print("0) bootstrap: 최초 상태")
print(SEP)
print(f"layer.state = {layer.state!r}")
layer.bootstrap()
ws0 = by_channel(published, "world_state")[0]
obs0 = by_channel(published, "observation")[0]
print(f"bootstrap 후 state_tick_id = {layer.state_tick}, layer.state = {layer.state!r}")


# ---------------------------------------------------------------------------
# 1) EgoControlCommand + NpcControlBatch 입력 -> 상태머신 변화 확인
# ---------------------------------------------------------------------------
print("\n" + SEP)
print("1) EgoControlCommand + NpcControlBatch 입력에 따른 상태머신 변화")
print(SEP)

ego_cmd = mock_ego_command(obs0)
npc_batch = mock_npc_batch(ws0)

print(f"입력 전                        : layer.state = {layer.state!r}")
layer.on_ego_control(ego_cmd)
print(f"EgoControlCommand 제출 후       : layer.state = {layer.state!r}  "
      f"(NpcControlBatch가 아직 없어 READY로는 못 감)")
layer.on_npc_batch(npc_batch)
print(f"NpcControlBatch 제출 후         : layer.state = {layer.state!r}  "
      f"(둘 다 도착 -> READY, CommandSetReady 발행)")
ready = by_channel(published, "command_ready")[0]
assert layer.state == "READY"
print(f"-> CommandSetReady 발행: target_tick_id={ready.target_tick_id}, "
      f"completeness={ready.completeness.name}, validity={ready.validity.name}")


# ---------------------------------------------------------------------------
# 2) EgoControlCommand를 "동일 내용"으로 2회 연속 전송 -> PAYLOAD_CONFLICT가 나면 안 됨
#    (지난 데모에서 보여준 "다른 내용"일 때의 PAYLOAD_CONFLICT와 대비)
# ---------------------------------------------------------------------------
print("\n" + SEP)
print('2) EgoControlCommand "동일 내용" 2회 연속 전송 -> PAYLOAD_CONFLICT가 나면 안 됨')
print(SEP)

layer2, backend2, published2 = make_layer()
layer2.bootstrap()
ws0_2 = by_channel(published2, "world_state")[0]
obs0_2 = by_channel(published2, "observation")[0]

same_cmd = mock_ego_command(obs0_2, throttle=0.3)
print(f"1차 전송: payload_hash = {same_cmd.header.payload_hash.hex()[:16]}...")
layer2.on_ego_control(same_cmd)
state_after_1st = layer2.state
print(f"1차 전송 후 state = {state_after_1st!r}")

print(f"2차 전송(완전히 동일한 메시지 객체, 같은 payload_hash): "
      f"{same_cmd.header.payload_hash.hex()[:16]}...")
layer2.on_ego_control(same_cmd)  # 완전 동일 내용 재전송
state_after_2nd = layer2.state
last_event = layer2.events[-1]
print(f"2차 전송 후 state = {state_after_2nd!r}")
print(f"마지막 evidence 이벤트 = {last_event}")

assert state_after_2nd != "ABORTED", "동일 내용인데 ABORTED가 나면 버그"
assert last_event["reason_code"] != m.PAYLOAD_CONFLICT, "동일 내용인데 PAYLOAD_CONFLICT가 나면 버그"
assert last_event["reason_code"] == m.DUPLICATE_IDEMPOTENT
print(f"-> PAYLOAD_CONFLICT(reason_code={m.PAYLOAD_CONFLICT}) 아님, "
      f"DUPLICATE_IDEMPOTENT(reason_code={m.DUPLICATE_IDEMPOTENT})로 처리됨. state는 COLLECTING 유지.")

# 이어서 NpcControlBatch까지 넣어서 정상적으로 READY까지 계속 진행되는지도 확인
layer2.on_npc_batch(mock_npc_batch(ws0_2))
print(f"이어서 NpcControlBatch 제출 후 state = {layer2.state!r} "
      f"(중복 수신이 있었어도 정상적으로 READY까지 진행됨을 확인)")
assert layer2.state == "READY"


# ---------------------------------------------------------------------------
# 3) AdvanceFrame(decision_id=D001, snapshot_hash=A) -> native step 1회
#    -> 동일 AdvanceFrame 재전송 -> native step count 확인
# ---------------------------------------------------------------------------
print("\n" + SEP)
print("3) AdvanceFrame(decision_id=D001, snapshot_hash=A) 1회 적용 후 동일 메시지 재전송")
print(SEP)

ready3 = by_channel(published2, "command_ready")[0]
snapshot_hash_A = ready3.snapshot_hash
adv = mock_advance(ready3, D001)
print(f"decision_id = D001 ({D001.hex()[:16]}...)")
print(f"snapshot_hash = A ({snapshot_hash_A.hex()[:16]}...)")

native_count_before = int(backend2.native_frame_id())
print(f"\n1차 AdvanceFrame 적용 전 native step count = {native_count_before}")
layer2.on_advance_frame(adv)
native_count_after_1st = int(backend2.native_frame_id())
print(f"1차 AdvanceFrame 적용 후 native step count = {native_count_after_1st}")
assert native_count_after_1st - native_count_before == 1

print("\n동일 AdvanceFrame(decision_id=D001, snapshot_hash=A) 재전송...")
layer2.on_advance_frame(adv)  # 완전히 동일한 메시지 재전송
native_count_after_2nd = int(backend2.native_frame_id())
print(f"재전송 후 native step count = {native_count_after_2nd}")

assert native_count_after_2nd == native_count_after_1st, "재전송인데 step count가 늘어나면 버그"
print(f"-> native step count가 {native_count_after_1st}에서 그대로 유지됨. "
      f"재전송이 native step을 다시 실행시키지 않음을 확인.")

print("\n" + SEP)
print("전부 PASS")
print(SEP)
