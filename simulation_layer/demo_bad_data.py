"""'진짜 데이터가 들어가서 실제로 검증/hash 되는지' + '통과 못하는 데이터는 어떻게 처리되는지' 증명용 데모.

pytest 대상 아님. 4개 시나리오를 순서대로 실행하고 각 단계의 실제 필드값/hash를 출력한다.
"""
from __future__ import annotations

import uuid

import avva_phase1 as m
from simulation_layer import hashing
from simulation_layer.gate import validate_neutral_control
from simulation_layer.test_frame_lifecycle import (
    by_channel, make_layer, mock_advance, mock_ego_command, mock_npc_batch,
)

SEP = "=" * 78


# ---------------------------------------------------------------------------
# 0) 이 hash가 텍스트가 아니라 실제 직렬화 결과라는 증거:
#    gate.py 내부에서 만든 CommandSetReady.header.payload_hash를,
#    여기서 hashing.py로 "독립적으로 다시" 계산해서 바이트 단위로 비교한다.
#    한 글자라도 필드가 다르면 SHA-256이라 절대 우연히 같을 수 없다.
# ---------------------------------------------------------------------------
print(SEP)
print("0) hash가 진짜 계산된 것인지 증명: CommandSetReady를 독립적으로 재계산해 비교")
print(SEP)

layer, backend, published = make_layer()
layer.bootstrap()
ws0 = by_channel(published, "world_state")[0]
obs0 = by_channel(published, "observation")[0]

layer.on_ego_control(mock_ego_command(obs0))
layer.on_npc_batch(mock_npc_batch(ws0))
ready = by_channel(published, "command_ready")[0]

stored_hash = ready.header.payload_hash
recomputed_hash = hashing.command_set_ready_hash(ready)  # 메시지 필드로부터 처음부터 재계산
print(f"gate.py가 메시지에 박아넣은 payload_hash : {stored_hash.hex()}")
print(f"여기서 필드값만 갖고 독립적으로 재계산  : {recomputed_hash.hex()}")
print(f"일치 여부: {stored_hash == recomputed_hash}")
assert stored_hash == recomputed_hash
print("-> 값이 정말 CommandSetReady의 실제 필드(target_tick_id, counts, "
      "control_set_meta, completeness, validity, snapshot_hash ...)로부터 계산된 것임을 확인.\n"
      "   (라벨을 찍은 게 아니라, 바이트가 하나라도 다르면 이 두 hash는 절대 같을 수 없다.)")

# 필드값 자체도 실제 숫자/enum임을 보여준다 (문자열이 아니라 진짜 CommandCounts 구조체)
print(f"\nCommandSetReady.counts (실제 구조체) = {ready.counts}")
print(f"CommandSetReady.snapshot_hash        = {ready.snapshot_hash.hex()}")


# ---------------------------------------------------------------------------
# 1) 위법 데이터: NPC 하나가 throttle과 brake를 동시에 양수로 보냄
#    -> 기준서 §6.6 "throttle과 brake 동시 양수는 금지" 위반
#    -> Frame Input Gate가 실제로 이 값을 검증해서 걸러내는지 확인
# ---------------------------------------------------------------------------
print("\n" + SEP)
print("1) 위법 데이터: NpcControlItem에 throttle=0.5, brake=0.5 동시 양수")
print(SEP)

layer2, backend2, published2 = make_layer()
layer2.bootstrap()
ws0_2 = by_channel(published2, "world_state")[0]
obs0_2 = by_channel(published2, "observation")[0]

bad_ctrl = m.NeutralControl(
    control_mode=m.ControlMode.DIRECT_ACTUATION,
    valid_fields_mask=(1 << 0) | (1 << 5) | (1 << 6),
    steering_tire_angle_rad=0.0, steering_tire_rotation_rate_rad_s=0.0,
    velocity_mps=0.0, acceleration_mps2=0.0, jerk_mps3=0.0,
    throttle=0.5, brake=0.5, gear=m.Gear.GEAR_UNKNOWN, hand_brake=False)  # 위법 조합

# 단독 검증 함수로도 실제로 거부되는지 먼저 확인
code = validate_neutral_control(bad_ctrl, require_steer=True)
print(f"validate_neutral_control(throttle=0.5, brake=0.5) -> reason_code = {code} "
      f"({'INVALID_COMMAND_COMBINATION' if code == m.INVALID_COMMAND_COMBINATION else '??'})")
assert code == m.INVALID_COMMAND_COMBINATION

good_batch = mock_npc_batch(ws0_2)
bad_item = m.NpcControlItem("npc_1", m.CommandAction.APPLY, m.ItemStatus.ITEM_OK,
                            m.OptionalNeutralControl(True, bad_ctrl), m.UNKNOWN_REASON)
items = sorted([bad_item, good_batch.items[1]], key=lambda i: i.actor_id)
npc_batch_bad = m.NpcControlBatch(
    header=None, based_on_tick_id=good_batch.based_on_tick_id, target_tick_id=good_batch.target_tick_id,
    control_set_meta=good_batch.control_set_meta, items=items, lifecycle_intents=[],
    engine_step_id=good_batch.engine_step_id, engine_sim_time_ns=good_batch.engine_sim_time_ns,
    engine_step_duration_ns=good_batch.engine_step_duration_ns)
npc_batch_bad.header = good_batch.header  # 헤더는 그대로 두고 payload_hash만 새로 계산
npc_batch_bad.header.payload_hash = hashing.npc_control_batch_hash(npc_batch_bad)

layer2.on_ego_control(mock_ego_command(obs0_2))
layer2.on_npc_batch(npc_batch_bad)

ready_bad = by_channel(published2, "command_ready")[0]
print(f"\n실제로 gate를 통과시킨 결과 CommandSetReady:")
print(f"  completeness = {ready_bad.completeness.name}")
print(f"  validity     = {ready_bad.validity.name}  (VALID이면 안 됨)")
print(f"  actor_reasons = {[(r.actor_id, r.reason_code) for r in ready_bad.actor_reasons]}")
print(f"  counts.failed_traffic = {ready_bad.counts.failed_traffic}")
assert ready_bad.validity == m.Validity.INVALID
assert any(r.actor_id == "npc_1" and r.reason_code == m.INVALID_COMMAND_COMBINATION
          for r in ready_bad.actor_reasons)
print("-> gate가 개별 필드를 실제로 파싱해서 위법 조합을 잡아냈다 (validity=INVALID).")
print("   기준서 §7.1.6: Core는 validity=INVALID면 ADVANCE를 보내지 않고 ABORT/재시도로 처리해야 함.")
print(f"   native step은 아직 한 번도 안 돎: native_frame_id = {backend2.native_frame_id()}")
assert backend2.native_frame_id() == "0"


# ---------------------------------------------------------------------------
# 2) 위법 데이터: 같은 target_tick에 대해 내용이 다른 EgoControlCommand 재전송
#    -> 기준서 §4.6/§8.4 "동일 key + 다른 hash = PAYLOAD_CONFLICT, frame ABORTED"
# ---------------------------------------------------------------------------
print("\n" + SEP)
print("2) 위법 데이터: 같은 target_tick_id=1에 대해 내용이 다른 EgoControlCommand 재전송 (payload 위조/충돌)")
print(SEP)

layer3, backend3, published3 = make_layer()
layer3.bootstrap()
ws0_3 = by_channel(published3, "world_state")[0]
obs0_3 = by_channel(published3, "observation")[0]

cmd_v1 = mock_ego_command(obs0_3, throttle=0.3)
cmd_v2 = mock_ego_command(obs0_3, throttle=0.9)  # 같은 target_tick, 다른 throttle -> 다른 payload_hash
print(f"cmd_v1.control.throttle = {cmd_v1.control.value.throttle}, payload_hash = {cmd_v1.header.payload_hash.hex()[:16]}...")
print(f"cmd_v2.control.throttle = {cmd_v2.control.value.throttle}, payload_hash = {cmd_v2.header.payload_hash.hex()[:16]}...")
assert cmd_v1.header.payload_hash != cmd_v2.header.payload_hash

layer3.on_ego_control(cmd_v1)
print(f"\n1차 제출 후 상태: {layer3.state}")
layer3.on_ego_control(cmd_v2)
print(f"2차 제출(다른 내용, 같은 target_tick) 후 상태: {layer3.state}")
print(f"마지막 evidence 이벤트: {layer3.events[-1]}")
assert layer3.state == "ABORTED"
assert layer3.events[-1]["reason_code"] == m.PAYLOAD_CONFLICT
print("-> 실제로 ABORTED로 전이됐고 reason_code=PAYLOAD_CONFLICT가 기록됨.")

layer3.on_npc_batch(mock_npc_batch(ws0_3))
print(f"ABORTED 상태에서 NpcControlBatch를 추가로 보내도: "
      f"CommandSetReady 발행 수 = {len(by_channel(published3, 'command_ready'))}, "
      f"native_frame_id = {backend3.native_frame_id()}")
assert by_channel(published3, "command_ready") == []
assert backend3.native_frame_id() == "0"
print("-> ABORTED 이후 어떤 입력도 진행되지 않고, native step도 끝까지 한 번도 안 돎.")


# ---------------------------------------------------------------------------
# 3) 위법 데이터: AdvanceFrame의 snapshot_hash를 위조해서 보냄
#    -> READY 시점에 얼린 snapshot과 다르면 apply/tick을 걸지 않고 abort
# ---------------------------------------------------------------------------
print("\n" + SEP)
print("3) 위법 데이터: AdvanceFrame.snapshot_hash를 위조 (실제 READY snapshot과 불일치)")
print(SEP)

layer4, backend4, published4 = make_layer()
layer4.bootstrap()
ws0_4 = by_channel(published4, "world_state")[0]
obs0_4 = by_channel(published4, "observation")[0]
layer4.on_ego_control(mock_ego_command(obs0_4))
layer4.on_npc_batch(mock_npc_batch(ws0_4))
ready4 = by_channel(published4, "command_ready")[0]

forged = mock_advance(ready4, uuid.uuid4().bytes)
real_hash = forged.snapshot_hash
forged.snapshot_hash = bytes((real_hash[0] ^ 0xFF,)) + real_hash[1:]  # 1바이트 위조
print(f"gate가 READY에서 얼린 진짜 snapshot_hash : {real_hash.hex()[:20]}...")
print(f"AdvanceFrame에 실려온(위조된) snapshot_hash: {forged.snapshot_hash.hex()[:20]}...")

layer4.on_advance_frame(forged)
print(f"\n처리 결과 상태: {layer4.state}")
print(f"마지막 evidence 이벤트: {layer4.events[-1]}")
print(f"native_frame_id = {backend4.native_frame_id()} (0이어야 정상 — tick이 걸리면 안 됨)")
assert layer4.state == "ABORTED"
assert layer4.events[-1]["reason_code"] == m.SNAPSHOT_HASH_MISMATCH
assert backend4.native_frame_id() == "0"
print("-> 위조된 snapshot_hash는 native tick으로 이어지지 않고 즉시 ABORTED됨.")

print("\n" + SEP)
print("전부 PASS — hash는 실제 필드 재계산으로 검증됐고,")
print("위법 데이터 3종(NPC 명령 위반 / payload 충돌 / snapshot 위조) 모두 실제로 거부·abort됨.")
print(SEP)
