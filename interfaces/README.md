# AVVA Phase 1 외부 계약 번들 v0.3

이 디렉터리를 저장소의 `interfaces/`로 배치한다. 공개 wire schema의 단일 원본은
`schema/avva_phase1.idl`이며, `generated/`는 직접 수정하지 않는다.

## 포함 범위

- 16개 외부 메시지와 모든 공개 named struct
- 고정 Enum, sparse `MessageKind`, `ReasonCode`, capacity
- C++20, Python 3, fixed-layout SHM binding
- ROS 2 `rosidl_generate_interfaces` package
- logical channel/QoS/depth registry
- canonical serializer, SHA-256 reference, golden vectors

Ego 내부의 stack relay, command assembly, **Ego Common Command Validator**는
외부 계약 대상이 아니다. Validator 통과 후 Virtual 경로가 발행하는
`EgoControlCommand`와 Traffic의 `NpcControlBatch`가 Sim Backend의 **Frame
Input Gate** 공개 입력이다. Physical/RC 경로는 내부 분기 후 **RC Safety
Command Gate**로 들어간다.

## 구현자가 반드시 지킬 규칙

1. `RunKey`는 언제나 `run_id + run_epoch`로 비교한다.
2. 모든 메시지의 첫 필드는 `CommonHeader`; `payload_hash`는 header를 제외한
   application body의 canonical bytes를 SHA-256한 값이다.
3. 동일 idempotency key + 동일 hash는 side effect 없이 cached result를
   반환하고, 동일 key + 다른 hash는 `PAYLOAD_CONFLICT`로 epoch를 중단한다.
4. `FrameCompleteAck`는 transport ACK로 대체하지 않는다.
5. `NeutralControl.valid_fields_mask`가 숫자 필드의 유일한 presence 기준이다.
   bit 0(`steering_tire_angle_rad`)은 Ego `COMMAND_OK` control과 NPC
   `ITEM_OK+APPLY` control에서 모든 control mode에 필수다. 직진도 bit를 켜고
   `0.0`을 보낸다. bit가 꺼진 숫자 값은 0-normalize하고 읽거나 hash하지 않는다.
6. sequence/set 정렬과 capacity를 발행 전 검증한다. NaN/Inf, 빈 ID,
   all-zero UUID sentinel은 금지한다.
7. `detail_message`는 사람용이며 Warning/RejectNotice/StatusEvent hash에서만
   제외한다. `reason_code`와 `DetailData`는 포함한다.

## 생성 및 자체검증

```bash
make test
```

검증 내용: binding 재생성, Python syntax, golden hash 4개, C++20 compile/hash,
SHM trivially-copyable static assertion. ROS 2 환경에서는 다음을 추가 실행한다.

```bash
colcon build --packages-select avva_interfaces
```

## 파일 지도

- `schema/avva_phase1.idl`: 유일한 schema 원본
- `generated/cpp/`: application C++ binding + hash reference
- `generated/python/`: dataclass binding + hash reference
- `generated/shm/`: bounded POD binding
- `generated/ros2/`: ROS 2 interface package
- `channels/channel_registry.yaml`: 논리 채널과 delivery class
- `golden/avva_hash_v1.json`: normative vectors
- `tools/generate_bindings.py`: 재생성 도구
- `tests/`: 배포 전 smoke/golden tests

## 변경 통제

Enum/ReasonCode/MessageKind는 append-only이며 번호 삭제·재사용을 금지한다.
capacity 또는 기존 필드 의미/순서 변경은 schema major 및 ADR 검토 대상이다.
optional 필드의 후행 추가만 minor 호환 후보이며, golden vector와 계약시험을
함께 갱신해야 한다.
