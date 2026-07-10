# cits_rsu_msgs

C-ITS 담당 조원이 공유해준 필드 표를 그대로 옮긴 ROS2 메시지 인터페이스.
"이런 느낌으로 보낼 것 같다"는 예측을 실제로 구독 가능한 형태로 만들어서,
`cits_adapter`를 지금 바로 개발/테스트할 수 있게 하는 게 목적이다.

## 원본 표

| 변수 | 타입 | 설명 | 예시 |
|---|---|---|---|
| Object_id | Uint32 | 객체 id | 4 |
| Object_type | Uint8 | 객체 종류 | Person: 1 |
| Position_x | Float | BEV 기준 x좌표 (m) | 10.45 |
| Position_y | Float | BEV 기준 y좌표 (m) | 20.24 |
| Velocity | Float | 속도 (m/s) | 34.2 |
| timestamp | Uint64 | 객체 생성 시간 | 1782911949610 |
| Confidence | Float | 신뢰도 | 0.87 |
| heading | Float | 진행 방향 (각도) | 91.2 |

## DetectedObject.msg / DetectedObjectArray.msg

위 표를 필드명만 snake_case로 바꿔서 그대로 옮김. 배열로 감싼
`DetectedObjectArray`(header + `DetectedObject[]`)는 우리 쪽 가정이다 —
한 메시지에 여러 객체를 묶어 보낼지, 객체 하나당 메시지 하나로 보낼지는
아직 조원과 확인 안 됨.

## 확인이 필요한 가정들 (조원과 맞춰봐야 함)

- **object_type 코드**: `Person=1`만 확정. 나머지(차량 등)는 `msg`에 임시로
  `CAR=2, BICYCLE=3, MOTORCYCLE=4, TRUCK=5, BUS=6`을 넣어뒀는데 순전히 추측.
- **timestamp 단위**: 예시값(`1782911949610`)의 자릿수로 봐서 Unix epoch
  **밀리초**로 가정.
- **Position_x/y의 기준(BEV)**: RSU(카메라/센서) 위치를 원점으로 하는
  로컬 좌표로 가정. x=전방/y=좌측인지, 다른 축 규약인지는 불확실.
- **배열 vs 단건**: 여러 객체를 한 메시지에 묶어 보낼지 불확실.

이 문서를 조원한테 보여주고 실제 스펙과 맞는지 확인받는 걸 추천.
