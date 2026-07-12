# module_integrate

시스템 코어(마스터 허브, 슬라이드 4페이지 M 카드). `cits_integration`이 이미
융합해 만든 Autoware Universe 인지 결과 2종(객체, 신호등)을 내부 버퍼에
캐싱했다가 타임스탬프 기준으로 동기화해서, `r2lp1_planning_node`가 기대하는
하나의 마스터 JSON(`/integrate/chameleon_in`)으로 압축 송신하는 노드.

```
[cits_integration] --DetectedObjects, TrafficLightGroupArray--> [module_integrate_node] --/integrate/chameleon_in (JSON)--> [r2lp1_planning_node]
```

## 하는 일

1. `/perception/object_recognition/detection/objects`(`DetectedObjects`)와
   `/perception/traffic_light_recognition/traffic_signals`
   (`TrafficLightGroupArray`)를 각각 최신 메시지+수신시각으로 캐싱
2. 매 주기(`output_rate_hz`)마다:
   - 두 입력이 **둘 다** stale(수신 안 됨/오래됨) → **아예 발행하지 않음**.
     겉보기엔 "정상"인 빈 메시지를 계속 내보내는 대신,
     `r2lp1_planning_node` 자신의 `chameleon_in` staleness 체크가 안전정지를
     트리거하도록 비워둠.
   - 하나만 살아있으면 그 정보만으로 최선의 payload 생성 (신호등 없으면
     `UNKNOWN`, 객체 없으면 `hazard` 필드 생략)
   - 객체 중 자차(`ego_x`/`ego_y`, 현재 placeholder `(0,0)`)에서 가장 가까운
     것을 위험물로 선정해 거리·각도·유형 추출
   - 신호등은 여러 그룹 중 가장 위험한(RED > YELLOW > GREEN > UNKNOWN) 상태를
     채택 - 신호 하나가 stale/unknown이어도 다른 신호의 RED를 가림지 않게
3. 두 입력이 모두 fresh할 때의 시간차를 `sync_error_ms`로 계산, 지금까지의
   최댓값을 `max_sync_error_ms`로 누적해 `/system/log`에 발행 (핵심 지표:
   센서 간 시간 동기화 최대 오차)

로직(`chameleon_builder.py`)은 rclpy 의존은 없지만 `autoware_perception_msgs`
타입은 사용하므로(= `cits_integration/converters.py`와 동일한 패턴), ROS
환경에서 노드 없이 단위테스트 가능:

```bash
python3 -m pytest test/test_chameleon_builder.py -v
```

## ⚠️ 확정 안 된 가정

- **자차 위치(`ego_x`/`ego_y`)는 `(0, 0)` placeholder**. 실제 로컬라이제이션이
  붙기 전까지 위험물 거리/각도는 map 원점 기준 근사치.
- **`chameleon_in` 스키마**는 `r2lp1_planning_node`가 이미 정의해둔 것을
  그대로 따름 (`traffic_light.state`, `hazard.{distance_m,angle_deg,object_type,confidence}`).

## 실행

```bash
ros2 launch fake_rsu fake_rsu.launch.py &
ros2 launch fake_rsu fake_rsu_objects.launch.py &
ros2 launch cits_adapter cits_adapter.launch.py &
ros2 launch cits_integration cits_integration.launch.py &
ros2 launch module_integrate module_integrate.launch.py &
ros2 launch r2lp1_planning r2lp1_planning.launch.py &

ros2 topic echo /integrate/chameleon_in
ros2 topic echo /autoware/control
```

이제 `fake_rsu` → `cits_adapter` → `cits_integration` → `module_integrate` →
`r2lp1_planning_node` 전체를 손으로 메시지를 흉내내지 않고 실제로 붙여서
검증할 수 있다.
