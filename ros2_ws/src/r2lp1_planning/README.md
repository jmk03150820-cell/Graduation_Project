# r2lp1_planning

마스터 허브(`module_integrate_node`)가 만들 동기화 데이터(`/integrate/chameleon_in`)를
받아 차량의 목표 속도·조향을 판단해 `/autoware/control`로 내보내는 판단제어 노드.
Phase 1은 강화학습 이전 단계로, 단순 if-else 로직으로 파이프라인 동작을 먼저
검증한다 (`fake_planning_node`를 따로 만들 필요 없이 이 노드 자체가 Phase 1 역할을 겸함).

```
[module_integrate_node] --/integrate/chameleon_in (JSON)--> [r2lp1_planning_node] --/autoware/control (Twist)--> [Autoware Control]
                                                                      |
                                                                      +--/system/log (JSON)--> [system_logger_node]
```

## 하는 일 (Phase 1 if-else 우선순위)

1. `chameleon_in` 수신이 없거나(`data_stale_timeout_s` 초과) JSON 파싱 실패 →
   안전 디폴트(정지: `linear.x=0, angular.z=0`) 강제 송신
2. 신호등 상태가 `RED`/`YELLOW` → 정지
3. 위험 객체(`hazard`) 거리가 `emergency_stop_distance_m` 이내 → 정지
4. `slow_down_distance_m` 이내 → 거리에 비례해 `min_speed_mps`~`target_speed_mps` 사이로
   감속, 객체 각도(`angle_deg`) 반대 방향으로 약한 조향
5. 그 외 → `target_speed_mps`로 순항

판단 로직(`decision_logic.py`)은 rclpy 의존성이 없어 ROS 없이 단위테스트 가능하고,
Phase 2에서 강화학습 정책으로 교체할 때도 노드의 ROS 배선은 그대로 둘 수 있다.

```bash
python3 -m pytest test/test_decision_logic.py -v
```

## ⚠️ 확정 안 된 가정 (module_integrate_node 구현 시 확인 필요)

- **`/integrate/chameleon_in`의 JSON 스키마**: `module_integrate_node`가 아직 없어서
  아래 형태로 추측해 만듦. M을 실제로 구현할 때 이 스키마에 맞추거나, 여기 파싱
  로직을 스키마에 맞게 고쳐야 함.
  ```json
  {
    "traffic_light": {"state": "RED|YELLOW|GREEN|UNKNOWN"},
    "hazard": {"distance_m": 12.3, "angle_deg": -5.0, "object_type": "pedestrian"}
  }
  ```
- **`/autoware/control`을 `geometry_msgs/Twist`로 낸다는 것**: 슬라이드 설계상의
  단순화. 실제 Autoware Control 모듈과 맞물리려면 `AckermannControlCommand` 등으로
  바꿔야 할 수 있음.
- **감속/조향 파라미터 값**(`emergency_stop_distance_m` 등)은 전부 임의 기본값.

## 실행

```bash
ros2 launch r2lp1_planning r2lp1_planning.launch.py &
ros2 topic echo /autoware/control
ros2 topic echo /system/log
```

`module_integrate_node`가 없는 동안은 아래처럼 수동으로 `chameleon_in`을 흉내내서
판단 로직을 확인할 수 있다:

```bash
ros2 topic pub /integrate/chameleon_in std_msgs/msg/String \
  '{data: "{\"traffic_light\": {\"state\": \"GREEN\"}, \"hazard\": {\"distance_m\": 8.0, \"angle_deg\": 10.0}}"}' -r 5
```
