# fake_vehicle

실차가 연결되지 않은 구간에서 `vehicle_interface_node`의 출력(`/vehicle/command`)을
받아 하드웨어 없이 차량의 물리적 반응(가감속·조향 rate limit)을 시뮬레이션하고,
`/vehicle/raw_status`로 피드백을 돌려주는 노드. 슬라이드 6페이지 설계 문서 기준
Phase 1 대체 노드.

```
[r2lp1_planning_node] --/autoware/control--> [vehicle_interface_node] --/vehicle/command--> [fake_vehicle_node] --/vehicle/raw_status--> [vehicle_interface_node]
```

## 하는 일

1. `/vehicle/command`(JSON: `target_speed_kmh`, `target_steer_deg`)를 구독
2. 매 주기(`publish_rate_hz`)마다 `max_accel_mps2`/`max_decel_mps2`/
   `max_steer_rate_deg_s`로 제한된 속도로 목표값에 접근 (즉시 도달 X, 관성 흉내)
3. 명령이 없거나 오래됐으면(`command_stale_timeout_s` 초과) 목표를 강제로
   속도 0·조향 0으로 두어 페일세이프 감속 (r2lp1_planning_node, cits_adapter와
   동일한 안전 디폴트 컨벤션)
4. 결과를 `/vehicle/raw_status`(JSON: `speed_kmh`, `steer_deg`, `timestamp`)로 발행

물리 시뮬레이션(`vehicle_simulator.py`)은 rclpy 의존성이 없어 ROS 없이
단위테스트 가능:

```bash
python3 -m pytest test/test_vehicle_simulator.py -v
```

## ⚠️ 확정 안 된 가정 (vehicle_interface_node 구현 시 확인 필요)

- **`vehicle_interface_node` 자체가 아직 없음** (신무웅 담당, 미구현). 이 노드는
  그 출력 규격을 아래처럼 추측해서 만듦 - 실제 노드가 나오면 스키마를 맞추거나
  이쪽 파싱 로직을 고쳐야 함.
  ```json
  {"target_speed_kmh": 20.0, "target_steer_deg": -5.0}
  ```
- **`/vehicle/raw_status`의 JSON 스키마**도 같은 이유로 placeholder
  (`speed_kmh`, `steer_deg`, `timestamp`).
- **가감속/조향 rate 한계값**은 전부 임의 기본값이며 실제 차량 사양과 무관.

## 실행

```bash
ros2 launch fake_vehicle fake_vehicle.launch.py &
ros2 topic echo /vehicle/raw_status
```

`vehicle_interface_node`가 없는 동안은 아래처럼 수동으로 `/vehicle/command`를
흉내내서 시뮬레이션 반응을 확인할 수 있다:

```bash
ros2 topic pub /vehicle/command std_msgs/msg/String \
  '{data: "{\"target_speed_kmh\": 20.0, \"target_steer_deg\": -5.0}"}' -r 10
```
