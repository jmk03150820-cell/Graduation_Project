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
5. 시뮬레이션 속도·조향으로 map 프레임 위치를 dead-reckoning 적분해서
   `map -> base_link` TF와 `/fake_vehicle/ego_marker`(MarkerArray, 차량 크기
   CUBE)를 발행 - 정지 중이면 빨간색, 움직이면 초록색이라서 RViz/Foxglove
   3D 패널에서 "신호등 RED일 때 실제로 멈추는지"를 눈으로 확인할 수 있음

### ⚠️ TEMPORARY: `/autoware/control` 직접 구독 (vehicle_interface_node 없음)

원래 데이터 흐름은 `r2lp1_planning_node → vehicle_interface_node → fake_vehicle_node`
인데, `vehicle_interface_node`가 아직 없어서 `/vehicle/command`가 전혀 발행되지
않는다 - 즉 지금까지 이 노드는 판단 결과를 한 번도 못 받고 항상 안전정지
상태였다. 그래서 `/vehicle/command`가 stale일 때만 `/autoware/control`
(Twist)을 직접 받아서 단위만 변환(m/s→km/h, rad/s→deg/s)해 대신 씀 -
`vehicle_interface_node`의 변환 역할을 아주 단순하게 흉내낸 것뿐이고
리미터 등 안전장치는 없음. **`vehicle_interface_node`가 실제로 생기면 이
fallback(`_on_autoware_control`, `autoware_control_topic` 파라미터)은
지워야 함.**

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
- **위치 적분(dead-reckoning)은 단순화된 모델**: `steer_deg`를 실제 조향각이
  아니라 그냥 요레이트(yaw rate, deg/s)로 취급해서 x/y/yaw를 적분함. 자전거
  모델(bicycle model)이 아니라서 궤적 정확도는 없고, "움직이다가 멈추는지"를
  시각적으로 보여주는 용도로만 충분함.
- 이 노드가 `map -> base_link` TF를 직접 발행하므로, 별도의
  `static_transform_publisher`를 띄워둔 게 있다면 충돌하니 꺼야 함.

## 실행

```bash
ros2 launch fake_vehicle fake_vehicle.launch.py &
ros2 topic echo /vehicle/raw_status
ros2 topic echo /fake_vehicle/ego_marker
```

`vehicle_interface_node`가 없는 동안은 아래처럼 수동으로 `/vehicle/command`를
흉내내서 시뮬레이션 반응을 확인할 수 있다:

```bash
ros2 topic pub /vehicle/command std_msgs/msg/String \
  '{data: "{\"target_speed_kmh\": 20.0, \"target_steer_deg\": -5.0}"}' -r 10
```
