# perception_markers

`cits_integration`이 만드는 Autoware 전용 인지 메시지(`DetectedObjects`,
`TrafficLightGroupArray`)를 Foxglove/RViz의 3D 패널이 이해하는
`visualization_msgs/MarkerArray`로 변환하는 노드. Foxglove는 이 커스텀
메시지 타입을 자동으로 3D 렌더링하지 못해서 Raw 데이터로만 보이는데, 이
노드를 붙이면 실제 객체 박스·신호등 색깔이 3D 패널에 찍힌다.

```
[cits_integration] --DetectedObjects--> [perception_markers_node] --MarkerArray--> [Foxglove/RViz 3D panel]
                   --TrafficLightGroupArray-->
```

## 하는 일

1. `/perception/object_recognition/detection/objects`를 구독해서 객체마다
   실제 pose/shape 그대로 CUBE(바운딩박스) 또는 CYLINDER 마커 + 그 위에
   타입·confidence를 보여주는 TEXT 마커를 생성. 분류(보행자/차량/hazard 등)
   별로 색을 다르게 칠함.
2. `/perception/traffic_light_recognition/traffic_signals`를 구독해서 신호
   그룹마다 SPHERE 마커 + "group N: STATE" TEXT 라벨을 생성, RED/YELLOW/GREEN에
   따라 색칠. **배치 규칙**: 모든 그룹이 같은 x/z(자차 앞 정지선 거리)에 있고,
   group id 기준 정렬 후 y축으로 `traffic_light_lane_spacing_m`(기본 4m)씩
   떨어뜨려 `traffic_light_y`를 중심으로 배치 - 즉 그룹마다 자기 차선 슬롯이
   있고, 순서가 항상 group id 기준으로 고정됨 (메시지 순서가 바뀌어도 위치
   안 바뀜).
3. 신호 그룹마다 **자기 차선에 주차된 데모용 차 마커**(`/perception/traffic_light_recognition/lane_vehicles`)도 함께 발행 -
   그 차선 신호가 RED/YELLOW면 빨간 차, GREEN이면 초록 차. 이건 실제
   `module_integrate`/`r2lp1_planning`(여러 차선 중 가장 위험한 상태 하나로
   자차 한 대를 판단)을 거치지 않는 **순수 시각화용 독립 데모**라서, "신호
   색깔 = 그 차선 차량 상태"가 파이프라인 지연이나 안전 로직 개입 없이
   바로 눈에 보인다. 진짜 자차(`fake_vehicle_node`의 ego marker)는 여러
   차선 중 가장 위험한 신호 하나만 보고 판단하므로 이 데모 차들과는 다르게
   움직일 수 있음 - 의도된 차이.
4. 마커 전부 `marker_lifetime_s`(기본 0.5초)로 짧은 lifetime을 둬서,
   다음 프레임에 사라진 객체/신호 id를 일일이 DELETE 액션으로 지우지 않고
   자동 만료되게 함.

마커 생성 로직(`marker_builder.py`)은 rclpy 의존은 없지만
`autoware_perception_msgs`/`visualization_msgs` 타입은 사용하므로(=
`chameleon_builder.py`와 동일한 패턴), ROS 환경에서 노드 없이 단위테스트
가능:

```bash
python3 -m pytest test/test_marker_builder.py -v
```

## ⚠️ 확정 안 된 가정

- **신호등 마커 위치는 고정 placeholder**(`traffic_light_x/y/z`).
  `TrafficLightGroupArray`는 그룹 id만 담고 실제 위치는 lanelet2 맵의
  regulatory element에서 가져오는 구조라, 맵이 없는 지금은 신호 상태를
  자차 앞 고정 좌표에 스피어로 띄워서 보여주는 것뿐 - 실제 신호등 위치가
  아님.
- 객체 마커는 실제 `DetectedObject`의 pose/shape를 그대로 쓰므로 위치는
  정확함 (module_integrate/cits_adapter가 만든 좌표 그대로).

## 실행

```bash
ros2 launch perception_markers perception_markers.launch.py &
```

Foxglove/RViz 3D 패널의 Topics 목록에 `/perception/object_recognition/detection/objects/markers`,
`/perception/traffic_light_recognition/traffic_signals/markers`,
`/perception/traffic_light_recognition/lane_vehicles`를 켜면 객체 박스,
신호등 색, 차선별 데모 차량이 바로 보인다.
