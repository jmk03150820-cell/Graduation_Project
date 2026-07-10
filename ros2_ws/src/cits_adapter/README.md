# cits_adapter

C-ITS RSU가 보내줄 것으로 예측되는 감지 객체 데이터
([`cits_rsu_msgs`](../cits_rsu_msgs/README.md))를 구독해서, 필요한 값만
추출·가공해 Autoware Universe `DetectedObjects`로 변환하는 Adapter 노드.

```
[C-ITS RSU (실제 또는 fake_rsu_objects_node)] --cits_rsu_msgs/DetectedObjectArray--> [cits_adapter] --autoware_perception_msgs/DetectedObjects--> [cits_integration]
```

## 하는 일

1. `object_type` 코드 → Autoware `ObjectClassification` 라벨 매핑
   (`Person=1` → `PEDESTRIAN`, 나머지는 추측)
2. BEV(RSU 로컬) 좌표 `position_x/y` → `map` 프레임 좌표로 회전+이동 변환
   (RSU 자신의 map 프레임상 위치/방향 `rsu_x`/`rsu_y`/`rsu_yaw_deg` 파라미터 사용)
3. `heading` → `map` 프레임 yaw → 쿼터니언
4. `confidence` → `existence_probability` 그대로
5. `confidence`가 너무 낮거나(`min_confidence`), `timestamp`가 너무
   오래됐으면(`max_object_age_s`) 걸러냄
6. 타입별 기본 바운딩박스 크기 적용 (표에 크기 필드가 없어서)

## ⚠️ 확정 안 된 가정 (조원 확인 필요)

- **RSU의 BEV 좌표계 원점/축 방향**: RSU 위치를 원점으로 하는 로컬 좌표에
  x=전방/y=좌측(ROS REP-103 바디 프레임)이라고 가정. 실제 카메라/센서
  장착 방향에 따라 축이 다를 수 있음.
- **RSU의 map 프레임 위치**: `rsu_x`/`rsu_y`/`rsu_yaw_deg` 파라미터가 전부
  기본값 `(0, 0, 0)`인 placeholder. 실제 RSU 설치 위치/방향을 측량해서
  넣어야 좌표가 실제 지도와 맞음.
- **object_type 코드 매핑**: `Person=1`만 확정, 나머지는
  [`cits_rsu_msgs`](../cits_rsu_msgs/README.md)에 정리된 추측값.
- **timestamp 단위**: Unix epoch 밀리초로 가정.

## 검증 완료

`ros2 interface show`로 대조한 실제 Autoware 메시지 필드 기준으로 작성,
`fake_rsu`의 `fake_rsu_objects_node`(같은 예측 스키마로 가짜 데이터 발행)와
엮어서 colcon build + 실행 + 좌표 변환 결과까지 직접 계산해서 대조 확인함.
지오메트리 변환(`geo.py`)은 ROS 없이도 단위테스트 가능:

```bash
python3 -m pytest test/test_geo.py -v
```

## 실행

```bash
ros2 launch fake_rsu fake_rsu_objects.launch.py &   # 테스트용 가짜 데이터
ros2 launch cits_adapter cits_adapter.launch.py &
ros2 topic echo /cits_adapter/objects
```

실제 조원의 RSU 노드가 `cits/rsu/detected_objects` 토픽(파라미터로 변경
가능)에 `cits_rsu_msgs/DetectedObjectArray`를 발행하기 시작하면,
`fake_rsu_objects_node` 없이 바로 연결된다.
