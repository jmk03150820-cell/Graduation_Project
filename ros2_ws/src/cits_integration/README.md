# cits_integration

파이프라인의 마지막 통합 단계. `cits_adapter`가 변환해준 RSU 감지 객체와,
`fake_rsu`가 발행하는 SPAT/DENM JSON을 합쳐서 최종 Autoware Universe
perception 메시지로 발행한다.

```
[fake_rsu_objects_node] --cits_rsu_msgs--> [cits_adapter] --DetectedObjects(RSU 객체만)--\
                                                                                          +--> [cits_integration] --Autoware msgs--> Perception/Planning --> Control --> (Fake Vehicle Interface)
[fake_rsu] --SPAT/DENM JSON---------------------------------------------------------------/
```

원래는 이 노드가 fake_rsu의 CAM까지 직접 파싱해서 Autoware 메시지로
변환했지만, 팀 회의에서 **별도 Adapter 노드를 두기로 결정**하면서 그 변환
책임은 [`cits_adapter`](../cits_adapter/README.md)로 옮겼다. 이 노드는 이제
"여러 소스를 merge해서 최종 발행"만 담당한다.

`perception_ai_node`의 출력 포맷은 아직 팀원과 합의되지 않았기 때문에,
그쪽 입력은 의도적으로 빼놨다 — 나중에 포맷이 정해지면 이 노드에 구독자를
하나 더 추가하고 `_on_publish_objects_timer`에서 합치면 된다.

## 입력 / 출력

| 방향 | 토픽 | 타입 |
|---|---|---|
| 입력 | `cits_adapter/objects` | `autoware_perception_msgs/DetectedObjects` (RSU 감지 객체, 이미 변환됨) |
| 입력 | `fake_rsu/spat`, `fake_rsu/denm` | `std_msgs/String` (JSON) |
| 출력 | `/perception/object_recognition/detection/objects` | `autoware_perception_msgs/DetectedObjects` |
| 출력 | `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs/TrafficLightGroupArray` |

- `cits_adapter/objects`의 객체 리스트를 그대로 가져와서 merge (변환은 이미
  끝난 상태로 들어옴)
- DENM(가상 위험 이벤트) → `DetectedObject` (HAZARD, 정지 장애물) — 별도의
  DENM 처리 로직을 Planning에 새로 만들지 않고, 기존 장애물 회피/정지 로직이
  그대로 반응하도록 하기 위한 선택
- SPAT → `TrafficLightGroupArray` (signal_group_id를 traffic_light_group_id로
  그대로 매핑 — 실제 lanelet2 맵이 생기면 그 맵의 regulatory element id와
  맞춰줘야 함)
- MAP은 아직 변환하지 않음 (lanelet2 맵 자체를 만드는 건 별도 작업)

`cits_adapter`/DENM 각각 수신 시각을 저장해뒀다가 `object_publish_rate_hz`로
주기 발행하며, `adapter_stale_timeout_s`/`denm_stale_timeout_s` 이상 갱신이
없으면 목록에서 제외한다 (업스트림 노드가 죽거나 DENM이 비활성 상태로
돌아가면 자동으로 사라짐).

## 지오메트리 변환 (검증 완료, ROS 불필요)

`geo.py`는 위경도 → 로컬 ENU(x=동쪽, y=북쪽) 미터 변환을 담당하며 (DENM용),
ROS/Autoware 없이 순수 Python으로 단위테스트 완료:

```bash
python3 -m pytest test/test_geo.py -v
```

`map_origin_lat`/`map_origin_lon`은 `fake_rsu`의 `ref_lat`/`ref_lon`과 반드시
같은 값을 써야 좌표계가 일치한다.

## 실행

```bash
ros2 launch fake_rsu fake_rsu.launch.py &
ros2 launch fake_rsu fake_rsu_objects.launch.py &
ros2 launch cits_adapter cits_adapter.launch.py &
ros2 launch cits_integration cits_integration.launch.py &
ros2 topic echo /perception/object_recognition/detection/objects
ros2 topic echo /perception/traffic_light_recognition/traffic_signals
```

## 다음 단계

- Autoware 기본 시스템(Planning/Control) 설치 및 이 토픽들을 실제로
  구독하는지 확인
- `perception_ai_node` 출력 포맷이 정해지면 이 노드에 구독자 추가
