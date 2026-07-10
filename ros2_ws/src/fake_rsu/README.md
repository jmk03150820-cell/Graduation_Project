# fake_rsu

파이프라인의 첫 단계인 **Fake C-ITS msg** 생성 노드.

```
[fake_rsu] --(JSON C-ITS msg)--> Autoware Perception/Planning Adapter --> Perception / Planning / Control --> (Fake Vehicle Interface)
```

실제 RSU가 브로드캐스트하는 CAM/DENM/SPAT/MAP 메시지를 ASN.1/UPER 대신
**단순화된 JSON**으로 발행한다. 필드명은 실제 ETSI 메시지 개념에 최대한
맞춰서, 이후 Adapter 노드가 매핑하기 쉽도록 했다.

## 토픽

| 토픽 | 타입 | 주기(기본값) | 내용 |
|---|---|---|---|
| `/fake_rsu/map` | `std_msgs/String` (JSON) | 0.2 Hz | 교차로 지오메트리 (4방향 진입/진출 차로) |
| `/fake_rsu/spat` | `std_msgs/String` (JSON) | 2 Hz | 신호 현시 (NS/EW 2현시, green/yellow/red) |
| `/fake_rsu/cam` | `std_msgs/String` (JSON) | 10 Hz | 가상 차량 1대가 접근로를 왕복 |
| `/fake_rsu/denm` | `std_msgs/String` (JSON) | 이벤트 활성 시에만 | 주기적으로 발생하는 가상 위험 이벤트 (기본: roadworks) |

메시지 스키마는 [`fake_rsu/message_generator.py`](fake_rsu/message_generator.py)의
`generate_map/spat/cam/denm`을 참고. `DENM`은 실제 규격처럼 이벤트가 활성
상태일 때만 발행되고, 비활성 구간에는 아무것도 보내지 않는다.

## 빌드 & 실행 (WSL / ROS 2 Humble)

이 폴더는 OneDrive로 동기화되는 Windows 경로에 있다. colcon 빌드는
DrvFs(`/mnt/c/...`)에서도 되지만 느리므로, 네이티브 WSL 파일시스템에 복사해서
빌드하는 것을 권장한다.

```bash
mkdir -p ~/ros2_ws/src
cp -r "/mnt/c/Users/jmk/OneDrive - 한국공학대학교/바탕 화면/졸작 노드/ros2_ws/src/fake_rsu" ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fake_rsu
source install/setup.bash

ros2 launch fake_rsu fake_rsu.launch.py
# 또는 직접 파라미터 파일 지정:
ros2 run fake_rsu fake_rsu_node --ros-args --params-file src/fake_rsu/config/fake_rsu.yaml
```

확인:

```bash
ros2 topic echo /fake_rsu/spat
ros2 topic echo /fake_rsu/cam
ros2 topic hz /fake_rsu/cam
```

## 파라미터 (`config/fake_rsu.yaml`)

- `ref_lat` / `ref_lon`: 가상 교차로 기준 좌표 (기본값은 임의 좌표이므로 실제
  테스트 환경/맵 원점에 맞게 교체할 것)
- `green_s` / `yellow_s` / `all_red_s`: SPAT 신호 주기
- `denm_interval_s` / `denm_duration_s` / `denm_cause`: DENM 발생 주기, 지속
  시간, 원인 코드
- `cam_rate_hz` / `spat_rate_hz` / `map_rate_hz` / `denm_check_rate_hz`: 각
  토픽 발행 주기

## 단위 테스트

`message_generator.py`는 rclpy 의존성이 없어 ROS 2 없이도 테스트 가능:

```bash
python3 -m pytest test/test_message_generator.py -v
```

## 다음 단계

- Adapter 노드: 이 JSON 메시지들을 구독해서 Autoware
  Perception(`autoware_perception_msgs`)/Planning 입력(예: 신호 정보,
  장애물/이벤트)으로 변환
- 필요 시 CAM을 Autoware `PredictedObjects`/`DetectedObjects`로, SPAT/MAP을
  신호 정보 메시지로 변환하는 매핑 로직 설계
- Fake Vehicle Interface 노드와 연동해 Control 출력(cmd) 확인
