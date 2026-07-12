# system_logger

파이프라인 전체의 로그 종착지. `module_integrate_node`와 `r2lp1_planning_node`
가 각자 `/system/log`에 JSON으로 던져두기만 하는 상태/지표 로그
(`sync_error_ms`, `latency_ms`, 파싱 에러 등)를 실제로 받아서 사람이 읽을 수
있게 콘솔에 찍고, 원하면 파일로도 남기고, 주기적으로 요약을 내는 노드
(두 노드의 README에 이미 그려져 있던 `system_logger_node`).

```
[module_integrate_node] --/system/log (JSON)--> [system_logger_node]
[r2lp1_planning_node]   --/system/log (JSON)--> [system_logger_node]
```

## 하는 일

1. `/system/log`(`std_msgs/String`, JSON)를 구독
2. 메시지마다 파싱 → 한 줄로 포맷(`[timestamp] node event key=value ...`)해서
   `get_logger().info(...)`로 출력, `log_file_path`가 설정돼 있으면 같은 줄을
   파일에도 append
3. `node`/`event` 조합별 카운트, `*_ms`로 끝나는 필드(예: `sync_error_ms`,
   `latency_ms`)의 노드별 최댓값, 에러 개수를 누적
4. `summary_interval_s`마다 누적 통계를 한 줄 요약으로 출력 (토픽을 직접
   `echo`하지 않아도 파이프라인 상태를 한눈에 확인 가능)

파싱/포맷/집계 로직(`log_record.py`)은 rclpy 의존이 없어 ROS 환경 없이
단위테스트 가능 (`chameleon_builder.py`, `decision_logic.py`와 동일한 패턴):

```bash
python3 -m pytest test/test_log_record.py -v
```

## ⚠️ 확정 안 된 가정

- `/system/log` 발행자들이 공통으로 `node`/`event`/`timestamp` 필드를 채운다는
  것 (지금까지는 `module_integrate_node`, `r2lp1_planning_node` 둘 다 이 관례를
  따름). 새 노드가 로그를 붙일 때도 이 세 필드는 유지해야 요약이 의미 있음.
- 로그 파일은 그냥 append-only 텍스트. 회전(rotation)이나 용량 제한은 없음.

## 실행

```bash
ros2 launch system_logger system_logger.launch.py &

ros2 launch fake_rsu fake_rsu.launch.py &
ros2 launch fake_rsu fake_rsu_objects.launch.py &
ros2 launch cits_adapter cits_adapter.launch.py &
ros2 launch cits_integration cits_integration.launch.py &
ros2 launch module_integrate module_integrate.launch.py &
ros2 launch r2lp1_planning r2lp1_planning.launch.py &
```

`system_logger_node`의 콘솔 출력만 보면 `module_integrate_node`의 동기화
오차와 `r2lp1_planning_node`의 판단 지연을 `/system/log`를 직접 `echo`하지
않고도 확인할 수 있다.
