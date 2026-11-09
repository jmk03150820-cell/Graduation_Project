# avva-platform Simulator 리뷰 — 1. 구조/책임 범위

TUK-AD-Lab/avva-platform (packages/avva-sim) 코드 리뷰. simulation_layer/와 비교 목적.

## 1-1. avva_sim 10개 클래스와 책임

| # | 클래스 | 파일 | 한 줄 책임 |
|---|---|---|---|
| 1 | `BackendRouter` | `backend_router.py` | run 시작 시 backend 하나를 factory로 만들고 잠금 |
| 2 | `PendingFrameRegistry` | `pending_frame.py` | 열린 frame이 정확히 1개임을 보장, terminal 전후 상태 전이 |
| 3 | `InputGate` | `input_gate.py` | ego/npc 2입력 수집·중복제거·freeze (lifecycle은 별도 입력 채널이 아니라 freeze 시점에 provider 콜백으로 내부 해석됨 — §5 참조) |
| 4 | `ActuationProfileResolver` | `actuation/profile.py` | manifest에서 actuation profile 확정 (거의 1회성) |
| 5 | `ActuationRealizer` | `actuation/realizer.py` | canonical target→제어기(종방향 PID/횡방향 P+rate-limit)→low-level actuation 계산 |
| 6 | `SimulatorFrameExecutor` | `frame_executor.py` | approval 확인→lifecycle 적용→backend 실행 위임 |
| 7 | `PhysicalActorLifecycleManager` | `actor_lifecycle.py` | spawn/despawn을 backend에 적용 |
| 8 | `ActorMappingStore` | `actor_mapping.py` | actor_key(canonical) ↔ native ref 매핑 소유 (CarlaBackend만 사용, MetaDriveBackend는 미사용 — §2 참조) |
| 9 | `SnapshotCommitPublisher` | `snapshot.py` | backend 결과를 검증해서 commit/abort 확정 |
| 10 | `ControlReceiptStore` | `receipts.py` | RECEIVED→ACCEPTED→APPLIED로 key당 최신 상태만 갱신, 역행 전이 금지 (5, 7번이 공유) — 전체 이력을 쌓는 로그가 아니라 이전 상태 객체는 교체되어 사라짐 |

## 1-2. 한 frame의 실제 호출 순서

```
1. PendingFrameRegistry.open_frame(run, world)
   → world N에서 frame N+1을 COLLECTING 상태로 연다

2. InputGate.ingest_ego(pending, command)     ─┐ Core가 ego/traffic 입력을
   InputGate.ingest_traffic(pending, batch)   ─┘ 보낼 때마다 개별 호출
   → 각각 ControlReceiptStore.record_accepted()로 RECEIVED→ACCEPTED

3. InputGate.freeze(pending)
   → ego+npc 다 모이면 lifecycle_provider 호출, FrozenInputSet + CommandSetReady 생성
   (Core가 CommandSetReady 보고 TickApprove를 발급 — avva-core 담당, avva-sim 밖)

4. ActuationRealizer.realize(inputs, world, profile)
   → 종방향은 PID, 횡방향은 P+rate-limit으로 canonical target(velocity/accel)을
     throttle/brake/steer로 변환 (§5 참조)

5. SimulatorFrameExecutor.execute(inputs, approval, actuation)
   ├─ PhysicalActorLifecycleManager.apply(lifecycle, approval)
   │    └─ backend.apply_lifecycle() → ActorMappingStore.bind()/tombstone()
   ├─ backend.encode_actuation(actuation)      → NativeCommandBatch
   ├─ backend.execute_frame(inputs, actuation) → BackendFrameResult (apply+tick+snapshot)
   └─ ControlReceiptStore.record_applied()/record_apply_failed() (actor별)

6. SnapshotCommitPublisher.finalize(result, inputs)
   → ActorMappingStore.snapshot()으로 provenance 검증
   → WorldState + EgoObservationFrame + StepComplete 묶음, 또는 FrameAbort

7. PendingFrameRegistry.mark_terminal(frame_id, committed)
   PendingFrameRegistry.clear_and_advance()
   → 다음 frame(N+2)을 위한 기준점 갱신
```

`BackendRouter.load()`는 이 루프 밖에서 run 시작 시 딱 한 번만 호출됨(simulator-service 부팅 시).

## 1-3. 구조적 특징 — `_FirstOperationThreadOwner` 반복

`backend_router.py`, `frame_executor.py`, `input_gate.py`, `snapshot.py`, `actor_lifecycle.py`,
`actor_mapping.py`, `actuation/realizer.py`, `receipts.py`, **`backends/carla.py`,
`backends/metadrive.py`** — **10개 파일**에 거의 동일한 스레드 소유권 클래스가 반복 정의됨
("첫 유효 연산을 수행한 스레드에 자신을 결박, 이후 다른 스레드가 건드리면
AUTHORITY_CONFLICT"). 이 중 8개는 byte-identical하고, `actor_lifecycle.py`와 `receipts.py`의
버전만 `require_if_bound()`라는 메서드가 하나 더 있는 변형. `pending_frame.py`만
`_require_owner()`라는 이름의 인라인 메서드로 동일 기능. 공유 base/mixin으로 뽑지 않고
파일마다 복붙됨.

## 1-4. 레이어 경계 — 어디까지가 avva-sim인가

```
┌─────────────────────────────────────────────────────────┐
│ ros2_ws/src/avva_sim_ros  (별도 ROS2 패키지)                │
│  AvvaSimulatorNode: ROS 메시지 decode → handler.on_xxx() 호출│
└───────────────────────┬─────────────────────────────────┘
                         │ handler (= 이 repo에 없음)
┌─────────────────────────────────────────────────────────┐
│ packages/avva-sim/src/avva_sim/   ← 진짜 "avva-sim"        │
│  backend-neutral 10개 클래스 (표 1-1 전체)                    │
│           ▼                                                │
│  avva_sim/backends/ (carla.py, metadrive.py) — vendor-specific │
└───────────────────────┬─────────────────────────────────┘
                         │ self._client (= 이 repo에 없음)
┌─────────────────────────────────────────────────────────┐
│ 실제 CARLA/MetaDrive SDK wrapper   ← 이 repo에 없음          │
└─────────────────────────────────────────────────────────┘
```

**진짜 avva-sim**: `packages/avva-sim/src/avva_sim/` 디렉토리 전체
(backends/, actuation/ 하위 포함) — 10개 클래스 + carla.py/metadrive.py 어댑터 골격까지.

**avva-sim이 아닌 것**:
- `avva-contracts` — DTO/타입만 정의하는 별도 패키지, sim/core가 공유하는 "언어"
- `avva-core` — `CoreFrameCoordinator`는 avva_sim을 한 줄도 import 안 함.
  `CommandSetReady`/`TickApprove`/`StepComplete`/`FrameAbort` typed 이벤트로만 통신 —
  Core와 Sim은 함수 호출이 아니라 완전히 분리된 프로세스/서비스
- `ros2_ws/src/avva_sim_ros` — ROS 접착부, 별도 ROS 패키지
- `apps/simulator-service/main.py` — composition root(실행 진입점)이지 avva-sim 코드 자체는 아님

**이 ZIP에 없는 것 (README에 명시된 범위 제한과 일치)**:
- avva_sim 10개 클래스를 ROS가 기대하는 단일 `handler`(on_ego_control 등)로 묶는
  접착 코드 (`apps/simulator-service/main.py`의 `_composition_provider`가 자리이지만
  `install_composition_provider()`로 외부 주입되어야 하는 빈 훅)
- `backends/carla.py`, `backends/metadrive.py`의 `self._client`가 실제로 호출할
  CARLA/MetaDrive SDK wrapper 구현체 — 이름(`apply_batch_sync`, `tick`, `snapshot`)은
  실제 CARLA API 그대로라 연동을 염두에 둔 설계이지만, 실제 통신 코드는 없음

## 2. adapter 역할 — 정확히 어디에 쪼개져 있나

둘 다 "공통 인터페이스 + vendor별 구현"으로 쪼개진 건 같음. 쪼개진 위치와 각 조각의
실체 유무가 다름.

### avva-platform — 6개 조각 (+ 1개는 이 repo에 아예 없음)

| # | 파일 | 역할 |
|---|---|---|
| 1 | `avva-contracts/backend.py` | 어댑터가 주고받는 **타입만** 정의(`BackendProfile`, `BackendSnapshot`, `BackendApplyReport`, `BackendStepReport`, `BackendFrameResult`) — 코드 없음, 계약서 |
| 2 | `avva-sim/backend_router.py` | `SimBackend` Protocol(최소 요구: `backend_id`, `capabilities()`, `close()`) 정의 + factory로 backend 하나 선택·고정 |
| 3 | `avva-sim/native_types.py` | `NativeActorRef`, `NativeCommandBatch` — vendor 쪽 식별자/명령을 감싸는 wrapper 타입 |
| 4 | `avva-sim/backends/carla.py` | `CarlaBackend` — CARLA **전용** 조율 로직(exactly-once 토큰, capability 검증, session poison, digest 재검증). 실제 native 호출은 `self._client`에 위임 |
| 5 | `avva-sim/backends/metadrive.py` | `MetaDriveBackend` — 위와 동일 패턴, MetaDrive 전용 |
| 6 | `avva-sim/actor_mapping.py` | `ActorMappingStore` — canonical actor_key ↔ `NativeActorRef` 매핑을 backend 밖에서 별도 소유. **`CarlaBackend`만 사용**(`mapping_store` 생성자 인자로 주입) — `MetaDriveBackend`엔 `mapping_store` 파라미터 자체가 없어 이 클래스를 안 씀, canonical actor_key를 매핑 없이 그대로 client에 전달 |
| — | **`self._client`의 실제 구현체** | **이 repo에 없음.** CarlaBackend/MetaDriveBackend가 호출하는 `apply_batch_sync`, `tick`, `snapshot`, `configure`가 실제로 뭘 하는지는 어디에도 없음 |

→ 4, 5번(vendor별 backend 클래스) 자체가 "진짜 어댑터"가 아니라, 진짜 어댑터(6번 자리,
`self._client`)를 감싸는 검증 wrapper. 알맹이가 빠진 채로 골격만 6군데 쪼개진 구조.

### simulation_layer (own) — 5개 조각, 전부 실체 있음

| # | 파일 | 역할 |
|---|---|---|
| 1 | `backend.py` | `SimulatorAdapter` Protocol 정의 + `create_simulator_adapter()` 팩토리 함수 + 공유 타입(`SimulatorCapability`/`RunConfig`/`ActorKinematics`) |
| 2 | `transforms.py` | 모듈 docstring이 명시하는 **"CARLA -> common" 전용** 좌표 변환 함수 모음(`location_to_common`/`velocity_to_common`/`steering_rad_to_native` 등, 전부 CARLA 축반전·부호반전 로직). MetaDrive는 이 중 `rpy_to_quaternion`(순수 사원수 수학, CARLA 종속 없음) 하나만 재사용 — "공유 유틸"이 아니라 "CARLA 전용 파일에서 범용 함수 하나만 얻어 쓰는" 구조 |
| 3 | `carla_backend.py` | `CarlaSimulatorAdapter` — CARLA 연결·spawn·apply·tick·snapshot을 전부 자체 구현(`import carla` 직접, `self._client = carla.Client(...)`) |
| 4 | `metadrive_backend.py` | `MetaDriveSimulatorAdapter` — MetaDrive 전부 자체 구현(`import metadrive` 직접) |
| 5 | `mock_backend.py` | `MockSimulatorAdapter` — 시뮬레이터 없이 1D kinematics로 자체 구현(테스트/데모용, 물리엔진 없음) |

→ 3, 4, 5번 각각이 그 자체로 완결된 어댑터. 밖에 위임하는 "client" 같은 게 없음 —
`carla_backend.py`가 곧 CARLA와 통신하는 코드 그 자체.

### 결론

둘 다 계층 구조 자체는 같은데, avva-platform은 vendor별 구현이 한 겹 더 실제 SDK 호출
(`client`)에 위임하는 구조라 그 마지막 겹이 repo에 없고, own은 vendor별 구현이 그 자체가
최종 실행 코드라 위임할 다음 겹이 없음 — **분할 깊이가 avva-platform이 하나 더 깊고, 그
마지막 층이 비어있다는 게 핵심 차이.**

## 3. approve / native tick / terminal exactly-once 보장 — 원리

### ① Approve — 재요청은 새 판단이 아니라 과거 답의 재확인

승인은 한 번 내려지면 내용을 저장해둔다. 같은 frame에 또 승인 요청이 오면 새로 판단하지
않고 "저장된 요청과 내용이 완전히 같은가"만 본다. 같으면 저장된 답을 그대로 반환(재승인
아님, 재확인), 조금이라도 다르면 재요청이 아니라 모순으로 보고 거부한다.

원리: **두 번째 호출을 막는 게 아니라 아무 효과도 없게 만든다** — 멱등성을 락이 아니라
메모이제이션으로 구현. 네트워크 중복 전송을 걱정할 필요가 없어짐.
→ 구현: `CoreFrameCoordinator.on_ready()` (frame_coordinator.py)

### ② Native tick — 결과를 몰라도 기회는 이미 썼다고 친다

tick은 되돌릴 수 없는 실제 side-effect라서, "성공 확인 후 재시도 여부 결정"은 위험하다.
타임아웃/예외가 나면 "진짜 안 됐는지, 됐는데 응답만 안 왔는지" 알 방법이 없기 때문.

원리: **native 호출을 시도하기도 전에 "이 frame의 tick 기회는 이미 소비됐다"고 먼저
기록한다.** 성공/실패/불명 무엇이든 상관없이 시도가 시작된 순간 재시도 가능성 자체를
차단 — 비관적 토큰 소비(pessimistic consumption). 결과를 알 수 없는 상황에서 안전한
쪽(재시도 금지)으로 미리 결정해버리는 방식.

여기에 사후 검증이 하나 더 붙는다: tick 후 "시뮬레이터 자체의 프레임 카운터가 정확히
1만큼 늘었는가"를 확인 — 토큰 체크로는 못 잡는 종류(다른 경로로 native tick이 몰래
두 번 불린 경우)를 잡는 독립적 2차 방어.

그리고 "알 수 없는 결과"가 한 번이라도 나면 그 frame만 막는 게 아니라 세션(런) 전체의
이후 native 호출을 다 막는다 — 한 번이라도 불확실한 상태가 생기면 그 시뮬레이터 인스턴스
자체를 더는 신뢰하지 않는다는 뜻.
→ 구현: 토큰 등록은 `SimulatorFrameExecutor.execute()` (frame_executor.py), 단조 체크와
사후 검증·세션 봉인은 `CarlaBackend.execute_frame()` (backends/carla.py)

### ③ Terminal — 결말은 한 번 나면 고정, 양쪽이 각자 기억한다

frame이 commit이든 abort든 결말이 확정되면 그 결말 자체를 저장한다. 같은 frame의 결말을
다시 요청/통지받으면 ①과 같은 원리로 "새로 계산 않고 저장된 결말을 재확인"한다.

여기엔 하나가 더 붙는다: 이 확인을 **결말을 결정하는 쪽과 통지받는 쪽 양쪽이 각자
독립적으로** 한다. 둘 사이의 통신 채널은 메시지를 잃거나 중복시킬 수 있다고 가정해야
하므로, "나는 정확히 한 번만 보냈다"만으론 부족하고 "같은 내용을 몇 번 받아도 정확히
한 번만 처리한다"를 양쪽 다 각자 지켜야 전체가 exactly-once가 된다. publish(외부 통지)
자체의 재시도도 별도로 막아서 내부 상태 안전이 외부 중복 통지로 무너지지 않게 한다.
→ 구현: 내부 상태는 `PendingFrameRegistry.mark_terminal()` (pending_frame.py), 결과
확정·publish 방어는 `SnapshotCommitPublisher.finalize()`/`_publish_terminal()`
(snapshot.py), Core 쪽 독립 dedupe는 `on_step_complete()`/`on_frame_abort()`
(frame_coordinator.py)

### 세 원리를 묶는 것 — "먼저 쓰고 나중에 확인" + "혼자만 만진다"

②③에서 반복되는 핵심은 **"확정 먼저, 실행은 그다음"** — 일반적 직관("해보고 성공하면
기록")과 반대로 뒤집혀 있다. 되돌릴 수 없는 side-effect를 다룰 때 나오는 표준 안전장치
패턴.

그리고 이 "체크했다가 표시한다"는 동작이 안전하려면 그 순간 딱 하나의 실행 흐름만 그
상태를 만질 수 있어야 한다 — 두 스레드가 동시에 "아직 안 썼네"를 통과하면 원리가 무너진다.
그래서 모든 컴포넌트가 "자신을 최초로 건드린 실행 흐름 하나에만 묶이고 다른 흐름이
건드리면 거부한다"는 규칙(`_FirstOperationThreadOwner` 패턴, §1-3)을 공유 — 이게 위 세
exactly-once 보장이 실제로 성립하기 위한 전제 조건.

### 네 `gate.py`의 "3-layer once-only defense" — 같은 세 원리의 다른 구현

`SimulationLayer.on_advance_frame()` (gate.py)에 defense 1/2/3으로 명명돼 있다.

| 방어 | 원리 | avva-platform | 네 gate.py |
|---|---|---|---|
| defense 1 | ③과 동일(멱등 재확인), 실행 전 단계에 적용 | `_terminal`/`_publish_attempts` (content-digest 기반) | `decision_cache` (decision_id 기반) |
| defense 2 | 구조적 제약 — 잘못된 상태에선 애초에 처리 자체가 불가능 | `PendingFrameRegistry`/`SimulatorFrameState` | `state != "READY"`면 즉시 거부 |
| defense 3 | ②와 동일(비관적 토큰 소비), 되돌릴 수 없는 호출 직전 | `_terminal_tokens` (tick 호출 전 등록) | `self.ticked` set (tick 호출 전 체크) |
| tick 후 사후 검증 | ②의 2차 방어 | `native_after == native_before + 1` | `native_now != self._native_frame_before + 1` |
| unknown 시 전체 차단 | 불확실하면 신뢰 자체를 접는다 | `_session_poison` 플래그 | `state = "ABORTED"` → 이후 입력 핸들러의 state 가드가 자동 거부 |

**주목할 차이**: `gate.py`의 defense 3엔 "예전엔 bare assert였음 → `python -O`로 돌리면
이 방어가 통째로 사라짐"이라는 주석이 있음 — 과거 `assert`로 구현했다가 최적화 플래그로
방어가 사라지는 버그를 겪고 지금의 명시적 처리로 고친 히스토리. avva-platform 쪽 검증
함수(`require()`)는 애초에 assert를 안 쓰는 일반 함수라 이 종류의 버그가 설계상 원천
차단됨.

## 4. 실제 SDK 호출 구현 여부, 미완성 지점, own Adapter와의 차이

### SDK 호출 구현 여부 — 확정: 안 돼 있음

`backends/carla.py`/`backends/metadrive.py`에 `import carla`/`import metadrive`가 아예 없음.
모든 native 호출(`apply_batch_sync`, `tick`, `snapshot`, `configure`, `reset`)이 생성자로
주입되는 `self._client`에 위임돼 있고, 그 client가 실제로 CARLA/MetaDrive SDK를 감싼 것인지
이 repo 어디에도 실체가 없음.
→ 구현: `CarlaBackend.__init__(client=...)` / `MetaDriveBackend.__init__(client=...)`

### 어디가 미완성인지 — 4가지 구멍

1. **client 구현체 자체가 없음.** `CarlaBackend`가 요구하는 메서드(`capabilities()`,
   `installed_version()`, `configure(...)`, `reset(seed=, scenario_id=)`,
   `encode_vehicle_control(...)`, `apply_batch_sync(...)`, `tick()`, `snapshot(frame_id=)`,
   `apply_lifecycle(intents=)`, `native_frame_id()`, `close()`)은 호출부에서 역추적해야만
   드러나고, 이를 만족하는 클래스가 repo 어디에도 없음.
2. **client 인터페이스 자체가 타입으로 정의돼 있지 않음.** `BackendRouter`가 요구하는
   `SimBackend` Protocol은 명시적으로 선언돼 있는데(`backend_router.py:16`), `CarlaBackend`가
   요구하는 "client Protocol"은 formal 선언이 없음 — 순전히 호출부 코드로만 유추 가능한
   암묵적 계약. avva-sim의 나머지 부분(모든 경계를 Protocol/dataclass로 명시)과 비교하면
   눈에 띄는 구멍.
3. **조립 접착 코드(glue)가 없음.** §1-4에서 확인한 대로 avva_sim 10개 클래스를 ROS가
   기대하는 단일 `handler`로 묶는 코드, `apps/simulator-service/main.py`의
   `_composition_provider` 모두 외부 주입 훅으로 비어있음.
4. **실제 profile 값이 없음.** `BackendProfile.fixed_step_ns`, `required_capabilities` 같은
   필드의 실제 인스턴스가 이 ZIP 어디에도 없음 — 타입 정의만 있고 실제 값은 없음.
5. **`CarlaBackend`에 `native_ref` 메서드가 없어 spawn 경로가 동작 안 함.**
   `PhysicalActorLifecycleManager._preflight_mapping()`이 spawn 처리 시
   `callable(getattr(backend, "native_ref", None))`을 요구하는데, `backends/carla.py`
   652줄 전체(생성자~`close()`)를 다 확인해도 `native_ref` 메서드가 정의돼 있지 않음.
   즉 이 상태로 spawn을 시도하면 이 체크에서 `False`가 나와 무조건
   `BACKEND_APPLY_FAILED`로 막힘 — despawn(`mapping_store.resolve_native()`만 필요)은
   되는데 spawn은 현재 코드로는 실행 불가능한 것으로 보임(client 미주입 상태에서의
   정적 코드 리뷰 기준, 런타임 재현은 안 해봄).

### own Adapter와의 차이

own(`carla_backend.py`/`metadrive_backend.py`)은 위 1, 3, 4가 이미 채워진 완결된 구현체임:
- 그 자체가 client 역할까지 겸함 — `carla.Client(host, port)`부터 `world.tick()`까지 직접 호출
- `MultiAgentMetaDrive` 실제 생성해서 `env.step()` 호출
- 실측 기반 세부사항이 코드에 녹아있음(실제로 돌려봐야만 나올 수 있는 지식):
  - CARLA(left-handed)와 MetaDrive(right-handed) 좌표계 차이 — 2026-08-27 headless smoke test 실측
  - MetaDrive는 native frame counter가 없어 `env.episode_step`으로 대체
  - MetaDrive는 `num_agents`가 env 생성 시점에 고정 — `reset()`에서 일괄 매핑
  - angular_velocity API 없어서 heading 차분으로 근사(`ponytail:` 태그)

**근데 그대로 갖다 붙일 순 없음 — 인터페이스 모양이 달라서 어댑팅 필요**:

| 지점 | own | avva-platform이 요구하는 것 |
|---|---|---|
| 실행 단계 | `apply_control()` + `tick()` 2단계 분리 | `execute_frame()` 1콜에 apply+tick+snapshot 다 묶임 |
| capability 표현 | `SimulatorCapability` dataclass(own 전용 스키마) | `CapabilityDeclaration`(avva_contracts, 다른 스키마) |
| actor 상태 | `dict[str, ActorKinematics]`(단순 dict) | `BackendApplyReport`/`BackendSnapshot`(numpy 배열, dtype 고정, native_actor_refs+mapping_registry_digest 포함) |
| lifecycle | `spawn_actor()`/`remove_actor()`가 즉시 호출 | `apply_lifecycle(intents)` 배치 단위, digest로 재시도 방지 결박 |
| actor 식별 | `actor_id: str` 그대로 | CarlaBackend는 canonical `actor_key`(int) ↔ `NativeActorRef` 매핑을 거쳐야 함(`ActorMappingStore`), MetaDriveBackend는 이 매핑 계층 없이 `actor_key`를 그대로 client에 전달 |

네 `carla_backend.py`가 "CARLA와 실제로 말하는 법"은 이미 아는데, avva-platform의
`self._client` 자리에 끼우려면 위 표의 형태 변환 계층을 하나 더 써야 함 — 단순 이식이
아니라 어댑터의 어댑터가 필요한 구조.

## 5. Input/Actuation — command freeze, input digest, actuation realization, actor lifecycle이 기존 설계와 어떻게 다른지

### command freeze — "몇 개의 독립 입구를 열어두는가" (정정됨 — 최초 정리는 부정확했음)

**정정**: `InputGate`엔 `ingest_ego()`, `ingest_traffic()` 두 개의 공개 수신 메서드만 있고
`ingest_lifecycle()` 같은 건 없음. avva-platform도 실제 메시지 레벨에선 own과 똑같이
**ego+npc 2개**만 진짜 입력. lifecycle은 `freeze()` 내부에서 `_resolve_lifecycle()`이
주입된 `lifecycle_provider` 콜백을 부르거나(없으면 빈 배치 생성) 하는 방식으로 **내부
해석**되지, 메시지로 "도착"하는 세 번째 입력이 아님. 코드 주석 자체가 이를 인정함:
*"Public API에 **누락된** lifecycle 운송 경로를 composition boundary에서 명시적으로
주입한다"* — 공식 계약(Public API)엔 lifecycle 입력 채널이 없고, `lifecycle_provider`는
그 구멍을 메우는 장치.

own은 lifecycle_intents를 `NpcControlBatch` 안의 정식 필드로 넣어서 traffic 메시지와
함께 처음부터 통합해 보낸다.

원리 차이(정정): **"lifecycle을 정식 채널 없이 별도 개념으로 취급하고 provider
injection으로 땜빵한다"(avva-platform) vs "lifecycle을 애초에 traffic 메시지의 일부로
설계해 정식 통합한다"(own)**. "3개 vs 2개 입력"의 차이가 아니라 "미완성 워크어라운드
vs 의도된 통합 설계"의 차이.
→ 구현: `InputGate.freeze()`/`_resolve_lifecycle()` (input_gate.py) /
`SimulationLayer._try_ready()` (gate.py)

### input digest — 실제로 확인한 결과, 방식 자체가 다름

`avva_contracts/algorithm.py`의 `stable_digest()`를 직접 확인한 결과:
```
dataclass → 필드명 기준 dict로 재귀 변환 → JSON 직렬화(sort_keys=True) → SHA-256
```
**JSON 기반**이고 필드 순서는 **dict key를 알파벳순 정렬**해서 결정된다. numpy 배열은
`{dtype, shape, values(list)}`로 풀어서 JSON에 태운다.

own(`hashing.py`)은 `CanonicalWriter`로 **고정폭 바이너리 인코딩**을 한다 — `u8`/`u16`/`u32`
/`f64`/`bool8`/`uuid128`/`hash256` 등 타입별 정확한 바이트 수, 필드 순서는 **IDL 스펙에
정의된 순서 그대로**(알파벳 아님), masked-off 필드는 인코딩에서 제외하는 규칙까지 명시적으로
문서화(§4.6, §6.6).

원리 차이: **"필드 이름을 정렬 기준으로 쓰는 구조적 해시"(avva-platform) vs "스펙에 못 박힌
바이트 순서의 이진 해시"(own)**. avva-platform 방식은 필드 이름이 바뀌면(의미가 같아도)
dict key가 바뀌어 digest가 달라짐 — own 방식은 필드 이름을 아예 해시에 안 넣고 값/순서만
고정폭으로 인코딩하므로 이름 변경엔 영향 안 받음(대신 값의 의미가 실제로 바뀌면 둘 다
당연히 바뀜). 대신 own은 인코더를 손으로 유지보수해야 하고 필드 추가 시 인코더를 안
고치면 조용히 스펙과 어긋날 위험이 있다(`capability_canonical_bytes()`의
`assert len(fields) == 11` 같은 필드 수 체크로 방어).
**(주의: 이전 정리에서 "JSON의 NaN/Infinity 리터럴이 실제 위험"이라고 썼던 부분은
과장이었음 — `stable_digest()` 자체는 이론적으로 비표준 JSON을 만들 수 있는 게 맞지만,
실제 digest 대상 타입(`TickApprove`, `FrozenInputSet` 등)엔 float 필드가 거의 없고
`ActuationRealizer.realize()`처럼 수치가 들어가는 경로는 해시 계산 전에
`np.isfinite()` 체크를 먼저 거치는 걸 확인함 — 실제로 이 이론적 결함이 뚫리는 호출
경로가 있는지는 확인하지 못했음.)**
→ 구현: `stable_digest()`/`_canonical()` (avva_contracts/algorithm.py) / `CanonicalWriter`
기반 `*_hash()` 함수들 (hashing.py)

### actuation realization — 제어 변환을 어디서 계산하는가 (정정 — "PID"는 절반만 맞음)

own: `carla_backend.py._to_vehicle_control()`이 `VELOCITY_TARGET`/`ACCELERATION_TARGET`을
어댑터 안에서 **순수 P(비례)만 있는 근사**로 계산(`err * 0.5` 식, 적분/미분 항 없음,
`ponytail:` 주석 "capability matrix 확정 전 임시"). `metadrive_backend.py._to_action()`도
동일 P-only 패턴을 **독립적으로 중복 구현**(조향 부호가 달라 `tf.steering_rad_to_native`
재사용 못 함, 종방향 게인 0.5는 CARLA 쪽과 같은 값을 복붙).

avva-platform: `ActuationRealizer.realize()`를 다시 정확히 보면 **종방향(속도/가속도
target)만 진짜 PID**(`_pid()`가 `kp*error + ki*integral + kd*derivative` 전항 계산)이고,
**횡방향(조향)은 PID가 아니라 목표 heading에 비례게인을 곱한 P 매핑 + `_rate_limit()`
(변화율 제한)**임 — 적분/미분 항이 조향엔 없음. "PID 변환을 한 번만 구현"은 부정확한
표현이었고, 정확히는 "종방향 PID + 횡방향 P/rate-limit을 한 번만 구현"임.

종방향만 놓고 비교하면 avva-platform(PID, 적분 windup 포함)이 own(순수 P, 적분 없음)보다
더 정교한 제어기를 쓰고 있음 — 이건 이전 정리에서 빠졌던 추가 차이점.

→ 중복 제거는 avva-platform이 유리, vendor-native 최적화(CARLA 자체 velocity controller 등)
활용은 own 구조가 유리. 트레이드오프.

### actor lifecycle — 정체성 매핑을 별도 계층으로 뽑았는가

avva-platform은 `ActorMappingStore`라는 **독립 컴포넌트**가 canonical `actor_key`(정수)와
vendor-native ref(backend_id+native_id+generation) 사이의 매핑을 전담한다. `bind()`는
"정확히 하나의 신규 key"만 허용하고, despawn된 key는 tombstone으로 영구 봉인(재사용 금지)
— 매핑 자체가 하나의 독립된 상태 기계.

own은 `actor_id: str`을 생성 시점부터 백엔드까지 그대로 들고 다닌다(`spawn_actor(actor_id,
spec)`) — canonical id와 native id를 구분하는 계층이 없다. CARLA 어댑터 내부에
`self._actors: dict[str, carla.Actor]`라는 로컬 매핑은 있지만, 이건 "avva-sim 레벨의 공유
서비스"가 아니라 그 백엔드 클래스 하나의 사적인 내부 상태.

원리 차이: **"정체성 매핑을 시스템 전체가 공유하는 1급 개념으로 승격한다"(avva-platform)
vs "정체성은 각 백엔드가 알아서 관리하는 구현 디테일이다"(own)**. 전자는 여러 backend나
컴포넌트가 같은 actor_key로 같은 대상을 가리킨다는 걸 시스템 차원에서 보장하고 ID reuse
버그를 구조적으로 막지만 매핑 계층 자체의 복잡도(bind/tombstone/epoch)가 늘어난다. 후자는
훨씬 단순하지만 "같은 actor_id가 다른 백엔드 인스턴스에서 다른 네이티브 객체를 가리킬 수
있다"는 걸 막는 장치가 없다(지금은 백엔드가 하나뿐이라 문제가 안 드러날 뿐).
→ 구현: `ActorMappingStore` (actor_mapping.py) / `CarlaSimulatorAdapter._actors`
(carla_backend.py, 사적 상태)

## 6. 20Hz / 0.05s가 실제로 고정된 것인지

repo 전체를 `fixed_step_ns`/`target_rate_hz`/`0.05`/숫자 리터럴 기준으로 grep해서 확인함.

**avva-platform**: `BackendProfile.fixed_step_ns: int`(avva_contracts/backend.py) 필드로
존재. `CarlaBackend.configure()`가 `fixed_delta_seconds = profile.fixed_step_ns /
1_000_000_000`으로 계산해 전달, `MetaDriveBackend.configure()`도 `> 0`만 검증.
**repo 전체에서 20Hz/50ms 같은 숫자가 하드코딩된 곳이 없음** — avva-traffic(SUMO),
avva-testkit(fixture)도 전부 `profile.fixed_step_ns`를 참조만 함, 값 하나가 Core/Sim/
Traffic 전체에 공유되는 단일 진실 공급원. 단, **실제 인스턴스 값(20Hz인지 몇인지)을
만드는 코드 자체가 이 ZIP에 없음** — §4에서 지적한 "실제 profile 값 없음"과 같은 구멍.

**own**: `RunConfig.target_rate_hz: float = 20.0`이 dataclass **기본값**(backend.py:86),
`fixed_step_ns = round(1e9 / target_rate_hz)`로 유도되는 property — override 가능.
`check_configuration()`이 이 값을 backend별 지원 범위와 대조:
- CARLA: `min_hz=10.0, max_hz=100.0`
- MetaDrive: `min_hz=1.0, max_hz=50.0`(comment: "physics_world_step_size 하한 0.02s 기준"
  — 0.02s=50Hz, 값 일치 확인됨)
- Mock: `min_hz=1.0, max_hz=1000.0`

기본값 20Hz는 세 backend 범위 전부에 들어가 어느 backend를 골라도 그대로 동작.

**결론**: 둘 다 "20Hz는 상수가 아니라 config에서 유도"라는 설계는 동일. 차이는 own은
backend별 지원 범위 검증(min/max Hz)과 물리적 근거(SUMO/MetaDrive 스텝 크기 하한)까지
코드에 있는데, avva-platform의 `BackendProfile`엔 `fixed_step_ns > 0` 검증만 있고 backend별
rate 범위 개념 자체가 없음 — `actuation/profile.py`의 `CONTROL_PERIOD_NS` capability 대조는
actuation profile 쪽 검증이지 backend rate의 min/max 범위 검증과는 별개.
