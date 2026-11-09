"""5번 공지 5번: 지금까지 통과한 핵심 계약 시험을 한 번에 돌리는 단일 진입점.

frame lifecycle(정상/중복/충돌/exactly-once/tick relation/capability/digest/
encoder provenance/new epoch/backend swap 경계, 10개) + startup(정상 Manifest->RUNNING/
지원 안 하는 simulator·rate/Run Transport 실패/READY 전 AdvanceFrame, 4개) +
component(RunControlCommand/ABORT/restart/RejectNotice 발행, 3개) + 불량 주입 자가검증.

`python -m simulation_layer.run_all_tests`
"""
from __future__ import annotations

from simulation_layer import test_component, test_frame_lifecycle, test_ros2_convert_portable, test_startup

SUITES = [
    ("frame_lifecycle", test_frame_lifecycle.TESTS),
    ("startup", test_startup.TESTS),
    ("component", test_component.TESTS),
    ("ros2_convert_portable", test_ros2_convert_portable.TESTS),
]

if __name__ == "__main__":
    total = 0
    for suite_name, tests in SUITES:
        print(f"\n== {suite_name} ==")
        for fn, desc in tests:
            fn()
            total += 1
            print(f"  PASS {fn.__name__}\n       검증: {desc}")

    print("\n== mutation self-check ==")
    test_frame_lifecycle._mutation_check()

    print(f"\n전체 {total}개 계약 시험 + 불량 주입 검출 PASS "
         f"({len(SUITES)}개 suite: {', '.join(n for n, _ in SUITES)})")
