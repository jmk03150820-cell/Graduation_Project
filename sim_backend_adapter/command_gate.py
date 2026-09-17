"""
Sim Backend Adapter — Command Gate + tick loop skeleton.

Implements the per-tick flow from the 2026-08-07 diagram ("both directions
bypass Core"):

    EgoControlCommand(N) ─┐
                          ├─ wait for both ─ apply ─ tick once ─ read state
    NpcControlBatch(N) ───┘        │
                                   ├─ publish EgoObservationFrame  → Ego Client
                                   ├─ publish WorldState+shadow    → Traffic Adapter
                                   └─ StepComplete(N, status)      → Core (notify only)

The actual simulator sits behind SimBackend so this gate is testable with no
live CARLA. Only CARLA is implemented first (later: MORAI via ROS Sync Mode,
AURELION via FMI). The gate does zero physics — it only sequences the tick.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Protocol


@dataclass
class EgoControlCommand:
    logical_id: str
    target_speed: float  # m/s
    target_steer: float  # rad


@dataclass
class NpcControl:
    logical_id: str
    target_speed: float  # m/s
    target_steer: float  # rad


@dataclass
class NpcControlBatch:
    controls: list[NpcControl] = field(default_factory=list)


class SimBackend(Protocol):
    """One simulator behind the adapter. CARLA first, MORAI/AURELION later."""

    def apply(self, ego: EgoControlCommand, npc: NpcControlBatch) -> None: ...
    def tick(self) -> None: ...
    def read_state(self) -> Any: ...  # WorldState; opaque to the gate


class CommandGate:
    """
    Gathers the ego command and the npc batch for the SAME tick, then runs one
    step: apply -> tick -> read state -> publish to ego & traffic -> notify Core.

    The wait IS the whole job. No content validation of the two inputs beyond a
    tick_id match — feedback 2026-07-25 #2 said the barrier is wait-only, so the
    diagram's "validate" box is treated as just this tick_id guard until the team
    says otherwise.
    ponytail: single in-flight tick, not pipelined. Enough for sync/lockstep; if
    async mode ever overlaps ticks, this needs a per-tick slot map instead.
    """

    def __init__(
        self,
        backend: SimBackend,
        publish_ego: Callable[[int, Any], None],
        publish_traffic: Callable[[int, Any], None],
        notify_core: Callable[[int, str], None],
    ) -> None:
        self.backend = backend
        self.publish_ego = publish_ego
        self.publish_traffic = publish_traffic
        self.notify_core = notify_core
        self._tick = 0
        self._ego: EgoControlCommand | None = None
        self._npc: NpcControlBatch | None = None

    def submit_ego(self, tick_id: int, ego: EgoControlCommand) -> None:
        self._check_tick(tick_id)
        self._ego = ego
        self._try_step()

    def submit_npc(self, tick_id: int, npc: NpcControlBatch) -> None:
        self._check_tick(tick_id)
        self._npc = npc
        self._try_step()

    def _check_tick(self, tick_id: int) -> None:
        if tick_id != self._tick:
            raise ValueError(f"expected tick {self._tick}, got {tick_id}")

    def _try_step(self) -> None:
        if self._ego is None or self._npc is None:
            return  # still waiting for the other side
        n = self._tick
        self.backend.apply(self._ego, self._npc)
        self.backend.tick()
        state = self.backend.read_state()
        self.publish_ego(n, state)      # EgoObservationFrame(N) -> Ego Client
        self.publish_traffic(n, state)  # WorldState(N)+shadow ego -> Traffic Adapter
        self.notify_core(n, "OK")       # StepComplete(N, status) -> Core
        self._ego = self._npc = None
        self._tick += 1


def _demo() -> None:
    events: list[tuple] = []

    class FakeBackend:
        def __init__(self) -> None:
            self.applied: tuple | None = None
            self.ticks = 0

        def apply(self, ego: EgoControlCommand, npc: NpcControlBatch) -> None:
            self.applied = (ego, npc)

        def tick(self) -> None:
            self.ticks += 1

        def read_state(self) -> str:
            return f"state@{self.ticks}"

    be = FakeBackend()
    gate = CommandGate(
        be,
        publish_ego=lambda n, s: events.append(("ego", n, s)),
        publish_traffic=lambda n, s: events.append(("traffic", n, s)),
        notify_core=lambda n, st: events.append(("core", n, st)),
    )

    # tick 0: only ego arrives -> must NOT tick until npc is here too
    gate.submit_ego(0, EgoControlCommand("ego0", 5.0, 0.0))
    assert be.ticks == 0, "must wait for npc before ticking"
    assert events == []

    # npc arrives -> the step runs, everyone is notified
    gate.submit_npc(0, NpcControlBatch([NpcControl("npc1", 3.0, 0.1)]))
    assert be.ticks == 1
    assert be.applied is not None and be.applied[0].logical_id == "ego0"
    assert ("ego", 0, "state@1") in events
    assert ("traffic", 0, "state@1") in events
    assert ("core", 0, "OK") in events

    # tick advanced to 1; arrival order reversed (npc first) still works
    events.clear()
    gate.submit_npc(1, NpcControlBatch([]))
    assert be.ticks == 1, "still waiting for ego"
    gate.submit_ego(1, EgoControlCommand("ego0", 6.0, 0.0))
    assert be.ticks == 2
    assert ("core", 1, "OK") in events

    # a stale / mismatched tick_id is rejected (the light "validate")
    try:
        gate.submit_ego(99, EgoControlCommand("ego0", 0.0, 0.0))
        assert False, "mismatched tick_id must be rejected"
    except ValueError:
        pass

    print("command_gate self-test OK")


if __name__ == "__main__":
    _demo()
