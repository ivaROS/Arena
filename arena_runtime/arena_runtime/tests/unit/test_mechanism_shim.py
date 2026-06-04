"""Pure-logic tests for the mechanism shim (unified always-present model).

Skipped if task_generator / arena_simulation_setup aren't importable (no
sourced overlay), since the shim's data types live there.
"""
from __future__ import annotations

import math

import pytest

pytest.importorskip("task_generator.shared")
pytest.importorskip("arena_runtime.sim._mechanism_shim")

from arena_runtime.sim._mechanism_shim import (  # noqa: E402
    DOOR_INSET,
    INSIDE_DOOR_BLOCKER_RADIUS,
    WALL_THICKNESS,
    _advance_state,
    _compute_teleport_destinations,
    _door_geometry,
    _door_open_pose,
    _door_slot,
    _DoorRuntime,
    _DoorState,
    _elevator_wall_geometries,
    _ElevatorRuntime,
    _inside_cabin,
    _is_triggered,
    _near_door_segment,
    _step_elevator,
)
from arena_simulation_setup.utils.geometry import Orientation, Pose, Position  # noqa: E402
from task_generator.shared import Door, Elevator  # noqa: E402

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _door(
    name: str = "d",
    start: tuple[float, float, float] = (0.0, 0.0, 0.0),
    end: tuple[float, float, float] = (1.0, 0.0, 0.0),
    activation: tuple[float, float] = (1.5, 1.5),
    transition_time: float = 1.0,
    hold_time: float = 2.0,
    kind: str = "sliding",
) -> Door:
    return Door(
        name=name,
        start=Position(*start),
        end=Position(*end),
        kind=kind,
        activation_distance=activation,
        transition_time=transition_time,
        hold_time=hold_time,
    )


def _elevator(
    name: str = "e",
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    size: tuple[float, float, float] = (2.0, 2.0, 2.5),
    destination: str = "",
    travel_time: float = 3.0,
    hold_time: float = 2.0,
    transition_time: float = 1.0,
    activation_distance: float = 1.5,
    door_side: str = "+x",
    accept_outside_calls: bool = True,
) -> Elevator:
    return Elevator(
        name=name,
        position=Position(*position),
        size=list(size),
        door_side=door_side,
        destination=destination,
        activation_distance=activation_distance,
        transition_time=transition_time,
        hold_time=hold_time,
        travel_time=travel_time,
        accept_outside_calls=accept_outside_calls,
    )


def _door_runtime(door: Door, kind: str = "sliding") -> _DoorRuntime:
    closed = Pose(position=Position(0.0, 0.0, 1.0), orientation=Orientation.from_yaw(0.0))
    open_p = Pose(position=Position(0.0, 1.0, 1.0), orientation=Orientation.from_yaw(0.0))
    return _DoorRuntime(door=door, closed_pose=closed, open_pose=open_p, effective_kind=kind)


def _elev_runtime(elev: Elevator, door_name: str = "d", destination: str | None = None) -> _ElevatorRuntime:
    return _ElevatorRuntime(elevator=elev, door_name=door_name, destination_name=destination or elev.destination)


# ---------------------------------------------------------------------------
# _inside_cabin
# ---------------------------------------------------------------------------


def test_inside_cabin_center():
    e = _elevator(position=(10.0, 5.0, 0.0), size=(2.0, 2.0, 2.5))
    assert _inside_cabin(e, (10.0, 5.0))


def test_inside_cabin_boundary():
    e = _elevator(position=(10.0, 5.0, 0.0), size=(2.0, 2.0, 2.5))
    assert _inside_cabin(e, (11.0, 6.0))
    assert _inside_cabin(e, (9.0, 4.0))


def test_inside_cabin_just_outside():
    e = _elevator(position=(10.0, 5.0, 0.0), size=(2.0, 2.0, 2.5))
    assert not _inside_cabin(e, (11.01, 5.0))
    assert not _inside_cabin(e, (10.0, 6.01))


def test_inside_cabin_far():
    e = _elevator(position=(10.0, 5.0, 0.0))
    assert not _inside_cabin(e, (100.0, 0.0))


# ---------------------------------------------------------------------------
# _is_triggered
# ---------------------------------------------------------------------------


def test_is_triggered_empty_positions():
    d = _door(start=(0, 0, 0), end=(1, 0, 0))
    assert not _is_triggered(d, [])


def test_is_triggered_at_start_within_radius():
    d = _door(start=(0, 0, 0), end=(10, 0, 0), activation=(1.5, 1.5))
    assert _is_triggered(d, [(0.0, 1.0)])  # 1.0 < 1.5


def test_is_triggered_at_end_within_radius():
    d = _door(start=(0, 0, 0), end=(10, 0, 0), activation=(1.5, 1.5))
    assert _is_triggered(d, [(10.0, 1.0)])


def test_is_triggered_outside_radius():
    d = _door(start=(0, 0, 0), end=(10, 0, 0), activation=(1.5, 1.5))
    assert not _is_triggered(d, [(0.0, 2.0)])  # 2.0 > 1.5


def test_is_triggered_midspan():
    d = _door(start=(0, 0, 0), end=(10, 0, 0), activation=(1.5, 1.5))
    assert _is_triggered(d, [(5.0, 1.0)])  # centre of a wide doorway, far from both jambs


def test_is_triggered_zero_radius_disabled():
    d = _door(start=(0, 0, 0), end=(10, 0, 0), activation=(0.0, 0.0))
    assert not _is_triggered(d, [(0.0, 0.0)])


# ---------------------------------------------------------------------------
# _advance_state (door state machine)
# ---------------------------------------------------------------------------


def test_advance_closed_to_opening_on_fresh_trigger():
    d = _door(transition_time=1.0, hold_time=2.0)
    r = _door_runtime(d, kind="sliding")
    r.last_trigger_sim_time = 10.0  # fresh
    _advance_state(r, dt=0.1, now=10.0)
    assert r.state == _DoorState.OPENING
    assert pytest.approx(r.progress) == 0.1


def test_advance_opening_reaches_open():
    d = _door(transition_time=1.0, hold_time=2.0)
    r = _door_runtime(d, kind="sliding")
    r.state = _DoorState.OPENING
    r.progress = 0.95
    r.last_trigger_sim_time = 10.0
    _advance_state(r, dt=0.1, now=10.0)
    assert r.state == _DoorState.OPEN
    assert r.progress == 1.0


def test_advance_open_to_closing_when_stale():
    d = _door(transition_time=1.0, hold_time=2.0)
    r = _door_runtime(d, kind="sliding")
    r.state = _DoorState.OPEN
    r.progress = 1.0
    r.last_trigger_sim_time = 5.0  # 5.0s old, hold_time=2 -> stale
    _advance_state(r, dt=0.1, now=10.0)
    assert r.state == _DoorState.CLOSING
    assert pytest.approx(r.progress) == 0.9


def test_advance_closing_reverses_on_fresh_trigger():
    d = _door(transition_time=1.0, hold_time=2.0)
    r = _door_runtime(d, kind="sliding")
    r.state = _DoorState.CLOSING
    r.progress = 0.4
    r.last_trigger_sim_time = 10.0  # fresh now
    _advance_state(r, dt=0.1, now=10.0)
    assert r.state == _DoorState.OPENING
    assert pytest.approx(r.progress) == 0.5


def test_advance_teleport_snaps_open_on_fresh():
    d = _door(kind="teleport")
    r = _door_runtime(d, kind="teleport")
    r.last_trigger_sim_time = 10.0
    _advance_state(r, dt=0.1, now=10.0)
    assert r.state == _DoorState.OPEN
    assert r.progress == 1.0


def test_advance_teleport_snaps_closed_on_stale():
    d = _door(kind="teleport", hold_time=2.0)
    r = _door_runtime(d, kind="teleport")
    r.state = _DoorState.OPEN
    r.progress = 1.0
    r.last_trigger_sim_time = 0.0
    _advance_state(r, dt=0.1, now=10.0)
    assert r.state == _DoorState.CLOSED
    assert r.progress == 0.0


# ---------------------------------------------------------------------------
# _step_elevator: outside trigger and accept_outside_calls
# ---------------------------------------------------------------------------


def test_outside_trigger_opens_door_when_accept_outside_calls():
    """Outside trigger refreshes last_trigger, keeping door open."""
    elev = _elevator(name="a", destination="b", accept_outside_calls=True)
    a = _elev_runtime(elev, door_name="a/door")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.last_trigger_sim_time = -100.0
    _step_elevator(a, dr, None, occupants=[], near_door=True, outside_trigger=True, now=5.0)
    assert dr.last_trigger_sim_time == 5.0
    assert not a.departing


def test_outside_trigger_blocked_when_not_accept_outside_calls():
    """accept_outside_calls=False: outside trigger does not refresh the door."""
    elev = _elevator(name="a", destination="b", accept_outside_calls=False)
    a = _elev_runtime(elev, door_name="a/door")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.last_trigger_sim_time = -100.0
    _step_elevator(a, dr, None, occupants=[], near_door=True, outside_trigger=True, now=5.0)
    # Door trigger not refreshed: door will close because no occupants.
    assert dr.last_trigger_sim_time == -100.0


def test_outside_trigger_blocked_does_not_set_departing():
    """accept_outside_calls=False: outside trigger with no occupants neither departs nor opens."""
    elev = _elevator(name="a", destination="b", accept_outside_calls=False)
    a = _elev_runtime(elev, door_name="a/door")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    _step_elevator(a, dr, None, occupants=[], near_door=True, outside_trigger=True, now=5.0)
    assert not a.departing


def test_arrival_opens_door_regardless_of_accept_outside_calls():
    """Scheduled arrival always opens the door, even with accept_outside_calls=False."""
    elev = _elevator(name="b", destination="a", accept_outside_calls=False)
    b = _elev_runtime(elev, door_name="b/door")
    b.arriving_eta = 10.0
    b.pending_occupants = (("r", (5.0, 0.0)),)
    dr = _door_runtime(_door(name="b/door"), kind="sliding")
    dr.last_trigger_sim_time = -100.0
    result = _step_elevator(b, dr, None, occupants=[], near_door=False, outside_trigger=False, now=10.0)
    assert dr.last_trigger_sim_time == 10.0
    assert result.teleport_job is not None
    assert b.just_arrived == {"r": False}


# ---------------------------------------------------------------------------
# _step_elevator: idle, departing, arrival
# ---------------------------------------------------------------------------


def test_idle_empty_cabin_does_not_refresh_door():
    """Empty cabin with no trigger does not refresh the door, letting it close."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.last_trigger_sim_time = -100.0
    _step_elevator(a, dr, None, occupants=[], near_door=False, outside_trigger=False, now=5.0)
    assert dr.last_trigger_sim_time == -100.0
    assert not a.departing


def test_new_occupant_door_still_open_does_not_depart():
    """Occupant inside, door not yet closed: stay idle (door closes naturally)."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.OPEN
    dr.progress = 1.0
    _step_elevator(a, dr, None, occupants=[("r", (0.0, 0.0))], near_door=False, outside_trigger=False, now=5.0)
    assert not a.departing


def test_new_occupant_door_closed_starts_departing():
    """Occupant inside, door fully closed: enter departing mode."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSED
    _step_elevator(a, dr, None, occupants=[("r", (0.0, 0.0))], near_door=False, outside_trigger=False, now=5.0)
    assert a.departing


def test_departing_closing_abort_reverts():
    """Doorway blocker while departing cancels departure, refreshes door."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.departing = True
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSING
    dr.progress = 0.4
    dr.last_trigger_sim_time = -100.0
    _step_elevator(a, dr, None, occupants=[], near_door=True, outside_trigger=False, now=7.0)
    assert not a.departing
    assert dr.last_trigger_sim_time == 7.0


def test_departing_door_closed_commits_teleport_to_dest():
    """Departing door fully closes: schedule arrival at destination."""
    elev_a = _elevator(name="a", destination="b", travel_time=3.0)
    elev_b = _elevator(name="b", destination="a")
    a = _elev_runtime(elev_a, door_name="a/door", destination="b")
    b = _elev_runtime(elev_b, door_name="b/door", destination="a")
    a.departing = True
    dr_a = _door_runtime(_door(name="a/door"), kind="sliding")
    dr_a.state = _DoorState.CLOSED
    result = _step_elevator(
        a, dr_a, dest_runtime=b,
        occupants=[("r1", (0.5, 0.5))], near_door=False, outside_trigger=False, now=2.0,
    )
    assert not a.departing
    assert b.arriving_eta == pytest.approx(5.0)
    assert b.pending_occupants == (("r1", (0.5, 0.5)),)
    assert result.teleport_job is None  # deferred until arrival


def test_departing_missing_destination_reverts():
    """Departing with no destination: cancel, hold door open."""
    elev = _elevator(name="a", destination="ghost")
    a = _elev_runtime(elev, destination="ghost")
    a.departing = True
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSED
    result = _step_elevator(a, dr, None, occupants=[], near_door=False, outside_trigger=False, now=5.0)
    assert result.missing_destination
    assert not a.departing
    assert dr.last_trigger_sim_time == 5.0


def test_arriving_before_eta_suppresses_door():
    """In-transit teleport suppresses all door triggers."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.arriving_eta = 10.0
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.last_trigger_sim_time = -100.0
    _step_elevator(a, dr, None, occupants=[], near_door=True, outside_trigger=True, now=5.0)
    assert a.arriving_eta == pytest.approx(10.0)
    assert dr.last_trigger_sim_time == -100.0


def test_arriving_at_eta_fires_teleport_and_opens_door():
    """ETA reached: teleport job created, door opened, arriving_eta cleared."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.arriving_eta = 10.0
    a.pending_occupants = (("r", (5.0, 0.5)),)
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    result = _step_elevator(a, dr, None, occupants=[], near_door=False, outside_trigger=False, now=10.0)
    assert a.arriving_eta == -math.inf
    assert dr.last_trigger_sim_time == 10.0
    assert result.teleport_job == ("b", "a", [("r", (5.0, 0.5))])
    assert a.just_arrived == {"r": False}
    assert a.pending_occupants == ()


def test_just_arrived_tracking_inside_confirmation():
    """First inside observation sets confirmed flag to True."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.just_arrived = {"r": False}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    _step_elevator(a, dr, None, occupants=[("r", (0.0, 0.0))], near_door=False, outside_trigger=False, now=5.0)
    assert a.just_arrived["r"] is True


def test_just_arrived_stale_outside_before_inside_does_not_clear():
    """Post-teleport: outside observation before inside confirmed must not clear just_arrived."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.just_arrived = {"r": False}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    _step_elevator(
        a, dr, None,
        occupants=[],
        near_door=False, outside_trigger=False, now=5.0,
        outside_names=frozenset({"r"}),
    )
    assert a.just_arrived == {"r": False}


def test_just_arrived_outside_after_inside_confirmed_clears():
    """Once inside confirmed, outside observation clears just_arrived."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.just_arrived = {"r": True}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    _step_elevator(
        a, dr, None,
        occupants=[],
        near_door=False, outside_trigger=False, now=5.0,
        outside_names=frozenset({"r"}),
    )
    assert a.just_arrived == {}


def test_just_arrived_occupant_holds_door_open():
    """just_arrived occupant is not counted as new: door stays open."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.just_arrived = {"r": True}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSED
    _step_elevator(a, dr, None, occupants=[("r", (0.0, 0.0))], near_door=False, outside_trigger=False, now=5.0)
    assert not a.departing
    assert dr.last_trigger_sim_time == 5.0


def test_new_occupant_with_just_arrived_resident_can_depart():
    """Fresh entrant (r2) triggers depart even though r is just_arrived."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.just_arrived = {"r": True}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSED
    _step_elevator(
        a, dr, None,
        occupants=[("r", (0.0, 0.0)), ("r2", (0.0, 0.0))],
        near_door=False, outside_trigger=False, now=5.0,
    )
    assert a.departing


def test_closing_abort_holds_door_when_inside_occupant_near_door():
    """Inside occupant at the doorway holds the door (closing_abort)."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSING
    dr.progress = 0.6
    dr.last_trigger_sim_time = -100.0
    _step_elevator(a, dr, None, occupants=[("r", (0.0, 0.0))], near_door=True, outside_trigger=False, now=8.0)
    assert dr.last_trigger_sim_time == 8.0


def test_dispatched_cabin_stays_sealed_during_transit():
    """Regression: a cabin whose rider is mid-flight keeps its door shut even under an outside
    call, so the door is closed before the teleport (it used to reopen and close only after)."""
    a = _elev_runtime(_elevator(name="a", destination="b", accept_outside_calls=True), destination="b")
    a.dispatched = {"r1"}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSED
    dr.last_trigger_sim_time = -100.0
    result = _step_elevator(a, dr, None, occupants=[("r1", (0.0, 0.0))], near_door=True, outside_trigger=True, now=5.0)
    assert dr.last_trigger_sim_time == -100.0  # not refreshed: door stays closed
    assert not a.departing
    assert result.teleport_job is None


def test_dispatched_clears_when_rider_leaves_cabin():
    """Once the dispatched rider's teleport has moved them out of the cabin, the cabin resumes."""
    a = _elev_runtime(_elevator(name="a", destination="b"), destination="b")
    a.dispatched = {"r1"}
    dr = _door_runtime(_door(name="a/door"), kind="sliding")
    dr.state = _DoorState.CLOSED
    _step_elevator(a, dr, None, occupants=[], near_door=False, outside_trigger=False, now=5.0)
    assert a.dispatched == set()


# ---------------------------------------------------------------------------
# 3-elevator ring (1->2->3->1): no init bug
# ---------------------------------------------------------------------------


def test_three_elevator_ring_no_init_bug():
    """Ring topology: all three elevators start with doors closed, no special priming needed."""
    e1 = _elevator(name="e1", position=(0.0, 0.0, 0.0), destination="e2", travel_time=2.0)
    e2 = _elevator(name="e2", position=(10.0, 0.0, 0.0), destination="e3", travel_time=2.0)
    e3 = _elevator(name="e3", position=(20.0, 0.0, 0.0), destination="e1", travel_time=2.0)
    r1 = _elev_runtime(e1, door_name="e1/door", destination="e2")
    r2 = _elev_runtime(e2, door_name="e2/door", destination="e3")
    r3 = _elev_runtime(e3, door_name="e3/door", destination="e1")
    runtimes = {"e1": r1, "e2": r2, "e3": r3}
    # All start with arriving_eta == -inf, departing == False: no stuck state.
    for rt in runtimes.values():
        assert rt.arriving_eta == -math.inf
        assert not rt.departing


def test_three_elevator_ring_occupant_rides_e1_to_e2():
    """Occupant in e1, door closes -> arrival scheduled at e2."""
    e1 = _elevator(name="e1", position=(0.0, 0.0, 0.0), destination="e2", travel_time=2.0)
    e2 = _elevator(name="e2", position=(10.0, 0.0, 0.0), destination="e3", travel_time=2.0)
    r1 = _elev_runtime(e1, door_name="e1/door", destination="e2")
    r2 = _elev_runtime(e2, door_name="e2/door", destination="e3")
    dr1 = _door_runtime(_door(name="e1/door"), kind="sliding")
    dr1.state = _DoorState.CLOSED
    result = _step_elevator(r1, dr1, r2, occupants=[("rob", (0.0, 0.0))], near_door=False, outside_trigger=False, now=1.0)
    # departing set, one more tick with closed door commits the teleport schedule.
    assert r1.departing
    result2 = _step_elevator(r1, dr1, r2, occupants=[("rob", (0.0, 0.0))], near_door=False, outside_trigger=False, now=1.05)
    assert r2.arriving_eta == pytest.approx(3.05)
    assert r2.pending_occupants == (("rob", (0.0, 0.0)),)
    assert result2.teleport_job is None


# ---------------------------------------------------------------------------
# _door_open_pose
# ---------------------------------------------------------------------------


def test_door_open_pose_sliding():
    d = _door(start=(0, 0, 0), end=(2, 0, 0))  # length 2 along +x
    _size, closed = _door_geometry(d)
    open_p = _door_open_pose(d, closed, "sliding")
    assert pytest.approx(open_p.position.x) == closed.position.x + 2.0
    assert pytest.approx(open_p.position.y) == closed.position.y
    assert pytest.approx(open_p.position.z) == closed.position.z


def test_door_open_pose_teleport_drops_z():
    d = _door(start=(0, 0, 0), end=(2, 0, 0), kind="teleport")
    _size, closed = _door_geometry(d)
    open_p = _door_open_pose(d, closed, "teleport")
    assert open_p.position.x == closed.position.x
    assert open_p.position.y == closed.position.y
    assert open_p.position.z == closed.position.z - 100.0


def test_door_open_pose_sliding_top():
    d = Door(
        name="d",
        start=Position(0.0, 0.0, 0.0),
        end=Position(2.0, 0.0, 0.0),
        kind="sliding_top",
        height=2.5,
    )
    _size, closed = _door_geometry(d)
    open_p = _door_open_pose(d, closed, "sliding_top")
    assert open_p.position.x == pytest.approx(closed.position.x)
    assert open_p.position.y == pytest.approx(closed.position.y)
    assert open_p.position.z == pytest.approx(closed.position.z + d.height)
    assert open_p.orientation == closed.orientation


# ---------------------------------------------------------------------------
# tick-driven full cycle tests
# ---------------------------------------------------------------------------


DT = 1.0 / 30.0


def _prime_pair(
    *,
    hold_time: float = 2.0,
    transition_time: float = 1.0,
    travel_time: float = 3.0,
    accept_outside_calls_a: bool = True,
    accept_outside_calls_b: bool = True,
) -> tuple[dict[str, _ElevatorRuntime], dict[str, _DoorRuntime]]:
    """Build a two-cabin pair. Both start idle (doors closed, no departure pending)."""
    elev_a = _elevator(name="a", destination="b",
                       hold_time=hold_time, transition_time=transition_time, travel_time=travel_time,
                       accept_outside_calls=accept_outside_calls_a)
    elev_b = _elevator(name="b", position=(10.0, 0.0, 0.0), destination="a",
                       hold_time=hold_time, transition_time=transition_time, travel_time=travel_time,
                       accept_outside_calls=accept_outside_calls_b)
    a = _elev_runtime(elev_a, door_name="a/door", destination="b")
    b = _elev_runtime(elev_b, door_name="b/door", destination="a")
    door_a = _door_runtime(_door(name="a/door", hold_time=hold_time, transition_time=transition_time))
    door_b = _door_runtime(_door(name="b/door", hold_time=hold_time, transition_time=transition_time))
    runtimes = {"a": a, "b": b}
    doors = {"a/door": door_a, "b/door": door_b}
    return runtimes, doors


def _run_ticks(
    runtimes: dict[str, _ElevatorRuntime],
    doors: dict[str, _DoorRuntime],
    *,
    start: float,
    n: int,
    outside_trigger: dict[str, bool] | None = None,
    occupants: dict[str, list[tuple[str, tuple[float, float]]]] | None = None,
    outside_names: frozenset[str] = frozenset(),
) -> tuple[float, list[tuple[str, str, list[tuple[str, tuple[float, float]]]]]]:
    outside_trigger = outside_trigger or {}
    occupants = occupants or {}
    now = start
    jobs: list[tuple[str, str, list[tuple[str, tuple[float, float]]]]] = []
    for _ in range(n):
        now += DT
        for name, elev in runtimes.items():
            door = doors[elev.door_name]
            dest = runtimes.get(elev.destination_name)
            occ = occupants.get(name, [])
            outside = outside_trigger.get(name, False)
            near = outside or _near_door_segment(door.door, [xy for _, xy in occ], INSIDE_DOOR_BLOCKER_RADIUS)
            r = _step_elevator(
                elev, door, dest,
                occupants=occ,
                near_door=near,
                outside_trigger=outside,
                now=now,
                outside_names=outside_names,
            )
            if r.teleport_job is not None:
                jobs.append(r.teleport_job)
        for door in doors.values():
            _advance_state(door, DT, now)
    return now, jobs


def test_tick_idle_both_doors_closed_no_trigger():
    """Without any trigger or occupants, doors remain closed."""
    runtimes, doors = _prime_pair()
    _, jobs = _run_ticks(runtimes, doors, start=0.0, n=300)
    assert jobs == []
    assert doors["a/door"].state == _DoorState.CLOSED
    assert doors["b/door"].state == _DoorState.CLOSED


def test_tick_outside_trigger_opens_door():
    """Outside trigger on A keeps A's door open."""
    runtimes, doors = _prime_pair()
    _, jobs = _run_ticks(runtimes, doors, start=0.0, n=100, outside_trigger={"a": True})
    assert jobs == []
    assert doors["a/door"].state == _DoorState.OPEN


def test_tick_robot_boards_and_rides():
    """Occupant inside A, door already closed: A departs immediately, robot teleports to B."""
    runtimes, doors = _prime_pair()
    _, jobs = _run_ticks(
        runtimes, doors, start=0.0, n=200,
        occupants={"a": [("r1", (-0.5, 0.0))]},
    )
    assert jobs == [("a", "b", [("r1", (-0.5, 0.0))])]
    assert runtimes["b"].just_arrived == {"r1": False}
    assert doors["b/door"].state == _DoorState.OPEN


def test_tick_outside_trigger_only_opens_that_door():
    """B-side outside trigger only opens B's door; no cabin movement, no teleport."""
    runtimes, doors = _prime_pair()
    _, jobs = _run_ticks(runtimes, doors, start=0.0, n=250, outside_trigger={"b": True})
    assert jobs == []
    assert doors["b/door"].state == _DoorState.OPEN
    assert doors["a/door"].state == _DoorState.CLOSED


def test_tick_full_round_trip():
    """A->B->A: each leg produces one teleport job via occupant boarding."""
    runtimes, doors = _prime_pair()
    # Leg 1: robot in A, no trigger needed (door already closed).
    now, jobs1 = _run_ticks(
        runtimes, doors, start=0.0, n=200,
        occupants={"a": [("r1", (-0.5, 0.0))]},
    )
    assert len(jobs1) == 1
    assert doors["b/door"].state == _DoorState.OPEN
    # Mark robot as arrived at B (clear just_arrived for re-departure).
    runtimes["b"].just_arrived.clear()
    # Leg 2: robot now in B (B's door open from arrival), robot stays in B.
    now, jobs2 = _run_ticks(
        runtimes, doors, start=now, n=200,
        occupants={"b": [("r1", (10.0, 0.0))]},
    )
    assert len(jobs2) == 1
    assert jobs2[0][:2] == ("b", "a")
    assert doors["a/door"].state == _DoorState.OPEN


def test_tick_blocker_at_departing_doorway_reverts():
    """Outside trigger on A while A is departing (occupant closing) cancels departure."""
    runtimes, doors = _prime_pair()
    a = runtimes["a"]
    # Manually set departing so we can test the abort.
    a.departing = True
    doors["a/door"].state = _DoorState.CLOSING
    doors["a/door"].progress = 0.4
    # Outside trigger on A cancels departing.
    _, _ = _run_ticks(runtimes, doors, start=0.0, n=1, outside_trigger={"a": True})
    assert not a.departing


def test_tick_missing_destination_reverts():
    """Departing with unknown destination holds door open."""
    runtimes, doors = _prime_pair()
    a = runtimes["a"]
    a.departing = True
    del runtimes["b"]
    doors["a/door"].state = _DoorState.CLOSED
    doors["a/door"].progress = 0.0
    _, jobs = _run_ticks(runtimes, doors, start=0.0, n=5)
    assert jobs == []
    assert not a.departing


def test_tick_no_pingpong_post_teleport():
    """Post-teleport TF gap: just_arrived prevents re-departure before first inside obs."""
    runtimes, doors = _prime_pair()
    b = runtimes["b"]
    now, jobs1 = _run_ticks(
        runtimes, doors, start=0.0, n=250,
        outside_trigger={"b": True},
        occupants={"a": [("r1", (-0.5, 0.0))]},
    )
    assert len(jobs1) == 1
    assert b.just_arrived == {"r1": False}
    # 20s with no observations (TF lag): cabin must not re-depart.
    _, jobs2 = _run_ticks(runtimes, doors, start=now, n=int(20.0 / DT))
    assert jobs2 == []
    assert not b.departing


def test_tick_source_door_closed_through_teleport_under_outside_call():
    """Regression: source door must close before the teleport and stay shut for the whole trip,
    even while an outside call hammers it. It used to reopen as the rider left and close after."""
    runtimes, doors = _prime_pair()  # travel_time=3.0, doors start closed
    # Board: rider in A, door already closed -> A departs and commits within a couple ticks.
    now, jobs = _run_ticks(runtimes, doors, start=0.0, n=5, occupants={"a": [("r1", (-0.5, 0.0))]})
    assert runtimes["a"].dispatched == {"r1"}
    assert doors["a/door"].state == _DoorState.CLOSED
    assert jobs == []  # still in transit
    # Transit: rider still inside, plus a persistent outside call. Door must not reopen.
    now, jobs = _run_ticks(
        runtimes, doors, start=now, n=75,
        occupants={"a": [("r1", (-0.5, 0.0))]}, outside_trigger={"a": True},
    )
    assert doors["a/door"].state == _DoorState.CLOSED
    assert jobs == []
    # Arrival: teleport fires exactly once, source door still closed at that moment.
    _, jobs = _run_ticks(
        runtimes, doors, start=now, n=60,
        occupants={"a": [("r1", (-0.5, 0.0))]}, outside_trigger={"a": True},
    )
    assert jobs == [("a", "b", [("r1", (-0.5, 0.0))])]
    assert doors["a/door"].state == _DoorState.CLOSED


def test_tick_accept_outside_calls_false_blocks_door():
    """accept_outside_calls=False: outside trigger does not open the door."""
    runtimes, doors = _prime_pair(accept_outside_calls_a=False)
    now, jobs = _run_ticks(runtimes, doors, start=0.0, n=100, outside_trigger={"a": True})
    assert jobs == []
    assert doors["a/door"].state == _DoorState.CLOSED


def test_tick_accept_outside_calls_false_still_receives_arrival():
    """accept_outside_calls=False: destination receives teleport arrival and opens door."""
    runtimes, doors = _prime_pair(accept_outside_calls_b=False)
    # Occupant in A; B-side trigger... but B doesn't accept outside calls.
    # A's own outside trigger drives departure.
    now, jobs = _run_ticks(
        runtimes, doors, start=0.0, n=250,
        outside_trigger={"a": True},  # A is called from outside: valid because A accepts.
        occupants={"a": [("r1", (-0.5, 0.0))]},
    )
    # A should depart... but with outside_trigger=True on A, the departure is cancelled each tick.
    # Instead test directly: inject departing + pending.
    runtimes2, doors2 = _prime_pair(accept_outside_calls_b=False)
    b2 = runtimes2["b"]
    b2.arriving_eta = 5.0
    b2.pending_occupants = (("r1", (9.5, 0.0)),)
    dr_b2 = doors2["b/door"]
    dr_b2.last_trigger_sim_time = -100.0
    result = _step_elevator(b2, dr_b2, None, occupants=[], near_door=False, outside_trigger=False, now=5.0)
    assert dr_b2.last_trigger_sim_time == 5.0
    assert result.teleport_job is not None


# ---------------------------------------------------------------------------
# _compute_teleport_destinations
# ---------------------------------------------------------------------------


def _runtime_dict_with_pair(a_pos: tuple[float, float, float], b_pos: tuple[float, float, float]) -> dict[str, _ElevatorRuntime]:
    return {
        "a": _elev_runtime(_elevator(name="a", position=a_pos, destination="b"), door_name="a/door"),
        "b": _elev_runtime(_elevator(name="b", position=b_pos, destination="a"), door_name="b/door"),
    }


def test_compute_teleport_preserves_relative_offset():
    rt = _runtime_dict_with_pair((0.0, 0.0, 0.0), (10.0, 5.0, 0.0))
    out = _compute_teleport_destinations(rt, "a", "b", [("r1", (0.3, -0.4))])
    assert "r1" in out
    assert out["r1"] == pytest.approx((10.3, 4.6))


def test_compute_teleport_multiple_agents():
    rt = _runtime_dict_with_pair((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    out = _compute_teleport_destinations(rt, "a", "b", [
        ("r1", (0.5, 0.0)),
        ("p1", (-0.5, 0.3)),
    ])
    assert out["r1"] == pytest.approx((10.5, 0.0))
    assert out["p1"] == pytest.approx((9.5, 0.3))


def test_compute_teleport_unknown_pair_returns_empty():
    rt = _runtime_dict_with_pair((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    assert _compute_teleport_destinations(rt, "ghost", "b", [("r1", (0.0, 0.0))]) == {}
    assert _compute_teleport_destinations(rt, "a", "ghost", [("r1", (0.0, 0.0))]) == {}


# ---------------------------------------------------------------------------
# elevator door slot + wall geometry helpers (unchanged by model rewrite)
# ---------------------------------------------------------------------------


def _make_elevator(
    name: str = "elev",
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    size: tuple[float, float, float] = (2.0, 2.0, 2.5),
    door_side: str = "+x",
) -> Elevator:
    return Elevator(name=name, position=Position(*position), size=list(size), door_side=door_side)


def test_door_slot_plus_x_inset():
    e = _make_elevator(position=(10.0, 5.0, 0.0), door_side='+x')
    start, end = _door_slot(e)
    assert start.x == pytest.approx(11.0 - DOOR_INSET)
    assert end.x == pytest.approx(11.0 - DOOR_INSET)
    assert start.y == pytest.approx(5.0 - 1.0 + DOOR_INSET)
    assert end.y == pytest.approx(5.0 + 1.0 - DOOR_INSET)
    assert start.z == pytest.approx(0.0)


def test_door_slot_minus_y_inset():
    e = _make_elevator(door_side='-y')
    start, end = _door_slot(e)
    assert start.y == pytest.approx(-1.0 + DOOR_INSET)
    assert end.y == pytest.approx(-1.0 + DOOR_INSET)
    assert start.x == pytest.approx(-1.0 + DOOR_INSET)
    assert end.x == pytest.approx(1.0 - DOOR_INSET)


def test_wall_geometries_plus_x_door():
    e = _make_elevator(door_side='+x')
    walls = {suffix: (size, pose) for suffix, size, pose in _elevator_wall_geometries(e)}
    assert set(walls) == {'back', 'side_pos', 'side_neg'}
    back_size, back_pose = walls['back']
    assert back_size == pytest.approx((WALL_THICKNESS, 2.0, 2.5))
    assert back_pose.position.x == pytest.approx(-1.0)
    assert back_pose.position.y == pytest.approx(0.0)
    side_size, side_pose_pos = walls['side_pos']
    assert side_size == pytest.approx((2.0, WALL_THICKNESS, 2.5))
    assert side_pose_pos.position.y == pytest.approx(1.0)
    assert walls['side_neg'][1].position.y == pytest.approx(-1.0)


def test_wall_geometries_minus_y_door():
    e = _make_elevator(door_side='-y')
    walls = {suffix: (size, pose) for suffix, size, pose in _elevator_wall_geometries(e)}
    back_size, back_pose = walls['back']
    assert back_size == pytest.approx((2.0, WALL_THICKNESS, 2.5))
    assert back_pose.position.y == pytest.approx(1.0)
    assert walls['side_pos'][0] == pytest.approx((WALL_THICKNESS, 2.0, 2.5))
    assert walls['side_pos'][1].position.x == pytest.approx(1.0)
    assert walls['side_neg'][1].position.x == pytest.approx(-1.0)


def test_wall_z_at_top_of_cabin():
    for _suffix, _size, pose in _elevator_wall_geometries(_make_elevator()):
        assert pose.position.z == pytest.approx(1.25)
