from __future__ import annotations

import pytest
from arena_simulation_setup.shared.world import Door, Elevator, Floor
from arena_simulation_setup.utils.geometry import Position

# ---------------------------------------------------------------------------
# Elevator
# ---------------------------------------------------------------------------


def _elev_pos() -> Position:
    return Position(0.0, 0.0, 0.0)


def test_elevator_default_size():
    e = Elevator(name="elev", position=_elev_pos())
    assert e.size == pytest.approx([2.0, 2.0, 2.5])


def test_elevator_default_door_side():
    e = Elevator(name="elev", position=_elev_pos())
    assert e.door_side == '+x'


def test_elevator_default_timings():
    e = Elevator(name="elev", position=_elev_pos())
    assert e.transition_time == pytest.approx(1.0)
    assert e.hold_time == pytest.approx(2.0)
    assert e.travel_time == pytest.approx(3.0)


def test_elevator_material_converter():
    from arena_simulation_setup.tree.assets.Material import MaterialIdentifier
    e = Elevator(name="elev", position=_elev_pos())
    assert isinstance(e.material, MaterialIdentifier)


def test_elevator_position_converter():
    e = Elevator(name="elev", position=Position(1.0, 2.0, 3.0))
    assert e.position.x == pytest.approx(1.0)


def test_elevator_cabin_corners_centered():
    e = Elevator(name="elev", position=Position(5.0, -3.0, 0.0), size=[4.0, 2.0, 2.5])
    corners = e.cabin_corners()
    assert len(corners) == 4
    xs = sorted({c.x for c in corners})
    ys = sorted({c.y for c in corners})
    assert xs == pytest.approx([3.0, 7.0])  # cx +- hw
    assert ys == pytest.approx([-4.0, -2.0])  # cy +- hh


# ---------------------------------------------------------------------------
# Door
# ---------------------------------------------------------------------------


def test_door_corners_count():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(1.0, 0.0, 0.0),
        width=0.2,
    )
    assert len(d.corners) == 4


def test_door_corners_perpendicular_offset():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(1.0, 0.0, 0.0),
        width=0.2,
    )
    corners = d.corners
    # y coordinates should be +/- width/2 = +/-0.1
    ys = sorted({c.y for c in corners})
    assert ys[0] == pytest.approx(-0.1, abs=1e-6)
    assert ys[1] == pytest.approx(0.1, abs=1e-6)


def test_door_corners_non_axis_aligned():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(0.0, 1.0, 0.0),
        width=0.2,
    )
    corners = d.corners
    assert len(corners) == 4
    # perpendicular to (0,1,0) is (-1,0,0) so x should be +/-0.1
    xs = sorted({c.x for c in corners})
    assert xs[0] == pytest.approx(-0.1, abs=1e-6)
    assert xs[1] == pytest.approx(0.1, abs=1e-6)


def test_door_zero_width_corners():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(1.0, 0.0, 0.0),
        width=0.0,
    )
    corners = d.corners
    assert len(corners) == 4


def test_door_activation_distance_scalar_broadcast():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(1.0, 0.0, 0.0),
        activation_distance=1.5,
    )
    assert d.activation_distance == (1.5, 1.5)


def test_door_activation_distance_tuple():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(1.0, 0.0, 0.0),
        activation_distance=(1.5, 0.0),
    )
    assert d.activation_distance == (1.5, 0.0)


def test_door_default_kind_and_timings():
    d = Door(
        name="door",
        start=Position(0.0, 0.0, 0.0),
        end=Position(1.0, 0.0, 0.0),
    )
    assert d.kind == 'sliding'
    assert d.transition_time == pytest.approx(1.0)
    assert d.hold_time == pytest.approx(2.0)
    assert d.activation_distance == (3.0, 3.0)


# ---------------------------------------------------------------------------
# Floor
# ---------------------------------------------------------------------------


def test_floor_default_lengths():
    f = Floor(name="floor", pos=Position(0.0, 0.0, 0.0))
    assert f.x_length == pytest.approx(20.0)
    assert f.y_length == pytest.approx(20.0)


def test_floor_position_converter():
    f = Floor(name="floor", pos=Position(5.0, 6.0, 0.0))
    assert f.pos.x == pytest.approx(5.0)
    assert f.pos.y == pytest.approx(6.0)


def test_floor_material_is_identifier():
    from arena_simulation_setup.tree.assets.Material import MaterialIdentifier
    f = Floor(name="floor", pos=Position(0.0, 0.0, 0.0))
    assert isinstance(f.material, MaterialIdentifier)
