"""Tests for WorldManager.get_positions_on_map and its grid helpers."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

try:
    from arena_rclpy_mixins.Time import Time
    from arena_simulation_setup.tree.World.World import LevelDescription
    from arena_simulation_setup.utils.geometry import Position as GeoPosition
    from task_generator.manager.world_manager.utils import WorldLayers, WorldMap, WorldOccupancy
    from task_generator.manager.world_manager.world_manager import (
        WorldManager,
        _disc_kernel,
        _occupancy_to_available,
        _sample_grid_positions,
    )
    from task_generator.shared import Position, PositionRadius
except ImportError:
    pytestmark = pytest.mark.skip(reason="ROS2 not available")


def empty_grid(h: int, w: int) -> np.ndarray:
    return np.full((h, w), WorldOccupancy.EMPTY, dtype=np.uint8)


def make_map(grid: np.ndarray, resolution: float = 0.05, origin: tuple[float, float] = (0.0, 0.0)) -> WorldMap:
    return WorldMap(
        occupancy=WorldLayers(walls=WorldOccupancy(grid.copy())),
        origin=Position(x=origin[0], y=origin[1]),
        resolution=resolution,
        time=Time(),
    )


def grid_distance_to_occupied(occupancy: np.ndarray, row: int, col: int) -> float:
    occ = np.argwhere(~WorldOccupancy.not_full(occupancy))
    if len(occ) == 0:
        return float("inf")
    return float(np.min(np.linalg.norm(occ - np.array([row, col]), axis=1)))


class TestDiscKernel:
    def test_normalised_to_unit_sum(self):
        for r in (1.0, 2.5, 4.0, 10.0):
            k = _disc_kernel(r)
            assert k.shape[0] == k.shape[1]
            assert k.sum() == pytest.approx(1.0, abs=1e-6)

    def test_disc_geometry(self):
        k = _disc_kernel(3.0)
        # Kernel size = 2*ceil(3)+1 = 7
        assert k.shape == (7, 7)
        # Corner (-3,-3) is at L2 distance sqrt(18) > 3 -> outside disc, zero.
        assert k[0, 0] == 0.0
        # Centre is inside.
        assert k[3, 3] > 0.0
        # Axial cell (-3, 0) at L2 = 3 -> inside disc.
        assert k[0, 3] > 0.0


class TestOccupancyToAvailable:
    def test_returned_indices_in_input_frame(self):
        grid = empty_grid(20, 20)
        for sd in (1.0, 3.0, 5.0):
            avail = _occupancy_to_available(grid, safe_dist_cells=sd)
            assert avail.size > 0
            assert int(avail[:, 0].max()) < 20
            assert int(avail[:, 1].max()) < 20

    def test_off_map_treated_as_occupied(self):
        grid = empty_grid(20, 20)
        avail = _occupancy_to_available(grid, safe_dist_cells=3.0)
        for r, c in avail:
            assert r >= 3 and r <= 16
            assert c >= 3 and c <= 16

    def test_excludes_cells_within_safe_dist_of_wall(self):
        grid = empty_grid(30, 30)
        grid[:, 10] = WorldOccupancy.FULL
        avail = _occupancy_to_available(grid, safe_dist_cells=2.5)
        for _, c in avail:
            assert abs(int(c) - 10) >= 2.5 - 1e-6

    def test_accepts_diagonal_cell_outside_disc(self):
        grid = empty_grid(20, 20)
        grid[10, 10] = WorldOccupancy.FULL
        avail = _occupancy_to_available(grid, safe_dist_cells=2.0)
        coords = {(int(r), int(c)) for r, c in avail}
        assert (12, 12) in coords  # L2 = 2*sqrt(2) ≈ 2.83 > 2; L∞ box would have rejected

    def test_full_grid_returns_empty(self):
        grid = np.full((20, 20), WorldOccupancy.FULL, dtype=np.uint8)
        avail = _occupancy_to_available(grid, safe_dist_cells=1.0)
        assert len(avail) == 0


class TestSampleGridPositions:
    def test_returns_n_distinct_cells(self):
        grid = empty_grid(40, 40)
        cells = _sample_grid_positions(grid, n=8, safe_dist_cells=2.0, rng=np.random.default_rng(0))
        assert cells.shape == (8, 2)
        assert len({tuple(c) for c in cells}) == 8

    @pytest.mark.parametrize("safe_dist_cells", [1.0, 2.5, 5.0])
    def test_clearance_from_walls(self, safe_dist_cells):
        grid = empty_grid(40, 60)
        grid[:, 30] = WorldOccupancy.FULL
        rng = np.random.default_rng(1)
        cells = _sample_grid_positions(grid, n=5, safe_dist_cells=safe_dist_cells, rng=rng)
        for r, c in cells:
            d = grid_distance_to_occupied(grid, int(r), int(c))
            assert d >= safe_dist_cells - 1e-6, f"cell ({r},{c}) only {d} from wall, need {safe_dist_cells}"

    def test_pairwise_distance_at_least_safe_dist(self):
        grid = empty_grid(80, 80)
        sd = 5.0
        cells = _sample_grid_positions(grid, n=20, safe_dist_cells=sd, rng=np.random.default_rng(2))
        diffs = cells[:, None, :] - cells[None, :, :]
        dists = np.linalg.norm(diffs, axis=2).astype(float)
        np.fill_diagonal(dists, np.inf)
        assert dists.min() >= sd - 1e-6

    def test_raises_when_no_room(self):
        grid = empty_grid(20, 20)
        with pytest.raises(RuntimeError):
            _sample_grid_positions(grid, n=5, safe_dist_cells=15.0, rng=np.random.default_rng(0))

    def test_raises_when_more_requested_than_available(self):
        grid = empty_grid(20, 20)
        # safe_dist_cells=6: feasible count is small; ask for many.
        with pytest.raises(RuntimeError):
            _sample_grid_positions(grid, n=200, safe_dist_cells=6.0, rng=np.random.default_rng(0))

    def test_deterministic_under_same_seed(self):
        grid = empty_grid(40, 40)
        a = _sample_grid_positions(grid, n=8, safe_dist_cells=3.0, rng=np.random.default_rng(42))
        b = _sample_grid_positions(grid, n=8, safe_dist_cells=3.0, rng=np.random.default_rng(42))
        assert np.array_equal(a, b)

    def test_handles_narrow_passage(self):
        h, w = 50, 50
        grid = np.full((h, w), WorldOccupancy.FULL, dtype=np.uint8)
        grid[:, 23:28] = WorldOccupancy.EMPTY  # width-5 vertical corridor
        cells = _sample_grid_positions(grid, n=3, safe_dist_cells=2.0, rng=np.random.default_rng(0))
        for _, c in cells:
            assert 23 <= int(c) <= 27


class TestGetPositionsOnMap:
    @staticmethod
    def make_wm(grid: np.ndarray, resolution: float = 0.05, seed: int = 0) -> WorldManager:
        wm = WorldManager.__new__(WorldManager)
        wm._map = make_map(grid, resolution=resolution)
        wm._detected_walls = None
        rng = np.random.default_rng(seed)
        fake_node = SimpleNamespace(conf=SimpleNamespace(General=SimpleNamespace(RNG=SimpleNamespace(value=rng))))
        wm._NodeInterface__node = fake_node  # NodeInterface uses name-mangled storage
        return wm

    def test_returns_n_positions(self):
        wm = self.make_wm(empty_grid(60, 60))
        out = wm.get_positions_on_map(n=4, safe_dist=0.4, forbid=False)
        assert len(out) == 4
        for p in out:
            assert isinstance(p, Position)

    def test_world_pairwise_distance_respects_safe_dist(self):
        wm = self.make_wm(empty_grid(80, 80))
        safe_dist = 0.5
        out = wm.get_positions_on_map(n=8, safe_dist=safe_dist, forbid=False)
        coords = np.array([[p.x, p.y] for p in out])
        diffs = coords[:, None, :] - coords[None, :, :]
        dists = np.linalg.norm(diffs, axis=2)
        np.fill_diagonal(dists, np.inf)
        assert dists.min() >= safe_dist - wm.resolution

    def test_forbid_persists_across_calls(self):
        wm = self.make_wm(empty_grid(80, 80), seed=7)
        safe_dist = 0.4
        first = wm.get_positions_on_map(n=1, safe_dist=safe_dist, forbid=True)
        second = wm.get_positions_on_map(n=1, safe_dist=safe_dist, forbid=False)
        d = float(np.linalg.norm(np.array([first[0].x - second[0].x, first[0].y - second[0].y])))
        assert d >= safe_dist - wm.resolution

    def test_forbid_false_does_not_persist(self):
        wm = self.make_wm(empty_grid(80, 80), seed=11)
        before = WorldOccupancy.full(wm.map.occupancy.grid).sum()
        wm.get_positions_on_map(n=3, safe_dist=0.3, forbid=False)
        after = WorldOccupancy.full(wm.map.occupancy.grid).sum()
        assert before == after

    def test_raises_when_map_too_tight(self):
        wm = self.make_wm(empty_grid(20, 20))
        with pytest.raises(RuntimeError):
            wm.get_positions_on_map(n=5, safe_dist=2.0, forbid=False)

    def test_forbidden_zones_enforced(self):
        wm = self.make_wm(empty_grid(80, 80), seed=3)
        zone = PositionRadius(x=2.0, y=2.0, radius=0.5)
        out = wm.get_positions_on_map(n=3, safe_dist=0.3, forbidden_zones=[zone], forbid=False)
        assert len(out) == 3
        for p in out:
            assert not (1.5 <= p.x <= 2.5 and 1.5 <= p.y <= 2.5), f"{p.x},{p.y} inside forbidden zone"


class TestCoordinateTransforms:
    def test_round_trip_asymmetric(self):
        wm = make_map(empty_grid(40, 80), resolution=0.05, origin=(10.0, -5.0))
        for x, y in [(10.5, -4.9), (12.0, -4.0), (13.5, -3.1)]:
            row, col = wm.tf_pos2grid(Position(x=x, y=y))
            back = wm.tf_grid2pos((row, col))
            assert back.x == pytest.approx(x, abs=wm.resolution)
            assert back.y == pytest.approx(y, abs=wm.resolution)

    def test_orientation(self):
        h, w, res, ox, oy = 40, 80, 0.05, 10.0, -5.0
        wm = make_map(empty_grid(h, w), resolution=res, origin=(ox, oy))
        max_y = oy + h * res
        assert tuple(int(v) for v in wm.tf_pos2grid(Position(x=ox, y=max_y))) == (0, 0)
        assert tuple(int(v) for v in wm.tf_pos2grid(Position(x=ox, y=oy))) == (h, 0)


class TestRenderedSampling:
    @staticmethod
    def make_wm_from_level(level: LevelDescription, seed: int = 0) -> WorldManager:
        wm = WorldManager.__new__(WorldManager)
        wm._map = WorldMap.from_world_description(level, resolution=0.05, time=Time())
        wm._multi_map = None
        wm._detected_walls = None
        rng = np.random.default_rng(seed)
        wm._NodeInterface__node = SimpleNamespace(conf=SimpleNamespace(General=SimpleNamespace(RNG=SimpleNamespace(value=rng))))
        return wm

    def test_positions_land_inside_rendered_zone(self):
        corners = [GeoPosition(10, 5), GeoPosition(30, 5), GeoPosition(30, 15), GeoPosition(10, 15), GeoPosition(10, 5)]
        level = LevelDescription(zones=[LevelDescription.Zone(name="room", corners=corners)])
        wm = self.make_wm_from_level(level)
        out = wm.get_positions_on_map(n=10, safe_dist=0.3, forbid=False)
        assert len(out) == 10
        for p in out:
            assert 10 <= p.x <= 30, f"x={p.x} outside rendered zone [10,30]"
            assert 5 <= p.y <= 15, f"y={p.y} outside rendered zone [5,15]"
