import asyncio
import typing
from collections.abc import Callable, Collection, Sequence
from typing import Any

import shapely
import shapely.affinity
from arena_runtime._node import NodeInterface
from arena_runtime.sim import BaseSim
from arena_simulation_setup.tree.World import LevelDescription, WorldDescription

from task_generator.manager.realizer import Realizer
from task_generator.shared import (
    DynamicObstacle,
    Obstacle,
    Pose,
    Robot,
)
from task_generator.simulators.human import BaseHumanSimulator
from task_generator.simulators.human.utils import ObstacleLayer


class EnvironmentManager(NodeInterface):
    _human_simulator: BaseHumanSimulator
    _simulator: BaseSim
    _realizer: Realizer

    _walls_geometry: shapely.MultiLineString
    _static_polygons: dict[str, shapely.Polygon]

    def __init__(
        self,
        *args: object,
        simulator: BaseSim,
        human_simulator: BaseHumanSimulator,
        realizer: Realizer,
        **kwargs: object,
    ):
        super().__init__(*args, **kwargs)

        self._simulator = simulator
        self._human_simulator = human_simulator
        self._realizer = realizer

        self._walls_geometry = shapely.MultiLineString()
        self._static_polygons = {}

    def realize(self, target: object) -> object:
        return self._realizer.realize(target)

    def ezilear(self, target: Pose) -> Pose:
        return self._realizer.ezilear(target)

    @property
    def walls_geometry(self) -> shapely.MultiLineString:
        """Map-frame line geometry of all world walls and doors."""
        return self._walls_geometry

    @property
    def static_polygons(self) -> dict[str, shapely.Polygon]:
        """Map-frame footprint polygons of every static obstacle currently spawned,
        keyed by obstacle name. Includes both WORLD-layer entities and INUSE
        episode-spawned obstacles. Pedestrians are not included — consumers should
        read those from the `arena_peds` topic."""
        return self._static_polygons

    async def _resolve_polygon(self, obstacle: Obstacle) -> shapely.Polygon | None:
        try:
            view = await obstacle.model.resolve()
        except FileNotFoundError:
            return None
        bounds = view.bounds
        if bounds is None:
            return None
        poly = shapely.Polygon(bounds)
        poly = shapely.affinity.rotate(poly, obstacle.pose.orientation.to_yaw(), origin=(0, 0), use_radians=True)
        return shapely.affinity.translate(poly, xoff=obstacle.pose.position.x, yoff=obstacle.pose.position.y)

    async def _cache_polygons(self, obstacles: Sequence[Obstacle]) -> None:
        polys = await asyncio.gather(*(self._resolve_polygon(o) for o in obstacles))
        for obstacle, poly in zip(obstacles, polys, strict=True):
            if poly is not None:
                self._static_polygons[obstacle.name] = poly

    def _sync_static_polygons(self) -> None:
        """Drop polygons whose obstacle is no longer registered in the human simulator."""
        alive = set(self._human_simulator._known_obstacles.keys())
        self._static_polygons = {n: p for n, p in self._static_polygons.items() if n in alive}

    async def spawn_world_obstacles(self, world: WorldDescription | LevelDescription, level_id: str = ""):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "world"
        """
        await self._spawn_world_obstacles(world, level_id)

    async def _spawn_world_obstacles(self, world: WorldDescription | LevelDescription, level_id: str = "") -> None:

        def _match_level_id(fid: str | None) -> bool:
            target_id = level_id
            if target_id == "":
                return True
            else:
                if fid is not None:
                    return target_id == fid
                else:
                    return False

        _world = WorldDescription.from_levels(world) if isinstance(world, LevelDescription) else world
        walls = tuple(self._realizer.realize(w, fid) for fid, level in _world.levels.items() if _match_level_id(fid) for w in level.all_walls)
        doors = tuple(self._realizer.realize(d, fid) for fid, level in _world.levels.items() if _match_level_id(fid) for d in level.all_doors)
        floors = tuple(self._realizer.realize(f, fid) for fid, level in _world.levels.items() if _match_level_id(fid) for f in level.all_floors)
        elevators = tuple(self._realizer.realize(e, fid) for fid, level in _world.levels.items() if _match_level_id(fid) for e in level.all_elevators)
        statics = tuple(self._realizer.realize(s, fid) for fid, level in _world.levels.items() if _match_level_id(fid) for s in level.all_static_entities)

        line_strings: list[shapely.LineString] = []
        for w in walls:
            line_strings.append(shapely.LineString([(w.start.x, w.start.y), (w.end.x, w.end.y)]))
        for d in doors:
            line_strings.append(shapely.LineString([(d.start.x, d.start.y), (d.end.x, d.end.y)]))
        if line_strings:
            floor_walls = shapely.MultiLineString(line_strings)
            if self._walls_geometry.is_empty:
                self._walls_geometry = floor_walls
            else:
                self._walls_geometry = shapely.MultiLineString(
                    [
                        *self._walls_geometry.geoms,
                        *floor_walls.geoms,
                    ]
                )

        await self._cache_polygons(statics)

        futures: list[typing.Awaitable] = []
        if floors:
            futures.append(self._simulator.spawn_floors(floors))
        if walls or doors:
            futures.append(self._human_simulator.spawn_world(walls, doors))
        futures.append(self._human_simulator.spawn_obstacles(statics, layer=ObstacleLayer.WORLD))
        if elevators:
            futures.append(self._simulator.spawn_elevators(elevators))

        await asyncio.gather(*futures)

    async def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        """
        realized = tuple(self._realizer.realize(obstacle, obstacle.level_id or "") for obstacle in setups)
        await self._human_simulator.spawn_dynamic_obstacles(realized)

    async def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        """
        realized = tuple(self._realizer.realize(obstacle, obstacle.level_id or "") for obstacle in setups)
        await self._cache_polygons(realized)
        await self._human_simulator.spawn_obstacles(realized)

    async def spawn_robot(self, robots: Sequence[Robot]) -> Sequence[Robot]:
        """
        Loads given robot into the simulator
        """
        await self._human_simulator.spawn_robot(robots=tuple(map(self.realize, robots)))
        return robots

    async def move_robot(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """
        Moves given robot
        """
        return await self._human_simulator.move_robot(tuple(map(self.realize, robots)))

    async def remove_robot(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """
        Deletes given robot
        """
        return await self._human_simulator.remove_robot(tuple(map(self.realize, robots)))

    async def respawn(self, callback: Callable[[], typing.Awaitable[Any]]):
        """
        Unuse obstacles, (re-)use them in callback, finally remove unused obstacles
        @callback: Function to call between unuse and remove
        """
        await self._human_simulator.unuse_obstacles()
        await callback()
        await self._human_simulator.remove_obstacles(purge=ObstacleLayer.UNUSED)
        self._sync_static_polygons()

    async def reset(self, purge: ObstacleLayer = ObstacleLayer.INUSE):
        """
        Unuse and remove all obstacles
        """
        await self._human_simulator.remove_obstacles(purge=purge)
        self._sync_static_polygons()
        if purge >= ObstacleLayer.WORLD:
            self._walls_geometry = shapely.MultiLineString()

    async def step(self, n: int = 1) -> bool:
        return await self._simulator.step(n)

    async def before_reset_episode(self) -> bool:
        return await self._simulator.before_reset_episode()

    async def after_reset_episode(self) -> bool:
        return await self._simulator.after_reset_episode()
