import abc
import asyncio
import typing
from collections.abc import Iterable, Mapping, Sequence

from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_rclpy_mixins.registry import AsyncFactoryRegistry as Registry
from arena_rclpy_mixins.shared import Namespace
from arena_runtime._node import NodeInterface
from arena_runtime.sim import BaseSim
from task_generator.constants import Constants
from task_generator.shared import Door, DynamicObstacle, Obstacle, Robot, Wall
from task_generator.simulators.human.utils import KnownObstacle, KnownObstacles, ObstacleLayer


class BaseHumanSimulator(NodeInterface, abc.ABC):
    _known_obstacles: KnownObstacles

    def __init__(self, *args: object, namespace: Namespace, simulator: BaseSim, **kwargs: object) -> None:
        """
        Initialize human simulator.

        Args:
            namespace: global namespace
            simulator: Simulator instance
        """
        super().__init__(*args, **kwargs)
        self._simulator = simulator
        self._namespace = namespace

        self._known_obstacles = KnownObstacles[Obstacle]()

        self._ped_positions_xy: dict[str, tuple[float, float]] = {}
        self.node.create_subscription(
            Pedestrians,
            self._namespace("arena_peds"),
            self._on_arena_peds,
            10,
        )
        self._simulator.attach_human_simulator(self)

    def _on_arena_peds(self, msg: Pedestrians) -> None:
        self._ped_positions_xy = {p.name: (p.pose.position.x, p.pose.position.y) for p in msg.pedestrians}

    def pedestrian_positions_xy(self) -> Iterable[tuple[str, tuple[float, float]]]:
        return list(self._ped_positions_xy.items())

    async def pedestrian_teleport(self, destinations: Mapping[str, tuple[float, float]]) -> bool:
        """Teleport tracked pedestrians to given (x, y). Default impl asks the sim to move them."""
        if not destinations:
            return True
        peds_msg = Pedestrians()
        for name, (x, y) in destinations.items():
            ped = Pedestrian()
            ped.name = name
            cur = self._ped_positions_xy.get(name)
            ped.pose.position.x = x
            ped.pose.position.y = y
            ped.pose.position.z = 0.0
            peds_msg.pedestrians.append(ped)
            if cur is not None:
                self._ped_positions_xy[name] = (x, y)
        results = await self._simulator.pedestrian_update(peds_msg)
        return all(results)

    async def spawn_obstacles(self, obstacles: Sequence[Obstacle], layer: ObstacleLayer = ObstacleLayer.INUSE):
        """Spawns static obstacles.

        Args:
            obstacles (Sequence[Obstacle]): Static obstacles to spawn.
            layer (ObstacleLayer, optional): Layer to assign to spawned obstacles. Defaults to ObstacleLayer.INUSE.
        """
        self._logger.debug(f'spawning {len(obstacles)} static obstacles')

        futures: list[typing.Awaitable] = []
        to_register: list[KnownObstacle[Obstacle]] = []
        to_move: list[Obstacle] = []

        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                to_move.append(known.obstacle)
                known.layer = layer
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                )
            if not known.spawned:
                to_register.append(known)
        if to_move:
            futures.append(self._simulator.obstacle_move(to_move))

        to_spawn: list[Obstacle] = []
        for known, obstacle in zip(to_register, await self._spawn_obstacles_impl([known.obstacle for known in to_register]), strict=False):
            if not obstacle:
                continue
            known.obstacle = obstacle
            known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                to_spawn.append(known.obstacle)
            known.layer = layer

        if to_spawn:
            futures.append(self._simulator.obstacle_spawn(to_spawn))
        await asyncio.gather(*futures)

    async def spawn_dynamic_obstacles(self, obstacles: typing.Sequence[DynamicObstacle]):
        """Spawns dynamic obstacles.

        Args:
            obstacles (typing.Sequence[DynamicObstacle]): Dynamic obstacles to spawn.
        """
        self._logger.debug(f'spawning {len(obstacles)} dynamic obstacles')

        futures: list[typing.Awaitable] = []
        to_register: list[KnownObstacle[DynamicObstacle]] = []
        to_move: list[DynamicObstacle] = []

        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                to_move.append(known.obstacle)
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(name=obstacle.name, obstacle=obstacle)
            if not known.spawned:
                to_register.append(known)
        if to_move:
            futures.append(self._simulator.pedestrian_move(to_move))

        to_spawn: list[DynamicObstacle] = []
        for known, obstacle in zip(to_register, await self._spawn_dynamic_obstacles_impl([known.obstacle for known in to_register]), strict=False):
            self._logger.debug(f"Spawned dynamic obstacle: {obstacle}")
            if not obstacle:
                continue

            known.obstacle = obstacle
            known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                to_spawn.append(known.obstacle)
            known.layer = ObstacleLayer.INUSE

        if to_spawn:
            futures.append(self._simulator.pedestrian_spawn(to_spawn))
        await asyncio.gather(*futures)

    async def spawn_world(
        self,
        walls: Sequence[Wall],
        doors: Sequence[Door],
    ):
        """Spawns world elements.

        Args:
            walls (Sequence[Wall]): _description_
            doors (Sequence[Door]): _description_
        """
        await asyncio.gather(
            self._simulator.spawn_doors(doors),
            self._simulator.spawn_walls(walls, clear_existing=False),
            self._spawn_walls_impl(walls),
            self._spawn_doors_impl(doors),
        )

    async def unuse_obstacles(self):
        """
        Prepares obstacles for reuse or removal.
        """
        self._logger.debug('unusing obstacles')
        await self._remove_obstacles_impl()
        for obstacle in self._known_obstacles.values():
            obstacle.spawned = False
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    async def remove_obstacles(self, purge: ObstacleLayer = ObstacleLayer.UNUSED):
        """Removes obstacles from simulator.

        Args:
            purge (ObstacleLayer, optional): Level of obstacles to remove. Defaults to ObstacleLayer.UNUSED.
        """
        self._logger.debug(f'removing obstacles (level {purge})')
        futures: list[typing.Awaitable] = []

        if purge >= ObstacleLayer.WORLD:
            futures.append(self._simulator.remove_world())

        static = []
        dynamic = []
        for oid, known in list(self._known_obstacles.items()):
            if purge >= known.layer:  # tmp: always respawn all dynamic obstacles
                if isinstance(known.obstacle, DynamicObstacle):
                    dynamic.append(known.obstacle)
                else:
                    static.append(known.obstacle)
                self._known_obstacles.forget(name=oid)

        futures.append(self._simulator.obstacle_delete(static))
        futures.append(self._simulator.pedestrian_delete(dynamic))
        await asyncio.gather(*futures)

    async def spawn_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """Spawns robots.

        Args:
            robots (Sequence[Robot]): Robots to spawn.

        Returns:
            Sequence[bool]: Success of each robot spawn.
        """
        self._logger.debug(f'spawning {len(robots)} robots')
        sim_success = await self._simulator.robot_spawn(robots)
        human_success = await self._spawn_robot_impl(tuple(r for r, s in zip(robots, sim_success, strict=False) if s))
        human_iter = iter(human_success)
        success = (s and next(human_iter) for s in sim_success)
        return tuple(success)

    async def remove_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """Removes robots from the simulation.

        Args:
            robots (Sequence[Robot]): Robots to remove.

        Returns:
            Sequence[bool]: Success of each robot removal.
        """
        self._logger.debug(f'removing {len(robots)} robots')
        sim_success = await self._simulator.robot_delete(robots)
        human_success = await self._remove_robot_impl(tuple(r for r, s in zip(robots, sim_success, strict=False) if s))
        human_iter = iter(human_success)
        success = (s and next(human_iter) for s in sim_success)
        return tuple(success)

    async def move_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """Moves robots.

        Args:
            robots (Sequence[Robot]): Robots to move.

        Returns:
            Sequence[bool]: Success of each robot move.
        """
        self._logger.debug(f'moving {len(robots)} robots')
        sim_success = await self._simulator.robot_move(robots)
        human_success = await self._move_robot_impl(tuple(r for r, s in zip(robots, sim_success, strict=False) if s))
        human_iter = iter(human_success)
        success = (s and next(human_iter) for s in sim_success)
        return tuple(success)

    # impl

    @abc.abstractmethod
    async def _spawn_obstacles_impl(
        self,
        obstacles: Sequence[Obstacle],
    ) -> Sequence[Obstacle | None]: ...

    @abc.abstractmethod
    async def _spawn_dynamic_obstacles_impl(
        self,
        obstacles: Sequence[DynamicObstacle],
    ) -> Sequence[DynamicObstacle | None]: ...

    @abc.abstractmethod
    async def _remove_obstacles_impl(
        self,
    ) -> bool: ...

    @abc.abstractmethod
    async def _spawn_walls_impl(
        self,
        walls: Sequence[Wall],
    ) -> bool: ...

    @abc.abstractmethod
    async def _spawn_doors_impl(
        self,
        doors: Sequence[Door],
    ) -> bool: ...

    @abc.abstractmethod
    async def _spawn_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]: ...

    @abc.abstractmethod
    async def _remove_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]: ...

    @abc.abstractmethod
    async def _move_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]: ...


HumanSimulatorRegistry = Registry[Constants.HumanSimulator, BaseHumanSimulator]()


@HumanSimulatorRegistry.register(Constants.HumanSimulator.DUMMY)
async def dummy(**kwargs: object) -> BaseHumanSimulator:
    from .dummy import DummyHumanSimulator

    return DummyHumanSimulator(**kwargs)


@HumanSimulatorRegistry.register(Constants.HumanSimulator.HUNAV)
async def lazy_hunavsim(**kwargs: object) -> BaseHumanSimulator:
    from .hunav.hunav import HunavHumanSimulator

    return await HunavHumanSimulator.create(**kwargs)


@HumanSimulatorRegistry.register(Constants.HumanSimulator.ISAAC)
async def isaacsim(**kwargs: object) -> BaseHumanSimulator:
    from .isaac import IsaacHumanSimulator

    return IsaacHumanSimulator(**kwargs)
