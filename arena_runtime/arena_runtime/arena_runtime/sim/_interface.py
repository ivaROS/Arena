"""Interface definitions for simulator interactions with obstacles, pedestrians, robots, and lifecycle."""

import abc
import asyncio
import typing
from collections.abc import Iterable, Mapping, Sequence

from arena_people_msgs.msg import Pedestrians
from task_generator.shared import (
    Door,
    DynamicObstacle,
    Elevator,
    Floor,
    Obstacle,
    Pose,
    Robot,
    Wall,
)

if typing.TYPE_CHECKING:
    from arena_rclpy_mixins import ArenaMixinNode


@typing.runtime_checkable
class HumanSimulator(typing.Protocol):
    """Capabilities the mechanism layer reads from the attached human simulator."""

    def pedestrian_positions_xy(self) -> Iterable[tuple[str, tuple[float, float]]]: ...

    async def pedestrian_teleport(self, destinations: Mapping[str, tuple[float, float]]) -> bool: ...


class SimLifecycle(abc.ABC):
    """Process-singleton hooks for sim-wide pause/unpause and namespace cleanup."""

    @abc.abstractmethod
    async def pause(self) -> bool: ...

    @abc.abstractmethod
    async def unpause(self) -> bool: ...

    @abc.abstractmethod
    async def cleanup_namespace(self, prefix: str) -> int:
        """Delete all entities under the given namespace prefix. Returns count removed."""
        ...

    def env_prefix(self, env_id: int) -> str:
        """Cleanup-namespace prefix for env_id. Default: slash-nested (USD-style)."""
        return f"env_{env_id}/"

    @abc.abstractmethod
    async def ensure_ready(self) -> None:
        """Block until the underlying sim's services are reachable."""
        ...


class ObstacleITF(abc.ABC):
    """Abstract base class for obstacle management in simulators."""

    @abc.abstractmethod
    async def obstacle_spawn(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        """Spawn obstacles."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def obstacle_move(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        """Move obstacles."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def obstacle_delete(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        """Delete obstacles."""
        raise NotImplementedError()


class PedestrianITF(abc.ABC):
    """Abstract base class for pedestrian management in simulators."""

    @abc.abstractmethod
    async def pedestrian_spawn(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        """Spawn pedestrians."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def pedestrian_move(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        """Teleport pedestrians."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def pedestrian_delete(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        """Delete pedestrians."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def pedestrian_update(self, pedestrians: Pedestrians) -> Sequence[bool]:
        """Navigate pedestrians to position with velocity."""
        raise NotImplementedError()


class RobotITF(abc.ABC):
    """Abstract base class for robot management in simulators."""

    @abc.abstractmethod
    async def robot_spawn(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """Spawn robots."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def robot_move(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """Move robots."""
        raise NotImplementedError()

    @abc.abstractmethod
    async def robot_delete(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """Delete robots."""
        raise NotImplementedError()


class WorldITF(abc.ABC):
    @abc.abstractmethod
    async def spawn_walls(self, walls: Sequence[Wall], clear_existing: bool = True) -> bool:
        """
        Add a list of walls to the simulator.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    async def spawn_floors(self, floors: Sequence[Floor]) -> bool:
        """
        Add a list of floors to the simulator.
        """
        raise NotImplementedError()

    async def remove_world(self) -> bool:
        """
        Remove every spawned world element.
        """
        raise NotImplementedError()


class MechanismITF:
    """Door + elevator orchestration with shim-backed defaults.

    Defaults spawn box geometry, animate doors, and pair-teleport elevator cabins.
    Simulators plug in by overriding the five primitives below. The attached
    HumanSimulator supplies ground-truth ped positions and ped teleport.

    The shim helpers read ``self.node`` (an ArenaMixinNode) for sim_time, the
    rate loop, and logging. That attribute is provided by NodeInterface in the
    BaseSim mixin chain; the annotation below makes the dependency type-visible
    without forcing MechanismITF to inherit NodeInterface directly.
    """

    if typing.TYPE_CHECKING:
        from ._mechanism_shim import _DoorRuntime, _ElevatorRuntime

        node: ArenaMixinNode
        _human_simulator: HumanSimulator | None
        _door_runtime: dict[str, _DoorRuntime]
        _elevator_runtime: dict[str, _ElevatorRuntime]
        _door_primitives: dict[str, list[str]]
        _elevator_primitives: dict[str, list[str]]
        _elevator_doors: dict[str, str]
        _mechanism_loop_task: asyncio.Task | None

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._human_simulator = None
        self._door_runtime = {}
        self._elevator_runtime = {}
        self._door_primitives = {}
        self._elevator_primitives = {}
        self._elevator_doors = {}
        self._mechanism_loop_task = None

    def attach_human_simulator(self, hs: HumanSimulator) -> None:
        """Attach a human simulator for ped position reads and teleport dispatch."""
        self._human_simulator = hs

    # defaults: shim logic. Override only if the simulator has native support.
    async def spawn_doors(self, doors: Sequence[Door]) -> bool:
        from ._mechanism_shim import shim_spawn_doors

        return await shim_spawn_doors(self, doors)

    async def remove_doors(self, names: Sequence[str]) -> bool:
        from ._mechanism_shim import shim_remove_doors

        return await shim_remove_doors(self, names)

    async def spawn_elevators(self, elevators: Sequence[Elevator]) -> bool:
        from ._mechanism_shim import shim_spawn_elevators

        return await shim_spawn_elevators(self, elevators)

    async def remove_elevators(self, names: Sequence[str]) -> bool:
        from ._mechanism_shim import shim_remove_elevators

        return await shim_remove_elevators(self, names)

    async def stop_mechanisms(self) -> None:
        """Cancel the mechanism tick loop if running."""
        if self._mechanism_loop_task is not None and not self._mechanism_loop_task.done():
            self._mechanism_loop_task.cancel()
            try:
                await self._mechanism_loop_task
            except (asyncio.CancelledError, Exception):
                pass
        self._mechanism_loop_task = None

    # primitives: simulators must implement these.
    async def spawn_box(self, name: str, size: tuple[float, float, float], pose: Pose) -> bool:
        """Spawn a static box primitive."""
        raise NotImplementedError

    async def move_box(self, name: str, pose: Pose) -> bool:
        """Move a previously spawned box primitive."""
        raise NotImplementedError

    async def delete_box(self, name: str) -> bool:
        """Delete a previously spawned box primitive."""
        raise NotImplementedError

    async def set_robot_pose(self, sim_path: str, pose: Pose) -> bool:
        """Teleport a robot to the given pose."""
        raise NotImplementedError

    def robot_positions_xy(self) -> Iterable[tuple[str, tuple[float, float]]]:
        """Yield (sim_path, (x, y)) for each tracked robot."""
        raise NotImplementedError

    def robot_pose(self, sim_path: str) -> Pose | None:
        """Return the current full pose of a tracked robot, or None if unavailable."""
        raise NotImplementedError
