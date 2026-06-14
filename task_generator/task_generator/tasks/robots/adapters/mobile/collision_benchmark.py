"""Timed constant-velocity adapter for benchmark collision-recorder checks."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, ClassVar

from arena_robots.bringup.mobile.collision_benchmark import CollisionBenchmarkBringup
from arena_robots.clients.goto_pose import GotoPoseClient
from arena_robots.task_kinds import TaskKind

from task_generator.manager.robot_manager.collision_tracker import CollisionTrackerNode
from task_generator.manager.world_manager.shims import requires_map_server
from task_generator.tasks.robots.adapters import AdapterMeta, ResetContext
from task_generator.tasks.robots.adapters.mobile import MobileAdapter

if TYPE_CHECKING:
    from task_generator.manager.robot_manager.robot_manager import RobotManager
    from task_generator.tasks.robots.request import TaskPhase


@AdapterMeta.attach(
    accepts={TaskKind.GOTO_POSE},
    bringup=CollisionBenchmarkBringup,
    client=GotoPoseClient,
    cap="mobile",
)
@requires_map_server
class CollisionBenchmarkAdapter(MobileAdapter):
    kind: ClassVar[str] = "collision-benchmark"

    def __init__(
        self,
        *args: object,
        duration_s: float = 8.0,
        linear_x: float = 1.0,
        rate_hz: float = 10.0,
        **kwargs: object,
    ):
        super().__init__(*args, duration_s=duration_s, linear_x=linear_x, rate_hz=rate_hz, **kwargs)
        self._duration_s = float(duration_s)
        self._linear_x = float(linear_x)
        self._rate_hz = float(rate_hz)
        self._episode_start_s: float | None = None
        self._drive_task: asyncio.Task | None = None
        self._collision: CollisionTrackerNode | None = None
        polys = self.rm.robot.model.resolve_sync().caps.mobile.polygons_dict
        if polys:
            self._collision = CollisionTrackerNode(self.rm, polys)
            self.rm.node.executor.add_node(self._collision)

    async def on_reset(self, robot: RobotManager, ctx: ResetContext) -> None:
        if self._drive_task is not None:
            self._drive_task.cancel()
            self._drive_task = None
        await super().on_reset(robot, ctx)
        self._episode_start_s = robot.node.sim_time.to_seconds()

    def is_phase_done(self, phase: TaskPhase, robot: RobotManager) -> bool | None:
        del phase
        start = self._episode_start_s
        if start is None:
            self._episode_start_s = robot.node.sim_time.to_seconds()
            return False
        return (robot.node.sim_time.to_seconds() - start) >= self._duration_s

    async def dispatch_phase(self, phase: TaskPhase, robot: RobotManager) -> None:
        del phase, robot
