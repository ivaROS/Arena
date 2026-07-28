"""Hybrid adapter (Option B): DynamicGap (nav2) + the async SICNav bridge, arbitrated by a cmd_vel mux.

Subclasses `Nav2Adapter` so DynamicGap keeps the full nav2 goto stack (navigate_to_pose action,
bt_navigator, planner_server, costmaps). On top of that it spawns the `arena_planners` SICNav *bridge*
as an async edge node (the smooth SICNav-np, distinct from the wiggly nav2py controller), publishing
`cmd_vel_sicnav`. The `HybridBringup` launch turns on the `hybrid_mux` substrate (DynamicGap ->
cmd_vel_dgap, plus the cmd_vel_mux + controller_switch nodes); the mux forwards the selected planner
to `cmd_vel`. The switch signal (controller_selector) is the seam a future RL policy plugs into.

Goal transport: DynamicGap uses the inherited navigate_to_pose action; the SICNav bridge needs a
`goal_pose` topic + `/plan`. `/plan` is published by planner_server during navigation; `goal_pose` is
re-enabled here via `publish_goal_loop` (Nav2Adapter no-ops it). The bridge's episode lifecycle is
driven by request_reset/request_cancel, mirroring DrlAdapter.
"""

from __future__ import annotations

import asyncio
import uuid
from typing import TYPE_CHECKING, ClassVar

from arena_robots.bringup.mobile.hybrid import HybridBringup
from arena_robots.clients.goto_pose import GotoPoseClient
from arena_robots.task_kinds import TaskKind

from task_generator.tasks.robots.adapters import AdapterMeta
from task_generator.tasks.robots.adapters.mobile import MobileAdapter
from task_generator.tasks.robots.adapters.mobile.nav2 import Nav2Adapter
from task_generator.tasks.robots.request import GoToPhase, TaskPhase

if TYPE_CHECKING:
    from task_generator.manager.robot_manager.robot_manager import RobotManager
    from task_generator.shared import Pose
    from task_generator.tasks.robots.adapters import ResetContext


# Do NOT re-apply @requires_map_server: it monkey-patches Nav2Adapter.ensure_services, which
# HybridAdapter already inherits — re-applying would double-wrap the map-server requirement.
@AdapterMeta.attach(
    accepts={TaskKind.GOTO_POSE},
    bringup=HybridBringup,
    client=GotoPoseClient,
    cap="mobile",
    displays=Nav2Adapter._meta().displays,
)
class HybridAdapter(Nav2Adapter):
    kind: ClassVar[str] = "hybrid"

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._edge_node: object = None
        self._run_loop_task: asyncio.Task | None = None
        self._current_phase: TaskPhase | None = None

    async def publish_goal_loop(self) -> None:
        # Nav2Adapter no-ops this (nav2 uses the action). Re-enable the <ns>/goal_pose republish so
        # the SICNav bridge's `goal_pose` datasource (and any /plan bridge) is fed.
        return await MobileAdapter.publish_goal_loop(self)

    async def wait_until_ready(
        self,
        robot: RobotManager,
        node_paths: set[str],
    ) -> None:
        # Wait for the nav2/DynamicGap stack (bt_navigator) first — it provides /plan the bridge needs.
        await super().wait_until_ready(robot, node_paths)

        from arena_planners.bridge.edge_node import PlannerEdgeNode  # noqa: PLC0415
        from arena_planners.resolver import planner_dir, resolve  # noqa: PLC0415

        res = resolve("sicnav")
        sub = planner_dir("sicnav")
        base_frame = robot._config.model_params.base_frame  # noqa: SLF001
        source_frame = robot.frame.tf(base_frame)
        robot.node.get_logger().info(
            f"Hybrid: spawning SICNav bridge edge node for robot={robot.robot.name!r} ns={robot.namespace}"
        )

        edge_node = PlannerEdgeNode(
            node_name="sicnav_edge_node",
            manifest=str(sub / "planner.yaml"),
            planner_command=["ros2", "run", res.package_name, "python", str(sub / "planner.py")],
            cmd_vel_topic=self.bringup.cmd_vel_topic,
            namespace=str(robot.namespace),
            source_frame=source_frame,
            target_frame="map",
        )
        robot.node.executor.add_node(edge_node)

        try:
            await edge_node.setup()  # spawns the SICNav subprocess; blocks on InitAck (CasADi/IPOPT) <= 60s
        except BaseException as exc:
            robot.node.get_logger().error(f"Hybrid: SICNav edge setup failed for {robot.robot.name!r}: {exc!r}")
            try:
                await edge_node.teardown()
            except BaseException as teardown_exc:
                robot.node.get_logger().error(f"Hybrid: SICNav edge teardown after setup failure also failed: {teardown_exc!r}")
            robot.node.executor.remove_node(edge_node)
            raise

        self._edge_node = edge_node
        self._run_loop_task = asyncio.ensure_future(edge_node.run_loop())

        def _on_run_loop_done(task: asyncio.Task) -> None:
            if task.cancelled():
                return
            exc = task.exception()
            if exc is None:
                return
            import traceback  # noqa: PLC0415

            tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
            robot.node.get_logger().error(f"Hybrid: SICNav run_loop for {robot.robot.name!r} crashed:\n{tb}")

        self._run_loop_task.add_done_callback(_on_run_loop_done)

    async def dispatch_phase(
        self,
        phase: TaskPhase,
        robot: RobotManager,
    ) -> None:
        # Nav2Adapter.dispatch_phase sends the navigate_to_pose action (DynamicGap) and sets robot._goal_pos.
        await super().dispatch_phase(phase, robot)
        self._current_phase = phase
        if self._edge_node is not None:
            assert isinstance(phase, GoToPhase)
            x, y, theta = phase.pose.to_2d()
            await self._edge_node.request_reset(
                episode_id=str(id(phase)),
                initial_state={"goal_pose": {"x": x, "y": y, "theta": theta}},
            )

    async def on_reset(self, robot: RobotManager, ctx: ResetContext) -> None:
        await super().on_reset(robot, ctx)
        if self._edge_node is not None and self._current_phase is not None:
            await self._edge_node.request_reset(
                episode_id=str(uuid.uuid4().hex),
                initial_state=None,
            )

    async def on_move(
        self,
        pose: Pose,
        robot: RobotManager,
    ) -> None:
        if self._edge_node is not None:
            await self._edge_node.request_cancel()
        await super().on_move(pose, robot)

    async def _teardown_edge_node(self) -> None:
        # Mirror DrlAdapter: defined for symmetry but not wired to a hook (no Adapter shutdown hook
        # exists). The SICNav subprocess lives for the process lifetime (re-init is ~60s CasADi/IPOPT).
        if self._run_loop_task is not None:
            self._run_loop_task.cancel()
            try:
                await self._run_loop_task
            except (asyncio.CancelledError, Exception):
                pass
            self._run_loop_task = None
        if self._edge_node is not None:
            await self._edge_node.teardown()
            self._edge_node = None
