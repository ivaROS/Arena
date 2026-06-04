"""DRL planner adapter: spawns a venv-isolated planner subprocess via arena_planners bridge."""

from __future__ import annotations

import uuid
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar

from arena_robots.bringup.mobile.drl import DrlBringup
from arena_robots.clients.goto_pose import GotoPoseClient
from arena_robots.task_kinds import TaskKind
from launch.actions import GroupAction

from task_generator.tasks.robots.adapters import AdapterCtx, AdapterMeta
from task_generator.tasks.robots.adapters.mobile import MobileAdapter
from task_generator.tasks.robots.request import GoToPhase, TaskPhase

if TYPE_CHECKING:
    from task_generator.manager.robot_manager.robot_manager import RobotManager
    from task_generator.shared import Pose
    from task_generator.tasks.robots.adapters import ResetContext


@AdapterMeta.attach(
    accepts={TaskKind.GOTO_POSE},
    bringup=DrlBringup,
    client=GotoPoseClient,
    cap="mobile",
)
class DrlAdapter(MobileAdapter):
    kind: ClassVar[str] = "drl"

    def __init__(
        self,
        robot_manager: RobotManager,
        *,
        planner: str,
        observations: dict | None = None,
        rate: float = 10.0,
        **bringup_kwargs: object,
    ) -> None:
        super().__init__(robot_manager, **bringup_kwargs)

        from arena_planners.resolver import ResolverError, planner_dir, resolve  # noqa: PLC0415

        resolved = resolve(planner)
        if resolved.source != "registry":
            raise ResolverError(f"DrlAdapter requires a registry planner; '{planner}' resolved as source={resolved.source!r}. Use mobile:=rosnav_rl or mobile:=nav2 for that planner.")
        if not resolved.package_name:
            raise ResolverError(f"DrlAdapter: planner '{planner}' has no package.xml <name> entry; cannot resolve ros2 run target.")

        self._planner_name = planner
        self._rate = rate
        self._observations_override: dict = dict(observations) if observations else {}

        sub_path: Path = planner_dir(planner)
        planner_script = sub_path / "planner.py"
        self._planner_command: list[str] = ["ros2", "run", resolved.package_name, "python", str(planner_script)]

        import yaml  # noqa: PLC0415

        manifest_path = sub_path / "planner.yaml"
        with open(manifest_path) as fh:
            manifest_dict: dict = yaml.safe_load(fh) or {}

        self._manifest: dict | str
        if self._observations_override:
            manifest_dict.setdefault("observations", {})
            _deep_merge(manifest_dict["observations"], self._observations_override)
            self._manifest = manifest_dict
        else:
            self._manifest = str(manifest_path)

        depends = manifest_dict.get("depends") or {}
        self._depends_global_plan: bool = bool(depends.get("global_plan", False))
        self._global_planner: str = str(bringup_kwargs.get("global_planner", "nav2/navfn"))

        self._handler_metadata: dict = {}
        if self._needs_global_plan():
            from arena_planners.resolver import (  # noqa: PLC0415
                ResolverError,
                resolve_global_planner,
                split_global_planner,
            )

            try:
                parsed = split_global_planner(self._global_planner)
            except ValueError as exc:
                raise ResolverError(f"DrlAdapter: invalid mobile.global_planner: {exc}") from exc
            assert parsed is not None  # _needs_global_plan checked
            family, _kind = parsed
            _launch_path, self._handler_metadata = resolve_global_planner(family)

        self._edge_node: object = None
        self._run_loop_task: object = None
        self._current_phase: TaskPhase | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _needs_global_plan(self) -> bool:
        return self._depends_global_plan and self._global_planner != "none"

    def launch_description(self, ctx: AdapterCtx) -> GroupAction:
        kwargs = dict(self._bringup_kwargs)
        if not self._depends_global_plan:
            kwargs["global_planner"] = "none"
        return GroupAction(
            [
                *self.bringup._launch_actions(
                    use_sim_time=ctx.use_sim_time,
                    frame=ctx.frame,
                    task_generator_node=ctx.task_generator_node,
                    **kwargs,
                ),
            ]
        )

    async def ensure_services(self) -> None:
        await super().ensure_services()
        logger = self.rm.node.get_logger()
        if self._depends_global_plan and self._global_planner == "none":
            logger.warning(f"DRL planner {self._planner_name!r} declares depends.global_plan=true but mobile.global_planner=none; planner will block on empty /plan")
        elif not self._depends_global_plan and self._global_planner != "none":
            logger.debug(f"DRL planner {self._planner_name!r} doesn't need a global plan; skipping global_planner launch")
        if self._needs_global_plan() and self._handler_metadata.get("requires_map_server", False):
            await self.rm.node._world_manager.require_map_server()

    async def wait_until_ready(
        self,
        robot: RobotManager,
        node_paths: set[str],
    ) -> None:
        import asyncio  # noqa: PLC0415

        from arena_planners.bridge.edge_node import PlannerEdgeNode  # noqa: PLC0415

        node_name = "edge_node"
        ns = str(robot.namespace)
        base_frame = robot._config.model_params.base_frame
        source_frame = robot.frame.tf(base_frame)

        robot.node.get_logger().info(f"DRL edge for robot={robot.robot.name!r} planner={self._planner_name!r} ns={ns}")

        edge_node = PlannerEdgeNode(
            node_name=node_name,
            manifest=self._manifest,
            planner_command=self._planner_command,
            namespace=ns,
            source_frame=source_frame,
            target_frame="map",
            cmd_vel_topic=self.bringup.cmd_vel_topic,
        )
        robot.node.executor.add_node(edge_node)

        try:
            await edge_node.setup()
        except BaseException as exc:
            robot.node.get_logger().error(f"DRL edge setup failed for {robot.robot.name!r}: {exc!r}")
            try:
                await edge_node.teardown()
            except BaseException as teardown_exc:
                robot.node.get_logger().error(f"DRL edge teardown after setup failure also failed: {teardown_exc!r}")
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
            robot.node.get_logger().error(f"DRL run_loop for {robot.robot.name!r} crashed:\n{tb}")

        self._run_loop_task.add_done_callback(_on_run_loop_done)

        await super().wait_until_ready(robot, node_paths)

    async def _teardown_edge_node(self) -> None:
        import asyncio  # noqa: PLC0415

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

    # ------------------------------------------------------------------
    # Phase dispatch
    # ------------------------------------------------------------------

    async def dispatch_phase(
        self,
        phase: TaskPhase,
        robot: RobotManager,
    ) -> None:
        assert isinstance(phase, GoToPhase), f"DrlAdapter only accepts GOTO_POSE phases; got {type(phase).__name__} (kind={phase.kind!r})"
        robot._goal_pos = phase.pose  # pylint: disable=protected-access
        self._current_phase = phase

        if self._edge_node is not None:
            x, y, theta = phase.pose.to_2d()
            await self._edge_node.request_reset(
                episode_id=str(id(phase)),
                initial_state={
                    "goal_pose": {"x": x, "y": y, "theta": theta},
                },
            )

    def is_phase_done(self, phase: TaskPhase, robot: RobotManager) -> bool | None:
        return None

    # ------------------------------------------------------------------
    # Reset / move (mirrors Nav2Adapter pattern)
    # ------------------------------------------------------------------

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

        request = robot._current_request  # pylint: disable=protected-access
        if request is None or robot._phase_index >= len(request.phases):  # pylint: disable=protected-access
            return
        await self.dispatch_phase(request.phases[robot._phase_index], robot)  # pylint: disable=protected-access


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------


def _deep_merge(base: dict, override: dict) -> None:
    for key, val in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(val, dict):
            _deep_merge(base[key], val)
        else:
            base[key] = val


__all__ = ["DrlAdapter"]
