"""Nav2 adapter: thin composer of Nav2Bringup + GotoPoseClient."""

from __future__ import annotations

import asyncio
import os
from typing import TYPE_CHECKING, ClassVar

import lifecycle_msgs.msg
from arena_robots.bringup.mobile.nav2 import Nav2Bringup
from arena_robots.clients.goto_pose import GotoPoseClient
from arena_robots.task_kinds import TaskKind
from arena_robots_msgs.action import GotoPose
from nav2_msgs.srv import ClearCostmapAroundRobot, ClearEntireCostmap

from task_generator.manager.world_manager.shims import requires_map_server
from task_generator.tasks.robots.adapters import AdapterDisplayHint, AdapterMeta
from task_generator.tasks.robots.adapters.mobile import MobileAdapter
from task_generator.tasks.robots.request import GoToPhase, TaskPhase

if TYPE_CHECKING:
    import geometry_msgs.msg

    from task_generator.manager.robot_manager.robot_manager import RobotManager
    from task_generator.shared import Pose
    from task_generator.tasks.robots.adapters import ResetContext


@AdapterMeta.attach(
    accepts={TaskKind.GOTO_POSE},
    bringup=Nav2Bringup,
    client=GotoPoseClient,
    cap="mobile",
    displays=[
        AdapterDisplayHint(
            name="Local Costmap",
            topic="{ns}/local_costmap/costmap",
            topic_type="nav_msgs/OccupancyGrid",
            rviz_class="rviz_default_plugins/Map",
            config_json=('{"Color Scheme": "costmap", "Draw Behind": false, "Alpha": 0.7, "Topic": {"Depth": 20, "History Policy": "Keep Last", "Reliability Policy": "Reliable", "Durability Policy": "Transient Local"}}'),
        ),
        AdapterDisplayHint(
            name="Global Costmap",
            topic="{ns}/global_costmap/costmap",
            topic_type="nav_msgs/OccupancyGrid",
            rviz_class="rviz_default_plugins/Map",
            config_json=('{"Enabled": false, "Color Scheme": "costmap", "Draw Behind": false, "Alpha": 0.7, "Topic": {"Depth": 20, "History Policy": "Keep Last", "Reliability Policy": "Reliable", "Durability Policy": "Transient Local"}}'),
        ),
        AdapterDisplayHint(
            name="Local Plan",
            topic="{ns}/local_plan",
            topic_type="nav_msgs/Path",
            rviz_class="rviz_default_plugins/Path",
            config_json='{"Color": "255; 0; 0", "Line Width": 0.05}',
        ),
        AdapterDisplayHint(
            name="Robot Footprint",
            topic="{ns}/local_costmap/published_footprint",
            topic_type="geometry_msgs/PolygonStamped",
            rviz_class="rviz_default_plugins/Polygon",
            config_json='{"Alpha": 1.0}',
        ),
    ],
)
@requires_map_server
class Nav2Adapter(MobileAdapter):
    kind: ClassVar[str] = "nav2"

    async def publish_goal_loop(self) -> None:
        # nav2 uses navigate_to_pose action; no topic republish.
        return

    async def dispatch_phase(
        self,
        phase: TaskPhase,
        robot: RobotManager,
    ) -> None:
        assert isinstance(phase, GoToPhase), f"Nav2Adapter only accepts GOTO_POSE phases; got {type(phase).__name__} (kind={phase.kind!r})"
        robot._goal_pos = phase.pose  # pylint: disable=protected-access
        if self.client.is_done() is False:
            self.client.cancel()
        goal = GotoPose.Goal()
        goal.target = self._phase_to_pose_stamped(phase, robot)
        goal.pose_tolerance = float(phase.tolerance_radius or 0.0)
        goal.yaw_tolerance = float(phase.tolerance_angle or 0.0)
        await self.client.send_goal(goal)

    def _phase_to_pose_stamped(
        self,
        phase: GoToPhase,
        robot: RobotManager,
    ) -> geometry_msgs.msg.PoseStamped:
        import geometry_msgs.msg

        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = robot.node.sim_time.to_msg()
        msg.pose = phase.pose.to_msg()
        return msg

    async def wait_until_ready(
        self,
        robot: RobotManager,
        node_paths: set[str],
    ) -> None:
        # TMP: remove once rosnavrl decoupled from nav
        if robot.node.rosparam[bool].get("train_mode", False):
            await super().wait_until_ready(robot, node_paths)
            return
        bt_node_path = str(robot.namespace("bt_navigator"))
        while bt_node_path not in node_paths:
            await asyncio.sleep(0.01)
        await super().wait_until_ready(robot, node_paths)

    async def on_reset(self, robot: RobotManager, ctx: ResetContext) -> None:
        if self.client.is_done() is False:
            self.client.cancel()
        await super().on_reset(robot, ctx)
        await self._clear_local_costmap(robot)

    async def on_move(
        self,
        pose: Pose,
        robot: RobotManager,
    ) -> None:
        await self._clear_local_costmap(robot)

        request = robot._current_request
        if request is None or robot._phase_index >= len(request.phases):
            return
        await self.dispatch_phase(request.phases[robot._phase_index], robot)

    async def _clear_local_costmap(
        self,
        robot: RobotManager,
        reset_distance: float = -1.0,
    ) -> bool:
        node_name = robot.namespace("local_costmap/local_costmap")

        if reset_distance < 0:
            srv_name = os.path.abspath(node_name("../clear_entirely_local_costmap"))
            srv_type = ClearEntireCostmap
            req = ClearEntireCostmap.Request()
        else:
            srv_name = os.path.abspath(node_name("../clear_around_local_costmap"))
            srv_type = ClearCostmapAroundRobot
            req = ClearCostmapAroundRobot.Request()
            req.reset_distance = reset_distance

        state = await robot.node.get_lifecycle_state_async(node_name)
        if state.id != lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE:
            return False

        cli = robot.node.create_client_wrapper(srv_type, srv_name)
        await cli.ensure()

        result = await cli.call_timeout(req)
        if result is None:
            robot.node.get_logger().error(f"service call failed for {srv_name}")
            return False
        return True
