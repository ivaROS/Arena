"""MoveIt arm adapter — thin composer of MoveItArmBringup + ReachPoseClient."""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, ClassVar

from arena_robots.bringup.arm.moveit import MoveItArmBringup
from arena_robots.clients.play_gesture import PlayGestureClient
from arena_robots.clients.reach_pose import ReachPoseClient
from arena_robots.task_kinds import TaskKind
from arena_robots_msgs.action import PlayGesture, ReachPose

from task_generator.tasks.robots.adapters import Adapter, AdapterDisplayHint, AdapterMeta
from task_generator.tasks.robots.request import PlayGesturePhase, ReachPhase

if TYPE_CHECKING:
    from task_generator.manager.robot_manager.robot_manager import RobotManager
    from task_generator.tasks.robots.adapters import ResetContext
    from task_generator.tasks.robots.request import TaskPhase

_log = logging.getLogger(__name__)


@AdapterMeta.attach(
    accepts={TaskKind.REACH_POSE, TaskKind.PLAY_GESTURE},
    bringup=MoveItArmBringup,
    clients={TaskKind.REACH_POSE: ReachPoseClient, TaskKind.PLAY_GESTURE: PlayGestureClient},
    cap="arm",
    displays=(
        AdapterDisplayHint(
            name="Planned Trajectory",
            topic="{ns}/move_group/display_planned_path",
            topic_type="moveit_msgs/DisplayTrajectory",
            rviz_class="moveit_rviz_plugin/Trajectory",
            config_json='{"Robot Description": "{robot}.robot_description"}',
        ),
        AdapterDisplayHint(
            name="Planning Scene",
            topic="{ns}/monitored_planning_scene",
            topic_type="moveit_msgs/PlanningScene",
            rviz_class="moveit_rviz_plugin/PlanningScene",
            config_json='{"Robot Description": "{robot}.robot_description", "Enabled": false}',
        ),
    ),
)
class MoveItArmAdapter(Adapter):
    kind: ClassVar[str] = "moveit"

    async def dispatch_phase(self, phase: TaskPhase, robot: RobotManager) -> None:
        if isinstance(phase, ReachPhase):
            goal = ReachPose.Goal()
            if phase.random:
                from task_generator.tasks.robots._reach_sampling import sample_reach_target

                arms = robot.robot_view.caps.arm
                if not arms:
                    raise ValueError(f"random ReachPhase requested but robot {robot.name!r} has no arm cap")
                (arm,) = arms.values()
                rng = robot.node.conf.General.RNG.value
                goal.target = sample_reach_target(arm, robot.frame, rng)
            elif phase.target is not None:
                goal.target = phase.target
            elif phase.named_target is not None:
                goal.named_target = phase.named_target
            goal.position_tolerance = float(phase.position_tolerance or 0.0)
            goal.orientation_tolerance = float(phase.orientation_tolerance or 0.0)
            goal.planning_time = float(phase.planning_time or 0.0)
            await self.client_for(TaskKind.REACH_POSE).send_goal(goal)
        elif isinstance(phase, PlayGesturePhase):
            gesture_name = phase.gesture
            if gesture_name is None:
                gesture_name = _pick_random_gesture(robot)
            goal = PlayGesture.Goal(gesture=gesture_name or "")
            await self.client_for(TaskKind.PLAY_GESTURE).send_goal(goal)
        else:
            raise TypeError(f"MoveItArmAdapter: unsupported phase type {type(phase).__name__} (kind={phase.kind!r})")

    async def on_reset(self, robot: RobotManager, ctx: ResetContext) -> None:
        # MoveIt cannot plan while sim is paused (no /joint_states, no current state),
        # the TM is responsible for emitting a stow phase if it wants the arm parked.
        del robot, ctx

    async def wait_until_ready(self, robot: RobotManager, node_paths: set[str]) -> None:
        mg = str(robot.namespace("move_group"))
        while mg not in node_paths:
            await asyncio.sleep(0.01)
        await super().wait_until_ready(robot, node_paths)


def _pick_random_gesture(robot: RobotManager) -> str:
    """Return a gesture name supported by this robot's arm, or empty string if none."""
    import yaml
    from arena_simulation_setup import ASS_DIR
    from arena_simulation_setup.tree.Gesture import GestureSpec

    arms = robot.robot_view.caps.arm
    if not arms:
        _log.warning("random PlayGesturePhase: robot %r has no arm cap", robot.name)
        return ""
    (arm,) = arms.values()
    available_poses = set(arm.named_poses.keys())

    # shared library: glob ASS_DIR/configs/gestures/*.yaml
    gestures: dict[str, GestureSpec] = {}
    gestures_dir = ASS_DIR / "configs" / "gestures"
    for path in sorted(gestures_dir.glob("*.yaml")):
        with open(path) as f:
            data = yaml.safe_load(f)
        gestures[path.stem] = GestureSpec.parse(data)

    # per-arm overrides shadow shared library
    gestures.update(arm.gestures)

    supported = [name for name, spec in gestures.items() if available_poses >= spec.required_poses()]
    if not supported:
        _log.warning(
            "random PlayGesturePhase: robot %r has no supported gestures (named_poses: %s)",
            robot.name,
            sorted(available_poses),
        )
        return ""
    rng = robot.node.conf.General.RNG.value
    return str(rng.choice(supported))
