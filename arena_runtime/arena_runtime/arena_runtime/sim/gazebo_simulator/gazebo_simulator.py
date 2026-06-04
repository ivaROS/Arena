"""
GazeboHost is constructed once by arena_node and owns process-singleton resources for Gazebo: the gz_services_bridge launch, the lifecycle (pause/unpause), and the control-world client.
GazeboSimulator is per-env on task_generator_node and adapts env-namespace state (spawned-name tracking, per-env service clients) over those shared resources. Sim_path prefixing is owned by the Realizer.
"""

import asyncio
import itertools
import math
import time
import traceback
import typing
from collections.abc import Iterable, Sequence

import arena_robots.Robot
import launch
import launch_ros
import rclpy.impl.rcutils_logger
import rclpy.time
import tf2_ros
from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_rclpy_mixins import ArenaMixinNode
from arena_rclpy_mixins.Async import ClientWrapper
from arena_rclpy_mixins.shared import Namespace
from geometry_msgs.msg import PoseWithCovarianceStamped
from launch import LaunchDescription
from ros_gz_interfaces.msg import Entity as EntityMsg
from ros_gz_interfaces.msg import EntityFactory, WorldControl
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity, SetEntityPose, SpawnEntity
from task_generator.shared import (
    DynamicObstacle,
    Entity,
    Floor,
    ModelType,
    Obstacle,
    Orientation,
    Pose,
    Position,
    Robot,
    Wall,
)

from arena_runtime.sim import BaseSim, SimLifecycle
from arena_runtime.sim._control import (
    controller_spawner_node,
    odom_relay_node,
    render_ros2_control_yaml,
    twist_stamper_node,
)

from .robot_bridge import BridgeConfiguration


class GazeboHost(SimLifecycle):
    def __init__(
        self,
        node: ArenaMixinNode,
        semaphore: asyncio.Semaphore,
        service_control_world: ClientWrapper,
        service_delete_entity: ClientWrapper,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self._node = node
        self._semaphore = semaphore
        self._service_control_world = service_control_world
        self._service_delete_entity = service_delete_entity
        self._logger = logger

    async def ensure_ready(self) -> None:
        await self._node.do_launch(
            LaunchDescription(
                [
                    launch_ros.actions.Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        name="gz_services_bridge",
                        output="screen",
                        arguments=[
                            "/world/default/create@ros_gz_interfaces/srv/SpawnEntity",
                            "/world/default/remove@ros_gz_interfaces/srv/DeleteEntity",
                            "/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                            "/world/default/control@ros_gz_interfaces/srv/ControlWorld",
                        ],
                        parameters=[{"use_sim_time": True}],
                    )
                ]
            )
        )
        await asyncio.gather(
            self._service_control_world.ensure(),
            self._service_delete_entity.ensure(),
        )

    async def pause(self) -> bool:
        async with self._semaphore:
            self._logger.debug("Attempting to pause simulation")
            request = ControlWorld.Request()
            request.world_control = WorldControl()
            request.world_control.pause = True
            try:
                result = await self._service_control_world.call_timeout(request)
                if result is None:
                    self._logger.error("Pause service call failed")
                    return False
                self._logger.debug(f"Pause result: {result.success}")
                return result.success
            except Exception as e:
                self._logger.error(f"Error pausing simulation: {str(e)}")
                traceback.print_exc()
                return False

    async def unpause(self) -> bool:
        async with self._semaphore:
            self._logger.debug("Attempting to unpause simulation")
            request = ControlWorld.Request()
            request.world_control = WorldControl()
            request.world_control.pause = False
            try:
                result = await self._service_control_world.call_timeout(request)
                if result is None:
                    self._logger.error("Unpause service call failed")
                    return False
                self._logger.debug(f"Unpause result: {result.success}")
                return result.success
            except Exception as e:
                self._logger.error(f"Error unpausing simulation: {str(e)}")
                traceback.print_exc()
                return False

    async def cleanup_namespace(self, prefix: str) -> int:
        names = await self._list_models()
        targets = [n for n in names if n.startswith(prefix)]
        if not targets:
            return 0

        async def _del(name: str) -> bool:
            req = DeleteEntity.Request()
            req.entity = EntityMsg(name=name, type=EntityMsg.MODEL)
            async with self._semaphore:
                try:
                    res = await self._service_delete_entity.call_timeout(req)
                except Exception as e:
                    self._logger.warning(f"cleanup_namespace: delete {name} raised: {e!r}")
                    return False
            return bool(res) and res.success

        results = await asyncio.gather(*(_del(n) for n in targets))
        return sum(1 for r in results if r)

    async def _list_models(self) -> list[str]:
        proc = await asyncio.create_subprocess_exec(
            'gz',
            'model',
            '--list',
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await proc.communicate()
        if proc.returncode != 0:
            self._logger.warning(f"gz model --list failed: {stderr.decode().strip()}")
            return []
        names: list[str] = []
        for line in stdout.decode().splitlines():
            stripped = line.strip()
            if stripped.startswith('- '):
                names.append(stripped[2:].strip())
        return names


class GazeboSimulator(BaseSim):
    def __init__(self, *args: object, namespace: Namespace, **kwargs: object) -> None:
        super().__init__(*args, namespace=namespace, **kwargs)

        self._semaphore = asyncio.Semaphore(5)

        self._logger.info(f"Initializing GazeboSimulator with namespace: {namespace}")

        self.entities: dict[str, Entity] = {}
        self._walls_entities: list[str] = []
        self._wall_counter = itertools.count()
        self._spawned_names: set[str] = set()

        self._agent_robots: dict[str, str] = {}
        self._mechanism_tf_buffer = tf2_ros.Buffer()
        self._mechanism_tf_listener = tf2_ros.TransformListener(self._mechanism_tf_buffer, self.node)

    async def before_reset_episode(self) -> bool:
        return True

    async def after_reset_episode(self) -> bool:
        return True

    def _robot_loader_args(self, robot: Robot) -> dict[str, object]:
        args: dict[str, object] = {
            **robot.asdict(),
            'sim_path': robot.sim_path,
            'optim': self.node.rosparam[str].get('optim', ''),
        }
        robot_config = arena_robots.Robot.RobotIdentifier(robot.model.name).resolve_sync()
        control_spec = robot_config.model_params.control
        if control_spec is None or not control_spec.is_ros2_control:
            return args
        args['namespace'] = str(self.node.service_namespace(robot.name))
        if control_spec.config is not None:
            args['gazebo_controllers'] = self._render_ros2_control_yaml(
                robot,
                control_spec.config,
                frame_prefix=robot.frame.tf(),
            )
        return args

    def _render_ros2_control_yaml(self, robot: Robot, config_uri: str, *, frame_prefix: str) -> str:
        return render_ros2_control_yaml(config_uri, robot.sim_path, frame_prefix)

    async def obstacle_spawn(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        # Pre-resolve models to allow batching of net fetch requests
        async def _resolve(obs: Obstacle):
            try:
                await obs.model.resolve()
            except Exception:
                # ignore errors here, they will be caught and logged during actual spawn
                pass

        await asyncio.gather(*(_resolve(o) for o in obstacles))
        return await asyncio.gather(*map(self._spawn_entity, obstacles))

    async def pedestrian_spawn(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        # Pre-resolve models to allow batching of net fetch requests
        async def _resolve(obs: Obstacle):
            try:
                await obs.model.resolve()
            except Exception:
                # ignore errors here, they will be caught and logged during actual spawn
                pass

        await asyncio.gather(*(_resolve(p) for p in pedestrians))
        return await asyncio.gather(*map(self._spawn_entity, pedestrians))

    async def robot_spawn(self, robots: Sequence[Robot]) -> Sequence[bool]:

        async def impl(robot: Robot) -> bool:
            if not await self._spawn_entity(robot):
                return False
            _loader_args = self._robot_loader_args(robot)
            model = await (await robot.model.resolve()).model.get(ModelType.URDF, loader_args=_loader_args)
            if model.type is ModelType.UNKNOWN:
                return False
            model_description = model.description
            self._robot_initialpose(robot)
            await self._robot_bridge(robot, model_description)
            robot_config = arena_robots.Robot.RobotIdentifier(robot.model.name).resolve_sync()
            self._agent_robots[robot.sim_path] = robot.frame.tf(robot_config.model_params.base_frame)
            return True

        success = await asyncio.gather(*map(impl, robots))
        return success

    async def obstacle_move(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        return await asyncio.gather(*map(self._move_entity, obstacles))

    async def pedestrian_move(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        return await asyncio.gather(*map(self._move_entity, pedestrians))

    async def robot_move(self, robots: Sequence[Robot]) -> Sequence[bool]:
        async def impl(robot: Robot) -> bool:
            return (await self._move_entity(robot)) and (await self._robot_move(robot))

        async with self.node.unpause_window():
            result = await asyncio.gather(*map(impl, robots))
        return result

    async def obstacle_delete(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        return await asyncio.gather(*(self._delete_entity(o.sim_path) for o in obstacles))

    async def pedestrian_delete(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        return await asyncio.gather(*(self._delete_entity(p.sim_path) for p in pedestrians))

    async def robot_delete(self, robots: Sequence[Robot]) -> Sequence[bool]:
        for robot in robots:
            self._agent_robots.pop(robot.sim_path, None)
        return await asyncio.gather(*(self._delete_entity(robot.sim_path) for robot in robots))

    async def pedestrian_update(self, pedestrians: Pedestrians) -> Sequence[bool]:
        async def impl(ped: Pedestrian) -> bool:
            req = SetEntityPose.Request()
            req.entity = EntityMsg(name=ped.name, type=EntityMsg.MODEL)
            req.pose = ped.pose
            res = await self._service_set_entity_pose.call_timeout(req)
            return bool(res and res.success)

        return await asyncio.gather(*(impl(p) for p in pedestrians.pedestrians))

    async def spawn_floors(self, floors: Sequence[Floor]) -> bool:
        # Gazebo does not support spawning floors
        del floors
        return True

    def robot_positions_xy(self) -> Iterable[tuple[str, tuple[float, float]]]:
        out: list[tuple[str, tuple[float, float]]] = []
        for sim_path, frame in list(self._agent_robots.items()):
            try:
                t = self._mechanism_tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            out.append((sim_path, (t.transform.translation.x, t.transform.translation.y)))
        return out

    def robot_pose(self, sim_path: str) -> Pose | None:
        frame = self._agent_robots.get(sim_path)
        if frame is None:
            return None
        try:
            t = self._mechanism_tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        rot = t.transform.rotation
        return Pose(
            position=Position(x=tr.x, y=tr.y, z=tr.z),
            orientation=Orientation(w=rot.w, x=rot.x, y=rot.y, z=rot.z),
        )

    async def set_robot_pose(self, sim_path: str, pose: Pose) -> bool:
        if sim_path not in self._agent_robots:
            return False
        req = SetEntityPose.Request()
        req.entity = EntityMsg(name=sim_path, type=EntityMsg.MODEL)
        req.pose = pose.to_msg()
        try:
            res = await self._service_set_entity_pose.call_timeout(req)
            return bool(res and res.success)
        except Exception as e:
            self._logger.warning(f"set_robot_pose({sim_path!r}) failed: {e}")
            return False

    async def spawn_box(self, name: str, size: tuple[float, float, float], pose: Pose) -> bool:
        sdf = _generate_box_sdf(name, size)
        async with self._semaphore:
            return await self._spawn_sdf(name, sdf, pose)

    async def move_box(self, name: str, pose: Pose) -> bool:
        # Fire-and-forget: animation is cosmetic, last-write-wins. Awaiting the service round-trip
        # would gate the shim's tick rate on bridge latency; skipping the semaphore lets parallel
        # door updates issue concurrently.
        request = SetEntityPose.Request()
        request.entity = EntityMsg(name=name, type=EntityMsg.MODEL)
        request.pose = pose.to_msg()
        try:
            self._service_set_entity_pose.client.call_async(request)
        except Exception as e:
            self._logger.warning(f"move_box dispatch failed for {name}: {e}")
            return False
        return True

    async def delete_box(self, name: str) -> bool:
        return await self._delete_entity(name)

    # IMPL

    async def _move_entity(self, entity: Entity, entity_type: int = EntityMsg.MODEL) -> bool:
        async with self._semaphore:
            self._logger.debug(f"Attempting to move entity: {entity.sim_path}")
            self._logger.debug(f"Moving entity {entity.sim_path} to position: {entity.pose}")

            request = SetEntityPose.Request()
            request.entity = EntityMsg(
                name=entity.sim_path,
                type=entity_type,
            )
            request.pose = entity.pose.to_msg()

            try:
                await self._service_set_entity_pose.ensure()
                result = await self._service_set_entity_pose.call_timeout(request)

                if result is None:
                    self._logger.error(f"Move service call failed for {entity.sim_path}")
                    return False

                self._logger.debug(f"Move result for {entity.sim_path}: {result.success}")

                return result.success

            except Exception as e:
                self._logger.error(f"Error moving entity {entity.sim_path}: {str(e)}")
                traceback.print_exc()
                return False

    async def _spawn_entity(self, entity: Entity) -> bool:
        async with self._semaphore:
            try:
                # Get model description
                try:
                    if isinstance(entity, Robot):
                        _loader_args = self._robot_loader_args(entity)
                        model = await (await entity.model.resolve()).model.get(ModelType.URDF, loader_args=_loader_args)
                    else:
                        model = await (await entity.model.resolve()).model.get(ModelType.SDF)
                except Exception:
                    self._logger.warning(f"Failed to resolve model for entity {entity.name}")
                    self._logger.debug(traceback.format_exc())
                    return False

                if model.type is ModelType.UNKNOWN:
                    self._logger.warning(f"Failed to resolve model for entity {entity.name}: unknown model type {model}")
                    return False

                if model.path and model.type not in (ModelType.URDF,):
                    # direct path available, use ros service call
                    sdf_path = model.path
                    # Resolve directory to actual SDF file (Gazebo requires a file, not a directory)
                    if sdf_path.is_dir():
                        candidate = sdf_path / f"{sdf_path.name}.sdf"
                        if candidate.is_file():
                            sdf_path = candidate
                        else:
                            # Also check for nested SDF file if directory is named ModelName.sdf
                            candidate = sdf_path / f"{sdf_path.name}.sdf" / f"{sdf_path.name}.sdf"
                            if candidate.is_file():
                                sdf_path = candidate
                            else:
                                candidates = list(sdf_path.glob("**/*.sdf"))
                                if candidates:
                                    sdf_path = candidates[0]
                                else:
                                    self._logger.error(f"Failed to find .sdf file in directory {sdf_path}")
                                    return False

                    ok = await self._spawn_sdf_file(entity.sim_path, str(sdf_path), entity.pose)
                    if ok:
                        self.entities[entity.name] = entity
                    return ok

                else:
                    ok = await self._spawn_sdf(entity.sim_path, model.description, entity.pose)
                    if ok:
                        self.entities[entity.name] = entity
                    return ok

            except Exception as e:
                self._logger.error(f"Error spawning entity {entity.name}: {str(e)}")
                traceback.print_exc()
                return False

    async def _spawn_sdf(self, name: str, sdf: str, pose: Pose) -> bool:
        """Spawn from an in-memory SDF via ros_gz_bridge. Caller holds self._semaphore."""
        request = SpawnEntity.Request()
        request.entity_factory = EntityFactory()
        request.entity_factory.name = name
        request.entity_factory.sdf = sdf
        request.entity_factory.pose = pose.to_msg()

        try:
            result = await self._service_spawn_entity.call_timeout(request)
        except Exception as e:
            self._logger.error(f"Spawn service call raised for {name}: {e}")
            return False

        if result is None:
            self._logger.error(f"Spawn service call failed for {name}")
            return False

        if result.success:
            self._spawned_names.add(name)
        return result.success

    async def _spawn_sdf_file(self, name: str, sdf_filename: str, pose: Pose) -> bool:
        """Spawn from an SDF file via ros_gz_bridge. Caller holds self._semaphore."""
        request = SpawnEntity.Request()
        request.entity_factory = EntityFactory()
        request.entity_factory.name = name
        request.entity_factory.sdf_filename = sdf_filename
        request.entity_factory.pose = pose.to_msg()

        try:
            result = await self._service_spawn_entity.call_timeout(request)
        except Exception as e:
            self._logger.error(f"Spawn service call raised for {name}: {e}")
            return False

        if result is None:
            self._logger.error(f"Spawn service call failed for {name}")
            return False

        if result.success:
            self._spawned_names.add(name)
        return result.success

    async def _delete_entity(self, sim_path: str, entity_type: int = EntityMsg.MODEL) -> bool:
        async with self._semaphore:
            self._logger.debug(f"Attempting to delete entity: {sim_path}")

            request = DeleteEntity.Request()
            request.entity = EntityMsg(
                name=sim_path,
                type=entity_type,
            )

            try:
                result = await self._service_delete_entity.call_timeout(request)

                if result is None:
                    self._logger.error(f"Delete service call failed for {sim_path}")
                    return False

                self._logger.debug(f"Delete result for {sim_path}: {result.success}")

                if result.success:
                    self.entities.pop(sim_path, None)
                    self._spawned_names.discard(sim_path)

                return result.success

            except Exception as e:
                self._logger.error(f"Error deleting entity {sim_path}: {str(e)}")
                traceback.print_exc()
                return False

    async def step(self, n: int = 1) -> bool:
        async with self._semaphore:
            request = ControlWorld.Request()
            request.world_control = WorldControl()
            request.world_control.multi_step = n
            try:
                result = await self._service_control_world.call_timeout(request)
                if result is None:
                    self._logger.error("Step service call failed")
                    return False
                return result.success
            except Exception as e:
                self._logger.error(f"Error stepping simulation: {str(e)}")
                traceback.print_exc()
                return False

    async def spawn_walls(self, walls: Sequence[Wall], clear_existing: bool = True) -> bool:
        if clear_existing:
            await self.remove_world()
        for wall in walls:
            wall_name = self._realizer.realize(f"wall_{next(self._wall_counter)}")
            wall_sdf = _generate_wall_sdf(
                name=wall_name,
                walls=[wall],
                height=2.0,
                thickness=0.05,
                base_position=(0, 0, 0),
            )
            if not wall_sdf:
                self._logger.error(f"Failed to generate SDF for wall: {wall_name}")
                continue
            async with self._semaphore:
                await self._spawn_sdf(wall_name, wall_sdf, Pose())
            self._walls_entities.append(wall_name)
        return True

    async def remove_world(self) -> bool:
        for entity in self._walls_entities:
            await self._delete_entity(entity)
        self._walls_entities = []
        self._wall_counter = itertools.count()
        return True

    async def _robot_bridge(self, robot: Robot, description: str):
        launch_description = launch.LaunchDescription()

        launch_description.add_action(launch_ros.actions.PushRosNamespace(namespace=self.node.service_namespace(robot.name)))

        robot_config = arena_robots.Robot.RobotIdentifier(robot.model.name).resolve_sync()

        mappings = BridgeConfiguration.from_file(robot_config.mappings).substitute(
            {
                "robot_name": robot.sim_path,
                "world": "/world/default",
            }
        )

        bridge_arguments = mappings.as_args()
        remappings = mappings.as_remappings()

        # Add parameter_bridge node
        launch_description.add_action(
            launch_ros.actions.Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=bridge_arguments,
                remappings=remappings,
                parameters=[{"use_sim_time": True}],
            )
        )
        launch_description.add_action(
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"robot_description": description},
                    {"frame_prefix": robot.frame.tf()},
                ],
            )
        )

        launch_description.add_action(
            launch_ros.actions.Node(
                package="pose_to_tf",
                executable="pose_to_tf",
                name="pose_to_tf",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "parent_frame": robot.frame.tf(robot_config.model_params.odom_frame),
                        "child_frame": robot.frame.tf(robot_config.model_params.base_frame),
                        "pose_topic": "pose",
                    }
                ],
            )
        )

        control_spec = robot_config.model_params.control
        if control_spec is not None and control_spec.is_ros2_control:
            if not control_spec.controllers:
                raise ValueError(f"control.mode=ros2_control but no controllers declared for {robot.name}")
            for controller_name in control_spec.controllers:
                launch_description.add_action(controller_spawner_node(controller_name))
            launch_description.add_action(
                twist_stamper_node(
                    control_spec.cmd_vel_topic,
                    robot.frame.tf(robot_config.model_params.base_frame),
                )
            )
            if control_spec.odom_topic != "odom":
                launch_description.add_action(odom_relay_node(control_spec.odom_topic))

        launch_description.add_action(
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "map",
                    robot.frame.tf(robot_config.model_params.odom_frame),
                ],
                parameters=[{"use_sim_time": True}],
            )
        )

        # launch_description.add_action(
        #     launch_ros.actions.Node(
        #         package='joint_state_publisher',
        #         executable='joint_state_publisher',
        #         output='screen',
        #         parameters=[
        #             {'use_sim_time': True},
        #             {'robot_description': description},  # Ensure URDF is passed here too
        #         ],
        #         remappings=[('/joint_states', '/joint_states')]
        #     )
        # )
        await self.node.do_launch(launch_description)

    def _robot_initialpose(self, robot: Robot):
        pose = PoseWithCovarianceStamped()
        pose.pose.pose = robot.pose.to_msg()
        pose.header.frame_id = "map"

        self.node.create_publisher(
            PoseWithCovarianceStamped,
            self.node.service_namespace(robot.name, "initialpose"),
            qos_profile=1,
        ).publish(pose)

    async def _robot_move(self, robot: Robot) -> bool:
        name = robot.name
        try:
            self._robot_initialpose(robot)

            max_attempts = 3
            attempt = 1
            initial_pose_triggered = False

            while attempt <= max_attempts and not initial_pose_triggered:
                self._logger.debug(f"Attempt {attempt}/{max_attempts}: Triggering initial pose update for robot {name}")
                try:
                    self._robot_initialpose(robot)
                    initial_pose_triggered = True
                    self._logger.debug(f"Initial pose update for {name} succeeded on attempt {attempt}")
                except Exception as e:
                    self._logger.error(f"Attempt {attempt}/{max_attempts} failed for {name}: {str(e)}")
                    traceback.print_exc()
                    if attempt < max_attempts:
                        self._logger.debug("Waiting 1 second before retrying...")
                        time.sleep(1)
                    attempt += 1

            if not initial_pose_triggered:
                self._logger.error(f"Failed to set initial pose for {name} after {max_attempts} attempts")

            return True

        except Exception as e:
            self._logger.error(f"Error moving robot {name}: {str(e)}")
            return False

    async def _set_up_services(self):
        futures: list[typing.Awaitable] = []

        # Initialize service clients
        # https://gazebosim.org/api/sim/8/entity_creation.html
        self._service_spawn_entity = self.node.create_client_wrapper(
            SpawnEntity,
            "/world/default/create",
        )
        self._service_delete_entity = self.node.create_client_wrapper(
            DeleteEntity,
            "/world/default/remove",
        )
        self._service_set_entity_pose = self.node.create_client_wrapper(
            SetEntityPose,
            "/world/default/set_pose",
        )
        self._service_control_world = self.node.create_client_wrapper(
            ControlWorld,
            "/world/default/control",
        )
        self._logger.info("Waiting for gazebo services...")
        services = (
            (self._service_spawn_entity, "spawn entity"),
            (self._service_delete_entity, "delete entity"),
            (self._service_set_entity_pose, "set entity pose"),
            (self._service_control_world, "control world"),
        )

        for service, name in services:
            self._logger.debug(f"Waiting for {name} service...")
            futures.append(service.ensure())

        await asyncio.gather(*futures)
        self._logger.info("All Gazebo services are available now.")

    async def shutdown(self) -> None:
        await self.stop_mechanisms()

        async def _delete_one(name: str) -> None:
            async with self._semaphore:
                req = DeleteEntity.Request()
                req.entity = EntityMsg(name=name, type=EntityMsg.MODEL)
                await self._service_delete_entity.call_timeout(req)

        names = list(self._spawned_names)
        await asyncio.gather(*(_delete_one(n) for n in names), return_exceptions=True)
        self._spawned_names.clear()

    @classmethod
    async def create(cls, *args: object, namespace: Namespace, **kwargs: object) -> "GazeboSimulator":
        simulator = cls(*args, namespace=namespace, **kwargs)
        await simulator._set_up_services()
        return simulator


def _generate_wall_sdf(
    name: str,
    walls: list[Wall],
    height: float,
    thickness: float,
    base_position: tuple[float, float, float] = (0, 0, 0),
) -> str:
    """
    Generate an SDF string for a wall structure based on given parameters and base position.
    """
    sdf_template = """
        <sdf version="1.6">
            <model name="{name}">
                <pose>{base_x} {base_y} {base_z} 0 0 0</pose>
                {links}
                <static>true</static>
            </model>
        </sdf>
        """
    link_template = """
        <link name="wall_segment_{index}">
            <visual name="visual">
                <geometry>
                    <box>
                        <size>{length} {thickness} {height}</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.7 0.7 0.7 1</ambient>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                    <box>
                        <size>{length} {thickness} {height}</size>
                    </box>
                </geometry>
            </collision>
            <pose>{x} {y} {z} 0 0 {orientation}</pose>
        </link>
        """
    links = []
    base_x, base_y, base_z = base_position
    z = height / 2.0  # Center the wall height relative to the base

    for i, w in enumerate(walls):
        x1, y1, x2, y2 = w.start.x, w.start.y, w.end.x, w.end.y
        length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        orientation = math.atan2(y2 - y1, x2 - x1)
        x = (x1 + x2) / 2 + base_x
        y = (y1 + y2) / 2 + base_y

        links.append(
            link_template.format(
                index=i,
                length=length,
                thickness=thickness,
                height=height,
                x=x,
                y=y,
                z=z + base_z,
                orientation=orientation,
            )
        )

    return sdf_template.format(name=name, base_x=base_x, base_y=base_y, base_z=base_z, links="\n".join(links))


_BOX_SDF_TEMPLATE = """
<sdf version="1.6">
    <model name="{name}">
        <static>true</static>
        <link name="link">
            <visual name="visual">
                <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
            </visual>
            <collision name="collision">
                <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
            </collision>
        </link>
    </model>
</sdf>
"""


def _generate_box_sdf(name: str, size: tuple[float, float, float]) -> str:
    sx, sy, sz = size
    return _BOX_SDF_TEMPLATE.format(name=name, sx=sx, sy=sy, sz=sz)
