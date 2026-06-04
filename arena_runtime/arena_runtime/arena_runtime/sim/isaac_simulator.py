import asyncio
import itertools
import math
import os
import random
import traceback
import types
import typing
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Sequence

import arena_people_msgs.msg
import arena_robots.Robot
import arena_simulation_setup.tree.assets.Material
import geometry_msgs.msg
import isaacsim_msgs.msg
import launch
import launch_ros
import rclpy.time
import std_msgs.msg
import std_srvs.srv
import tf2_ros
from arena_people_msgs.msg import Pedestrian, SpawnPedestrian
from arena_people_msgs.srv import (
    DeletePedestrians,
    MovePedestrians,
    SpawnPedestrians,
    UpdatePedestrians,
)
from arena_rclpy_mixins import ArenaMixinNode
from arena_rclpy_mixins.Async import ClientWrapper
from arena_rclpy_mixins.shared import Namespace
from arena_simulation_setup.shared import Obstacle as ObstacleDefinition
from arena_simulation_setup.tree.Wall import WallSegment
from isaacsim_msgs.msg import (
    Floor,
    Material,
    Prim,
    Wall,
)
from isaacsim_msgs.srv import (
    DeletePrims,
    EditPrims,
    ResetWorld,
    SpawnFloors,
    SpawnPrims,
    SpawnUrdf,
    SpawnUsd,
    SpawnWalls,
)
from task_generator.shared import (
    DynamicObstacle,
    ModelType,
    Obstacle,
    Orientation,
    Pose,
    Position,
    Robot,
)
from task_generator.shared import Floor as FloorDefinition
from task_generator.shared import Wall as WallDefinition

from arena_runtime._node import NodeInterface
from arena_runtime.sim import BaseSim, SimLifecycle
from arena_runtime.sim._control import (
    controller_spawner_node,
    odom_relay_node,
    render_ros2_control_yaml,
    twist_stamper_node,
)

"""
IsaacHost is constructed once by arena_node and owns process-singleton resources for Isaac: the lifecycle (pause/unpause/cleanup) and the pause/unpause/delete service clients.
IsaacSimulator is per-env on task_generator_node and adapts env-namespace state (per-robot publishers, env-prefixed entity names) over those shared resources.
"""


def _transform_urdf_for_bridge(
    urdf_path: str,
    commands_topics: dict[str, str],
    states_topic: str,
    out_path: str,
) -> None:
    """Rewrite the URDF for the Isaac bridge.

    - Swap gz_ros2_control/GazeboSimSystem to JointStateTopicSystem.
    - Split each block into per-command-interface-kind blocks so JointStateTopicSystem
      emits well-formed JointState messages (one kind per topic). Each kind goes to
      its own commands_topics[kind].
    - Add <state_interface name="effort"/> to every joint, because JointStateTopicSystem
      unconditionally writes incoming effort and would otherwise crash.
    """
    bridge_plugin = 'joint_state_topic_hardware_interface/JointStateTopicSystem'
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for plugin_el in root.iter('plugin'):
        if plugin_el.text and plugin_el.text.strip() == 'gz_ros2_control/GazeboSimSystem':
            plugin_el.text = bridge_plugin

    originals = [rc for rc in root.iter('ros2_control') if (h := rc.find('hardware')) is not None and (p := h.find('plugin')) is not None and p.text and p.text.strip() == bridge_plugin]

    for rc in originals:
        parent = next(p for p in root.iter() if rc in list(p))
        name = rc.get('name', 'bridge')
        rc_type = rc.get('type', 'system')

        by_kind: dict[str, list[ET.Element]] = {}
        for joint in rc.findall('joint'):
            cmd = joint.find('command_interface')
            if cmd is None:
                continue
            kind = cmd.get('name')
            if kind not in commands_topics:
                continue
            by_kind.setdefault(kind, []).append(joint)

        rc_idx = list(parent).index(rc)
        parent.remove(rc)

        for offset, (kind, joints) in enumerate(by_kind.items()):
            block = ET.Element('ros2_control', {'name': f'{name}_{kind}', 'type': rc_type})
            hw = ET.SubElement(block, 'hardware')
            ET.SubElement(hw, 'plugin').text = bridge_plugin
            ET.SubElement(hw, 'param', {'name': 'joint_commands_topic'}).text = commands_topics[kind]
            ET.SubElement(hw, 'param', {'name': 'joint_states_topic'}).text = states_topic
            # No anti-spam gating on a sim bridge: publish every CM tick so Isaac sees
            # the very first non-zero command without waiting for state to diverge.
            ET.SubElement(hw, 'param', {'name': 'trigger_joint_command_threshold'}).text = '0.0'
            for joint in joints:
                if not any(si.get('name') == 'effort' for si in joint.findall('state_interface')):
                    ET.SubElement(joint, 'state_interface', {'name': 'effort'})
                block.append(joint)
            parent.insert(rc_idx + offset, block)

    tree.write(out_path, encoding='utf-8', xml_declaration=True)


class IsaacHost(SimLifecycle):
    def __init__(self, node: ArenaMixinNode) -> None:
        self._logger = node.get_logger().get_child(type(self).__name__)
        self._pause_client: ClientWrapper = node.create_client_wrapper(
            std_srvs.srv.Trigger,
            "/isaac/PauseSimulation",
        )
        self._unpause_client: ClientWrapper = node.create_client_wrapper(
            std_srvs.srv.Trigger,
            "/isaac/UnpauseSimulation",
        )
        self._delete_prims_client: ClientWrapper = node.create_client_wrapper(
            DeletePrims,
            "/isaac/DeletePrims",
        )

    async def ensure_ready(self) -> None:
        await asyncio.gather(
            self._pause_client.ensure(),
            self._unpause_client.ensure(),
            self._delete_prims_client.ensure(),
        )

    async def pause(self) -> bool:
        res = await self._pause_client.call_timeout(std_srvs.srv.Trigger.Request())
        return bool(res) and res.success

    async def unpause(self) -> bool:
        res = await self._unpause_client.call_timeout(std_srvs.srv.Trigger.Request())
        return bool(res) and res.success

    async def cleanup_namespace(self, prefix: str) -> int:
        res = await self._delete_prims_client.call_timeout(DeletePrims.Request(names=[prefix]))
        if res is None or not res.ret:
            return 0
        return 1 if res.ret[0] else 0


def material_to_msg(material: arena_simulation_setup.tree.assets.Material.Material) -> isaacsim_msgs.msg.Material:
    return Material(
        name=material.name,
        path=material.path,
    )


class IsaacSimulator(BaseSim, NodeInterface):
    def __init__(self, *args: object, **kwargs: object) -> None:
        """Initialize IsaacSimulator"""
        super().__init__(*args, **kwargs)

        env_prefix = f"env_{self._env_id}"
        self._NS_PRIM = Namespace(env_prefix)('Obstacles')
        self._NS_PEDESTRIAN = Namespace(env_prefix)('Pedestrians')
        self._NS_ROBOT = Namespace(env_prefix)('Robots')
        self._NS_WALL = Namespace(env_prefix)('Walls')
        self._NS_FLOOR = Namespace(env_prefix)('Floors')

        self.wall_counter = itertools.count()
        self.floor_counter = itertools.count()
        self._clients = types.SimpleNamespace(
            DeletePedestrians=self.node.create_client_wrapper(DeletePedestrians, "/isaac/DeletePedestrians"),
            DeletePrims=self.node.create_client_wrapper(DeletePrims, "/isaac/DeletePrims"),
            EditPrims=self.node.create_client_wrapper(EditPrims, "/isaac/EditPrims"),
            MovePedestrians=self.node.create_client_wrapper(MovePedestrians, "/isaac/MovePedestrians"),
            ResetWorld=self.node.create_client_wrapper(ResetWorld, "/isaac/ResetWorld"),
            UpdatePedestrians=self.node.create_client_wrapper(UpdatePedestrians, "/isaac/UpdatePedestrians"),
            SpawnFloors=self.node.create_client_wrapper(SpawnFloors, "/isaac/SpawnFloors"),
            SpawnPedestrians=self.node.create_client_wrapper(SpawnPedestrians, "/isaac/SpawnPedestrians"),
            SpawnPrims=self.node.create_client_wrapper(SpawnPrims, "/isaac/SpawnPrims"),
            SpawnUrdf=self.node.create_client_wrapper(SpawnUrdf, "/isaac/SpawnUrdf"),
            SpawnUsd=self.node.create_client_wrapper(SpawnUsd, "/isaac/SpawnUsd"),
            SpawnWalls=self.node.create_client_wrapper(SpawnWalls, "/isaac/SpawnWalls"),
        )

        # sim_path -> (tf_frame, prim_name)
        self._agent_robots: dict[str, tuple[str, str]] = {}
        self._mechanism_tf_buffer = tf2_ros.Buffer()
        self._mechanism_tf_listener = tf2_ros.TransformListener(self._mechanism_tf_buffer, self.node)

    def _robot_loader_args(self, robot: Robot) -> dict[str, object]:
        return {
            **robot.asdict(),
            'optim': self.node.rosparam[str].get('optim', ''),
        }

    async def robot_spawn(self, robots: Sequence[Robot]) -> Sequence[bool]:
        async def impl(robot: Robot) -> bool:
            try:
                model = await (await robot.model.resolve()).model.get(
                    (
                        ModelType.URDF,
                        # ModelType.USD
                    ),
                    loader_args=self._robot_loader_args(robot),
                )

                if model.type == ModelType.URDF:
                    assert model.path is not None, f"URDF model {model.name} must have a valid file path"
                    robot_params = (await arena_robots.Robot.RobotIdentifier(robot.model.name).resolve()).model_params

                    fq_name = self._NS_ROBOT(robot.name)

                    await self._clients.SpawnUrdf.call_timeout(
                        SpawnUrdf.Request(
                            name=fq_name,
                            urdf_path=str(model.path),
                            robot_model=robot.model.name,
                            localization=True,
                            tf_prefix=robot.frame.tf(),
                            base_frame=robot_params.base_frame,
                            odom_frame=robot_params.odom_frame,
                            pose=robot.pose.to_msg(),
                            cmd_vel_topic=self.node.service_namespace(robot.name, 'cmd_vel'),
                            joint_states_topic=self.node.service_namespace(robot.name, 'joint_states'),
                            odom_topic=self.node.service_namespace(robot.name, 'odom'),
                        )
                    )

                    control_spec = robot_params.control
                    is_ros2_control = control_spec is not None and control_spec.is_ros2_control

                    # Jazzy controller_manager reads URDF from the robot_description topic, not
                    # its parameter; feed RSP the bridge URDF so the same topic serves both
                    # RViz/Nav2 (which only care about link/joint geometry) and the bridge
                    # controller_manager (which needs TopicBasedSystem as the hw plugin).
                    rsp_urdf_path = str(model.path)
                    if is_ros2_control:
                        rsp_urdf_path = os.path.join('/tmp', f"arena_bridge_{robot.frame.sanitize()}.urdf")
                        ns = str(self.node.service_namespace(robot.name))
                        _transform_urdf_for_bridge(
                            str(model.path),
                            {
                                'velocity': f"{ns}/isaac/joint_commands_velocity",
                                'position': f"{ns}/isaac/joint_commands_position",
                            },
                            f"{ns}/isaac/joint_states",
                            rsp_urdf_path,
                        )

                    await self._launch_robot_stack(robot, robot_params, rsp_urdf_path)

                    self._agent_robots[robot.sim_path] = (robot.frame.tf(robot_params.base_frame), fq_name)
                    return True

                # TODO
                raise NotImplementedError(f"robot model of type {model.type} can't be spawned by {self.__class__.__name__}")

            except Exception as e:
                self._logger.error(f"{repr(e)}\n{traceback.format_exc()}")
                return False

        return await asyncio.gather(*map(impl, robots))

    async def _launch_robot_stack(
        self,
        robot: Robot,
        robot_params: arena_robots.Robot.ModelParams,
        urdf_path: str,
    ) -> None:
        """Launch RSP, and (if ros2_control) controller_manager + spawners + twist_stamper."""
        ns = str(self.node.service_namespace(robot.name))
        with open(urdf_path) as f:
            description = f.read()

        ld = launch.LaunchDescription()
        ld.add_action(launch_ros.actions.PushRosNamespace(namespace=ns))

        ld.add_action(
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'robot_description': description},
                    {'frame_prefix': robot.frame.tf()},
                ],
            )
        )

        control_spec = robot_params.control
        if control_spec is not None and control_spec.is_ros2_control:
            if not control_spec.controllers:
                raise ValueError(f"control.mode=ros2_control but no controllers declared for {robot.name}")

            rendered_yaml = (
                render_ros2_control_yaml(
                    control_spec.config,
                    robot.sim_path,
                    robot.frame.tf(),
                )
                if control_spec.config is not None
                else None
            )

            cm_params: list[object] = [{'use_sim_time': True}]
            if rendered_yaml is not None:
                cm_params.append(rendered_yaml)

            cm_node = launch_ros.actions.Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                output='screen',
                parameters=cm_params,
            )
            ld.add_action(cm_node)
            urdf_pub_node = launch_ros.actions.Node(
                package='arena_runtime',
                executable='urdf_publisher',
                name='urdf_publisher',
                output='screen',
                parameters=[{'robot_description': description}],
            )
            ld.add_action(
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessStart(
                        target_action=cm_node,
                        on_start=[urdf_pub_node],
                    )
                )
            )
            for controller_name in control_spec.controllers:
                ld.add_action(controller_spawner_node(controller_name))
            ld.add_action(
                twist_stamper_node(
                    control_spec.cmd_vel_topic,
                    robot.frame.tf(robot_params.base_frame),
                )
            )
            if control_spec.odom_topic != "odom":
                ld.add_action(odom_relay_node(control_spec.odom_topic))

        await self.node.do_launch(ld)

    async def obstacle_spawn(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:

        async def impl(obstacle: Obstacle) -> Prim | None:
            try:
                model = await (await obstacle.model.resolve()).model.get([ModelType.USD])
                if model.type is ModelType.UNKNOWN:
                    raise ValueError(f"obstacle model {obstacle.model.name} has no USD representation")
            except Exception:
                self._logger.warning(f"Failed to resolve model for obstacle {obstacle.name}")
                self._logger.debug(traceback.format_exc())
                return None
            assert model.path is not None, f"USD model {model.name} must have a valid file path"
            prim = Prim()
            prim.usd_path = str(model.path)
            prim.name = self._NS_PRIM(obstacle.name)
            prim.pose = obstacle.pose.to_msg()
            if obstacle.scale is not None:
                prim.scale.x = obstacle.scale.x
                prim.scale.y = obstacle.scale.y
                prim.scale.z = obstacle.scale.z
            return prim

        prims = await asyncio.gather(*map(impl, obstacles))

        req = SpawnPrims.Request()
        req.prims = list(filter(None, prims))
        response = await self._clients.SpawnPrims.call_timeout(req)
        if response is None:
            return tuple(False for _ in obstacles)

        response_iter = iter(response.ret)

        return tuple((a is not None) and next(response_iter) for a in prims)

    async def obstacle_move(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        return await self._move_entities([(self._NS_PRIM(o.name), o.pose) for o in obstacles])

    async def pedestrian_move(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        req = MovePedestrians.Request(
            pedestrians=[
                Pedestrian(
                    name=self._NS_PEDESTRIAN(p.name),
                    pose=p.pose.to_msg(),
                )
                for p in pedestrians
            ]
        )
        res = await self._clients.MovePedestrians.call_timeout(req)
        if res is None:
            return tuple(False for _ in pedestrians)
        return tuple(r == MovePedestrians.Response.SUCCESS for r in res.results)

    async def robot_move(self, robots: Sequence[Robot]) -> Sequence[bool]:
        async def move_robot(robot: Robot) -> bool:
            try:
                return await self._move_entity(self._NS_ROBOT(robot.name), robot.pose)
            except Exception as e:
                self._logger.error(f"Failed to move robot {robot.name}: {e}\n{traceback.format_exc()}")
                return False

        return await asyncio.gather(*map(move_robot, robots))

    async def obstacle_delete(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        return await asyncio.gather(*(self._delete_entity(self._NS_PRIM(o.name)) for o in obstacles))

    async def pedestrian_delete(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        res = await self._clients.DeletePedestrians.call_timeout(DeletePedestrians.Request(names=[self._NS_PEDESTRIAN(p.name) for p in pedestrians]))
        if res is None:
            return tuple(False for _ in pedestrians)
        return tuple(r == DeletePedestrians.Response.SUCCESS for r in res.results)

    async def robot_delete(self, robots: Sequence[Robot]) -> Sequence[bool]:
        for robot in robots:
            self._agent_robots.pop(robot.sim_path, None)
        return await asyncio.gather(*(self._delete_entity(self._NS_ROBOT(r.name)) for r in robots))

    async def remove_world(self) -> bool:
        res = await self._clients.ResetWorld.call_timeout(ResetWorld.Request())
        return bool(res) and res.ret

    async def spawn_walls(self, walls: Sequence[WallDefinition], clear_existing: bool = True) -> bool:
        if clear_existing:
            await self.remove_world()
        self._logger.debug("Attempting to spawn walls")

        async def create_segment(segment: WallSegment) -> Wall | None:
            end = segment.end.to_msg()
            end.z += segment.height
            try:
                wall_name = self._realizer.realize(f"wall_{next(self.wall_counter)}")
                return Wall(
                    name=self._NS_WALL(wall_name),
                    start=segment.start.to_msg(),
                    end=end,
                    material=material_to_msg(await segment.material.resolve()),
                    thickness=segment.width,
                )

            except Exception as e:
                self._logger.error(f"Failed to spawn wall: {e}\n{traceback.format_exc()}")
                return None

        async def create_obstacle(obstacle: ObstacleDefinition) -> Prim | None:
            try:
                prim_name = self._realizer.realize(f"obstacle_{next(self.wall_counter)}")
                model = await (await obstacle.model.resolve()).model.get(ModelType.USD)
                if model.type is ModelType.UNKNOWN:
                    return None
                assert model.path is not None, f"USD model {model.name} must have a valid file path"
                prim = Prim()
                prim.usd_path = str(model.path)
                prim.name = self._NS_WALL(prim_name)
                prim.pose = obstacle.pose.to_msg()
                return prim

            except Exception as e:
                self._logger.error(f"Failed to spawn wall obstacle: {e}\n{traceback.format_exc()}")
                return None

        async def create_wall(wall: WallDefinition) -> tuple[typing.Iterator[object], typing.Iterator[object]]:
            segments, obstacles = await wall.assets()
            return map(create_segment, segments), map(create_obstacle, obstacles)

        wall_futures = await asyncio.gather(*map(create_wall, walls))
        segment_futures, obstacle_futures = zip(*wall_futures, strict=False)

        walls_req = SpawnWalls.Request()
        prims_req = SpawnPrims.Request()
        walls_req.walls = list(filter(None, await asyncio.gather(*itertools.chain.from_iterable(segment_futures))))
        prims_req.prims = list(filter(None, await asyncio.gather(*itertools.chain.from_iterable(obstacle_futures))))

        walls_res = await self._clients.SpawnWalls.call_timeout(walls_req)
        prims_res = await self._clients.SpawnPrims.call_timeout(prims_req)
        res = bool(walls_res) and all(walls_res.ret) and bool(prims_res) and all(prims_res.ret)

        self._logger.debug("All walls spawned.")
        return res

    async def spawn_floors(self, floors: Sequence[FloorDefinition]) -> bool:
        self._logger.debug("Attempting to spawn floors")

        async def impl(floor: FloorDefinition) -> Floor | None:
            try:
                return Floor(
                    name=self._NS_FLOOR(floor.name),
                    x_length=floor.x_length,
                    y_length=floor.y_length,
                    pos=floor.pos.to_msg(),
                    material=material_to_msg(await floor.material.resolve()),
                )

            except Exception:
                self._logger.error(f"Failed to spawn floor: {floor.name}\n{traceback.format_exc()}")
                return None

        floors_req = SpawnFloors.Request()
        floors_req.floors = list(filter(None, await asyncio.gather(*map(impl, floors))))
        floors_res = await self._clients.SpawnFloors.call_timeout(floors_req)

        res = bool(floors_res) and all(floors_res.ret)
        self._logger.debug("All floors spawned successfully.")
        return res

    async def spawn_box(self, name: str, size: tuple[float, float, float], pose: Pose) -> bool:
        """Spawn an axis-aligned box using the SpawnWalls service.

        SpawnWalls creates a cube via create_cube(scale=(length, thickness, height)).
        Encoding: the AB vector carries sx (length) and yaw; thickness carries sy; the z
        extent of the start/end points carries sz. This lets a single Wall message
        fully describe a box without a dedicated service call.
        """
        sx, sy, sz = size
        yaw = pose.orientation.to_yaw()
        cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
        half = sx / 2.0
        start_x = cx - half * math.cos(yaw)
        start_y = cy - half * math.sin(yaw)
        end_x = cx + half * math.cos(yaw)
        end_y = cy + half * math.sin(yaw)
        req = SpawnWalls.Request()
        req.walls = [
            Wall(
                name=name,
                start=geometry_msgs.msg.Point(x=start_x, y=start_y, z=cz - sz / 2.0),
                end=geometry_msgs.msg.Point(x=end_x, y=end_y, z=cz + sz / 2.0),
                thickness=sy,
            )
        ]
        res = await self._clients.SpawnWalls.call_timeout(req)
        return bool(res) and bool(res.ret) and res.ret[0]

    async def move_box(self, name: str, pose: Pose) -> bool:
        # Fire-and-forget: animation is cosmetic, last-write-wins, don't gate tick rate on bridge latency.
        req = EditPrims.Request(prims=[Prim(name=name, pose=pose.to_msg())], pose=True)
        self._clients.EditPrims.client.call_async(req)
        return True

    async def delete_box(self, name: str) -> bool:
        return await self._delete_entity(name)

    def robot_positions_xy(self) -> Iterable[tuple[str, tuple[float, float]]]:
        out: list[tuple[str, tuple[float, float]]] = []
        for sim_path, (frame, _prim) in list(self._agent_robots.items()):
            try:
                t = self._mechanism_tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            out.append((sim_path, (t.transform.translation.x, t.transform.translation.y)))
        return out

    def robot_pose(self, sim_path: str) -> Pose | None:
        entry = self._agent_robots.get(sim_path)
        if entry is None:
            return None
        frame, _prim = entry
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
        entry = self._agent_robots.get(sim_path)
        if entry is None:
            return False
        _frame, prim_name = entry
        return await self._move_entity(prim_name, pose)

    async def before_reset_episode(self) -> bool:
        return True

    async def after_reset_episode(self) -> bool:
        return True

    async def step(self, n: int = 1) -> bool:
        async with self.node.unpause_window():
            await asyncio.sleep(0.01 * n)
        return True

    async def pedestrian_spawn(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:

        # TODO implement targeted pedestrian models
        available_models: dict[str, str] = {
            # "F_Business_02",
            # "F_Medical_01",
            # "M_Medical_01",
            # "biped_demo",
            # "female_adult_police_01_new",
            # "female_adult_police_02",
            # "female_adult_police_03_new",
            # "male_adult_construction_01_new",
            # "male_adult_construction_03",
            # "male_adult_construction_05_new",
            # "male_adult_police_04",
            "female_adult_business_02": "original_female_adult_business_02",
            "female_adult_medical_01": "original_female_adult_medical_01",
            "female_adult_police_01": "original_female_adult_police_01",
            "female_adult_police_02": "original_female_adult_police_02",
            "female_adult_police_03": "original_female_adult_police_03",
            "male_adult_construction_01": "original_male_adult_construction_01",
            "male_adult_construction_02": "original_male_adult_construction_02",
            "male_adult_construction_03": "original_male_adult_construction_03",
            "male_adult_construction_05": "original_male_adult_construction_05",
            "male_adult_medical_01": "original_male_adult_medical_01",
            "male_adult_police_04": "original_male_adult_police_04",
        }

        items = []
        for pedestrian in pedestrians:
            if pedestrian.model.name in available_models:
                model_name = pedestrian.model.name
            else:
                model_name = random.choice(tuple(available_models.keys()))
            items.append(
                SpawnPedestrian(
                    pedestrian=Pedestrian(
                        name=self._NS_PEDESTRIAN(pedestrian.name),
                        pose=pedestrian.pose.to_msg(),
                    ),
                    model_ref=available_models[model_name],
                )
            )

        req = SpawnPedestrians.Request(pedestrians=items)
        res = await self._clients.SpawnPedestrians.call_timeout(req)
        if res is None:
            return tuple(False for _ in pedestrians)

        success = tuple(r == SpawnPedestrians.Response.SUCCESS for r in res.results)

        await self.pedestrian_update(
            arena_people_msgs.msg.Pedestrians(
                pedestrians=[
                    arena_people_msgs.msg.Pedestrian(
                        name=ped.name,
                        pose=ped.pose.to_msg(),
                    )
                    for status, ped in zip(success, pedestrians, strict=False)
                    if status
                ]
            )
        )

        return success

    async def pedestrian_update(self, pedestrians: arena_people_msgs.msg.Pedestrians) -> Sequence[bool]:
        req = UpdatePedestrians.Request(
            pedestrians=[
                Pedestrian(
                    name=self._NS_PEDESTRIAN(ped.name),
                    pose=ped.pose,
                    twist=ped.twist,
                )
                for ped in pedestrians.pedestrians
            ]
        )
        res = await self._clients.UpdatePedestrians.call_timeout(req)
        if res is None:
            return tuple(False for _ in pedestrians.pedestrians)
        return tuple(r == UpdatePedestrians.Response.SUCCESS for r in res.results)

    async def _delete_entity(self, name: str) -> bool:
        self._logger.debug(f"Attempting to delete prim {name}")

        res = await self._clients.DeletePrims.call_timeout(DeletePrims.Request(names=[name]))
        if res is None:
            return False

        return res.ret[0]

    async def _move_entity(self, name: str, pose: Pose) -> bool:
        return (await self._move_entities([(name, pose)]))[0]

    async def _move_entities(self, actions: Sequence[tuple[str, Pose]]) -> Sequence[bool]:
        req = EditPrims.Request(
            prims=[
                Prim(
                    name=name,
                    pose=pose.to_msg(),
                )
                for name, pose in actions
            ],
            pose=True,
        )

        response = await self._clients.EditPrims.call_timeout(req)
        if response is None:
            return [False] * len(actions)

        return response.ret

    async def setup(self):
        """
        Initialize all ROS 2 service clients and wait for their availability.
        """
        self._logger.info("Setting up IsaacSimulator service clients...")
        futures: list[typing.Awaitable] = []
        # Define services with their corresponding client attributes
        for client in self._clients.__dict__.values():
            client = typing.cast(ClientWrapper, client)
            self._logger.debug(f"Initializing service client: {client.client.srv_name}")
            futures.append(client.ensure())
        await asyncio.gather(*futures)

        self._logger.info("All service clients are available.")

        self.node.create_publisher(std_msgs.msg.String, '/isaac/add_pedestrians_topic', 10).publish(std_msgs.msg.String(data=self._namespace('arena_peds')))

        self._logger.info("All service clients initialized and available.")

    @classmethod
    async def create(cls, *args: object, namespace: Namespace, **kwargs: object) -> "IsaacSimulator":
        self = cls(*args, namespace=namespace, **kwargs)
        self._logger.info("Creating IsaacSimulator instance...")
        await self.setup()
        return self
