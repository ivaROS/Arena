from __future__ import annotations

import math
from typing import TYPE_CHECKING

import arena_people_msgs.msg
import arena_robots_msgs.msg
import geometry_msgs.msg
import nav2_msgs.msg
import rclpy
import rclpy.node
import shapely
import shapely.affinity
from arena_robots.caps import PolygonSpec
from rclpy.parameter import Parameter

if TYPE_CHECKING:
    from task_generator.manager.environment_manager import EnvironmentManager
    from task_generator.manager.robot_manager.robot_manager import RobotManager

_ACTION_CODES: dict[str | None, int] = {
    None: 0,
    'stop': 1,
    'slowdown': 2,
    'approach': 3,
    'limit': 4,
}


class CollisionTrackerNode(rclpy.node.Node):
    """Separate Node, same process, same executor as the task_generator.

    Reads RobotManager.pose + EnvironmentManager geometry caches and publishes
    collision topics. Tick timer uses sim time → automatically gated during
    pauses.
    """

    def __init__(
        self,
        robot_manager: RobotManager,
        polygons: dict[str, PolygonSpec],
        *,
        rate_hz: float = 10.0,
        near_miss_margin: float = 0.0,
        default_pedestrian_radius: float = 0.3,
    ):
        super().__init__(
            'collision_tracker',
            namespace=str(robot_manager.namespace),
            use_global_arguments=False,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        self._rm = robot_manager
        self._env: EnvironmentManager = robot_manager._environment_manager
        self._near_miss_margin = near_miss_margin
        self._default_peds_radius = default_pedestrian_radius
        self._peds_msg: arena_people_msgs.msg.Pedestrians | None = None

        self._poly_cache: dict[str, dict] = {}
        for name, spec in polygons.items():
            if spec.type == 'polygon':
                if spec.points is None or len(spec.points) < 3:
                    self.get_logger().warning(f'collision_tracker: skipping polygon {name!r}: fewer than 3 points')
                    continue
                self._poly_cache[name] = {
                    'type': 'polygon',
                    'shapely': shapely.Polygon(spec.points),
                    'action_code': _ACTION_CODES.get(spec.action_type, 0),
                }
            else:
                r_c = spec.radius if spec.radius is not None else 0.0
                self._poly_cache[name] = {
                    'type': 'circle',
                    'R_c': r_c,
                    'action_code': _ACTION_CODES.get(spec.action_type, 0),
                }

        self._pub_state = self.create_publisher(nav2_msgs.msg.CollisionMonitorState, 'collision_monitor_state', 10)
        self._pub_events = self.create_publisher(arena_robots_msgs.msg.CollisionEvents, 'collision_events', 10)
        self._sub_peds = self.create_subscription(
            arena_people_msgs.msg.Pedestrians,
            str(robot_manager.namespace('arena_peds')),
            self._on_peds,
            10,
        )
        env_peds_topic = str(robot_manager._namespace('arena_peds'))  # pylint: disable=protected-access
        self._sub_env_peds = None
        if env_peds_topic != str(robot_manager.namespace('arena_peds')):
            self._sub_env_peds = self.create_subscription(
                arena_people_msgs.msg.Pedestrians,
                env_peds_topic,
                self._on_peds,
                10,
            )
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _on_peds(self, msg: arena_people_msgs.msg.Pedestrians):
        self._peds_msg = msg

    def _robot_polygon(self, entry: dict, rx: float, ry: float, rth: float) -> shapely.Polygon:
        if entry['type'] == 'polygon':
            poly = shapely.affinity.rotate(entry['shapely'], rth, origin=(0, 0), use_radians=True)
            return shapely.affinity.translate(poly, xoff=rx, yoff=ry)
        return shapely.Point(rx, ry).buffer(entry['R_c'])

    def _tick(self):
        pose = self._rm.pose
        if pose is None:
            return

        rx, ry, rth = pose.to_2d()

        walls = self._env.walls_geometry
        statics = self._env.static_polygons

        events: list[arena_robots_msgs.msg.CollisionEvent] = []
        polygons_hit: dict[str, int] = {}

        for name, entry in self._poly_cache.items():
            robot_poly = self._robot_polygon(entry, rx, ry, rth)

            if not walls.is_empty and robot_poly.intersects(walls):
                ev = arena_robots_msgs.msg.CollisionEvent()
                ev.obstacle_id = '<wall>'
                ev.polygon_name = name
                ev.distance = 0.0
                ev.obstacle_position = geometry_msgs.msg.Point(x=rx, y=ry, z=0.0)
                events.append(ev)
                polygons_hit[name] = entry['action_code']

            for obs_name, poly in statics.items():
                if not robot_poly.intersects(poly):
                    continue
                centroid = poly.centroid
                ev = arena_robots_msgs.msg.CollisionEvent()
                ev.obstacle_id = obs_name
                ev.polygon_name = name
                ev.distance = 0.0
                ev.obstacle_position = geometry_msgs.msg.Point(x=float(centroid.x), y=float(centroid.y), z=0.0)
                events.append(ev)
                polygons_hit[name] = entry['action_code']

            if self._peds_msg is not None:
                for p in self._peds_msg.pedestrians:
                    px, py = p.pose.position.x, p.pose.position.y
                    if math.isnan(px) or math.isnan(py):
                        continue
                    ped_disc = shapely.Point(px, py).buffer(self._default_peds_radius)
                    if not robot_poly.intersects(ped_disc):
                        continue
                    ev = arena_robots_msgs.msg.CollisionEvent()
                    ev.obstacle_id = p.name
                    ev.polygon_name = name
                    ev.distance = 0.0
                    ev.obstacle_position = geometry_msgs.msg.Point(x=px, y=py, z=0.0)
                    events.append(ev)
                    polygons_hit[name] = entry['action_code']

            robots_manager = getattr(self._rm.node, 'robots_manager', None)
            if robots_manager is not None:
                for other_name, other in robots_manager.managers.items():
                    if other is self._rm:
                        continue
                    other_pose = other.pose
                    if other_pose is None:
                        continue
                    ox, oy, _ = other_pose.to_2d()
                    if math.isnan(ox) or math.isnan(oy):
                        continue
                    other_disc = shapely.Point(ox, oy).buffer(max(other.radius, 0.01))
                    if not robot_poly.intersects(other_disc):
                        continue
                    ev = arena_robots_msgs.msg.CollisionEvent()
                    ev.obstacle_id = f'<robot:{other_name}>'
                    ev.polygon_name = name
                    ev.distance = 0.0
                    ev.obstacle_position = geometry_msgs.msg.Point(x=ox, y=oy, z=0.0)
                    events.append(ev)
                    polygons_hit[name] = entry['action_code']

        state = nav2_msgs.msg.CollisionMonitorState()
        if polygons_hit:
            name, code = min(polygons_hit.items(), key=lambda kv: (1 if kv[1] == 0 else 0, kv[1]))
            state.polygon_name = name
            state.action_type = code
        else:
            state.polygon_name = ''
            state.action_type = 0
        self._pub_state.publish(state)

        evmsg = arena_robots_msgs.msg.CollisionEvents()
        evmsg.header.stamp = self.get_clock().now().to_msg()
        evmsg.header.frame_id = 'map'
        evmsg.events = events
        self._pub_events.publish(evmsg)

    def shutdown(self):
        self.destroy_node()
