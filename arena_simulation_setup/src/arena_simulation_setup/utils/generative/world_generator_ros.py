import json
import sys

import std_srvs.srv
from arena_rclpy_mixins.ServiceNamespace import ServiceNamespace

from arena_simulation_setup.tree.World.World import WorldDescription, WorldIdentifier

from .world_generator import WorldGenerator, WorldGeneratorType


class WorldGeneratorROS(WorldGenerator, ServiceNamespace):
    def _get_parameters(self) -> tuple[WorldGeneratorType, dict]:
        name = WorldGeneratorType(self.get_parameter('generator').value)
        config = json.loads(self.get_parameter('configuration').value)  # type: ignore

        self.get_logger().info(f'world generator: "{name}"')
        self.get_logger().info(f'config: {config}')

        return name, config

    def _cb_generate(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response) -> std_srvs.srv.Trigger.Response:
        try:
            self.update_generator(*self._get_parameters())
            WorldIdentifier('.generated').resolve_sync().save(WorldDescription.from_levels(self.compute()))
            response.success = True
        except BaseException as e:
            response.success = False
            response.message = repr(e)
            self.get_logger().error(f"Failed to generate world: {repr(e)}")

        return response

    def __init__(self):
        ServiceNamespace.__init__(self, 'world_generator')

        self.declare_parameter('generator', WorldGeneratorType.HALLWAY.value)
        self.declare_parameter('configuration', '{}')

        WorldGenerator.__init__(self, *self._get_parameters())

        self.set_up_services()
        self.get_logger().info('initialized')

    def set_up_services(self):
        self.create_service(std_srvs.srv.Trigger, self.service_namespace('generate_world'), self._cb_generate)


def main(argv: list[str] = sys.argv) -> None:
    import os

    import rclpy
    import rclpy.utilities
    from arena_rclpy_mixins.spin import spin_node

    rclpy.init(args=argv)
    argv = rclpy.utilities.remove_ros_args(argv)

    if len(argv) > 1:
        print(f'usage: {os.path.basename(__file__)}')
        sys.exit(1)

    spin_node(WorldGeneratorROS())


if __name__ == '__main__':
    main()
