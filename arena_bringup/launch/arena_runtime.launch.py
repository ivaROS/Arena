import launch
import launch_ros.actions
from arena_bringup.actions import IsolatedGroupAction
from arena_bringup.extensions.NodeLogLevelExtension import SetGlobalLogLevelAction
from arena_bringup.substitutions import LaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_RUNTIME_OWNED = frozenset({'log_level', 'sim', 'use_sim_time', 'world', 'headless', 'slot_buffer'})


def generate_launch_description():
    ld_items = []
    LaunchArgument.auto_append(ld_items)

    log_level = LaunchArgument(
        name='log_level',
        default_value='warn',
        description='Per-node log level. See launch README.',
    )

    sim = LaunchArgument(
        name='sim',
        default_value='dummy',
    )

    use_sim_time = LaunchArgument(
        name='use_sim_time',
        default_value='true',
    )

    world = LaunchArgument(
        name='world',
        default_value='map_empty',
    )

    headless = LaunchArgument(
        name='headless',
        default_value='False',
    )

    slot_buffer = LaunchArgument(
        name='slot_buffer',
        default_value='5.0',
        description='Runtime env packing buffer in metres. Use 0 for literal world coordinates.',
    )

    launch_sim = launch.actions.IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('arena_bringup'),
            'launch', 'simulator', 'sim', 'sim.launch.py',
        ]),
        launch_arguments={
            **use_sim_time.dict,
            **sim.dict,
            **world.dict,
            'headless': headless.substitution,
        }.items(),
    )

    world_generator_node = launch_ros.actions.Node(
        package='arena_simulation_setup',
        executable='world_generator',
        name='world_generator',
        output='screen',
    )

    def _arena_node(context: launch.LaunchContext) -> list[launch.Action]:
        env_args = [
            f'{k}:={v}'
            for k, v in context.launch_configurations.items()
            if k not in _RUNTIME_OWNED
        ]
        return [
            launch_ros.actions.LifecycleNode(
                package='arena_runtime',
                executable='arena_node',
                name='arena',
                namespace='',
                output='screen',
                parameters=[{
                    'sim': sim.substitution,
                    'env_args': env_args,
                    'slot_buffer': slot_buffer.substitution,
                }],
            ),
        ]

    return launch.LaunchDescription([
        *ld_items,
        SetGlobalLogLevelAction(log_level.substitution),
        IsolatedGroupAction([launch_sim]),
        world_generator_node,
        launch.actions.OpaqueFunction(function=_arena_node),
    ])


if __name__ == '__main__':
    generate_launch_description()
