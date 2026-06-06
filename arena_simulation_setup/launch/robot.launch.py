import launch_ros
from arena_bringup.future import PythonExpression
from arena_bringup.substitutions import LaunchArgument
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions


def generate_launch_description():

    ss_path = FindPackageShare('arena_simulation_setup')

    ld_items = []
    LaunchArgument.auto_append(ld_items)

    use_sim_time = LaunchArgument("use_sim_time")

    task_generator_node = LaunchArgument('task_generator_node')
    namespace = LaunchArgument("namespace")
    robot = LaunchArgument("robot")
    frame = LaunchArgument("frame")

    record_data_dir = LaunchArgument('record_data_dir', default_value='')

    # launch robot control
    state_pub_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    ss_path,
                    "launch",
                    "state_publisher.launch.py",
                ]
            )),
        launch_arguments={
            **use_sim_time.dict,
            **frame.dict,
            **namespace.dict,
            **robot.dict,
        }.items(),
    )

    data_recorder = launch_ros.actions.Node(
        package='arena_evaluation',
        executable='record',
        name=PythonExpression(['"data_recorder" + "', namespace.substitution, '".replace("/","_")']),
        # Pass --dir as two argv tokens; a nested list was not reliably
        # reaching argparse, so benchmark recorder output fell back to auto:/.
        arguments=[
            '--dir',
            record_data_dir.substitution,
        ],
        condition=launch.conditions.IfCondition(PythonExpression(['bool("', record_data_dir.substitution, '")'])),
    )

    ld = launch.LaunchDescription([
        *ld_items,
        PushRosNamespace(namespace=namespace.substitution),
        # robot_localization_node,
        # state_pub_launch,
        data_recorder,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
