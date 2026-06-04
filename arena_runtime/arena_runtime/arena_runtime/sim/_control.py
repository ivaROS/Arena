"""Shared launch helpers for the ros2_control plane (Gazebo + Isaac)."""

import os
import tempfile

import launch_ros.actions
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def render_ros2_control_yaml(
    config_uri: str,
    sim_path: str,
    frame_prefix: str,
) -> str:
    """Resolve, render, and write a ros2_control YAML to /tmp.

    Resolves package:// URIs, prefixes bare frame_id values, and writes
    the result to a deterministic temp path. Returns the absolute output path.
    """
    if config_uri.startswith("package://"):
        pkg, _, sub = config_uri[len("package://") :].partition('/')
        try:
            src_path = os.path.join(get_package_share_directory(pkg), sub)
        except PackageNotFoundError as e:
            raise FileNotFoundError(f"control.config package '{pkg}' not found") from e
    else:
        src_path = config_uri

    with open(src_path) as f:
        data = yaml.safe_load(f)

    def _prefix_frames(obj: object) -> object:
        if isinstance(obj, dict):
            return {k: (f"{frame_prefix}{v}" if isinstance(k, str) and k.endswith("_frame_id") and isinstance(v, str) and "/" not in v else _prefix_frames(v)) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_prefix_frames(x) for x in obj]
        return obj

    rendered = _prefix_frames(data)
    if isinstance(rendered, dict):
        rendered = {(k if k.startswith('/') else f'/**/{k}'): v for k, v in rendered.items()}

    out_path = os.path.join(tempfile.gettempdir(), f"arena_control_{sim_path.replace('/', '_')}.yaml")
    with open(out_path, 'w') as f:
        yaml.safe_dump(rendered, f, sort_keys=False)
    return out_path


def controller_spawner_node(controller_name: str) -> launch_ros.actions.Node:
    """Spawn one controller into the namespace-local controller_manager.

    `--controller-manager-timeout 0` is the spawner's wait-forever sentinel.
    `--switch-timeout 600` is a large finite (CM rejects 0 and uses 1s); covers
    the sim-paused-during-reset window without masking real hangs forever.
    """
    return launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{controller_name}',
        output='screen',
        arguments=[
            controller_name,
            '--controller-manager',
            'controller_manager',
            '--controller-manager-timeout',
            '0',
            '--switch-timeout',
            '600',
            '--service-call-timeout',
            '600',
        ],
        parameters=[{'use_sim_time': True}],
    )


def odom_relay_node(odom_topic: str) -> launch_ros.actions.Node:
    """Relay `<odom_topic>` (controller-specific) -> `odom` for the Arena convention."""
    return launch_ros.actions.Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',
        arguments=[odom_topic, 'odom'],
        parameters=[{'use_sim_time': True}],
    )


def twist_stamper_node(cmd_vel_topic: str, frame_id: str) -> launch_ros.actions.Node:
    """Stamp `cmd_vel` (Twist) -> `cmd_vel_out` (TwistStamped) for stamped controllers."""
    return launch_ros.actions.Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel'),
            ('cmd_vel_out', cmd_vel_topic),
        ],
        parameters=[
            {
                'use_sim_time': True,
                'frame_id': frame_id,
            }
        ],
    )
