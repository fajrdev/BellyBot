"""Launch the BellyBot Mission Variant: door-docked shuttle and mobile manipulation.

Loads:
- worlds/mission_shuttle.sdf (no corner_transfer, no longitudinal_buffer, inclined door_belt)
- config/mission_bridge.yaml (purged station feedback; added dynamic pose stream)
- scripts/mission_controller.py (FSM, wall watchdog, measured predicates)
- scripts/mission_observer.py (ground-truth observer, settling dwell verification, metrics)
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    p = Path(get_package_share_directory('bellybot_aircraft_environment'))
    bp = Path(get_package_share_directory('bellybot_dock_stack_description'))
    
    bag_height = float(LaunchConfiguration('bag_height').perform(context))
    bag_mass = float(LaunchConfiguration('bag_mass').perform(context))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    run_id = LaunchConfiguration('run_id').perform(context)

    paths = os.pathsep.join([
        str(p / 'models'),
        str(bp.parent),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ])

    world_arg = LaunchConfiguration('world').perform(context)
    if world_arg:
        world_sdf = world_arg
    else:
        world_sdf = str(p / 'worlds/mission_shuttle.sdf')
    gz_args = ['gz', 'sim', '-v', '3']
    if headless:
        gz_args.extend(['-s', '--headless-rendering'])
    gz_args.append(world_sdf)

    bridge_config = str(p / 'config/mission_bridge.yaml')

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', paths),
        ExecuteProcess(cmd=gz_args, output='screen'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen'
        ),
        Node(
            package='bellybot_aircraft_environment',
            executable='mission_controller.py',
            parameters=[{
                'use_sim_time': True,
                'bag_height': bag_height,
                'bag_mass': bag_mass,
                'run_id': run_id
            }],
            output='screen'
        ),
        Node(
            package='bellybot_aircraft_environment',
            executable='mission_observer.py',
            parameters=[{
                'use_sim_time': True,
                'run_id': run_id
            }],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('bag_height', default_value='0.38'),
        DeclareLaunchArgument('bag_mass', default_value='32.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('run_id', default_value='run_001'),
        OpaqueFunction(function=setup)
    ])
