"""ROS 2 Jazzy / Gazebo Harmonic. All worlds start paused."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    pkg=Path(get_package_share_directory('bellybot_aircraft_environment'))
    name=LaunchConfiguration('world').perform(context)
    world=pkg/'worlds'/(name+'.sdf')
    if not world.is_file():raise RuntimeError('Unknown world: '+name)
    paths=str(pkg/'models')+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',paths),
      ExecuteProcess(cmd=['gz','sim','-v','3',str(world)],output='screen'),
      Node(package='ros_gz_bridge',executable='parameter_bridge',
        arguments=['--ros-args','-p','config_file:='+str(pkg/'config/bridge.yaml')],output='screen')]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('world',default_value='loading_task'),OpaqueFunction(function=setup)])
