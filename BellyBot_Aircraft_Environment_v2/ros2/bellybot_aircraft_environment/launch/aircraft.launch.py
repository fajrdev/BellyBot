"""Start the revised world PAUSED, bridge feedback, and enforce aircraft limits."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
 p=Path(get_package_share_directory('bellybot_aircraft_environment'));bp=Path(get_package_share_directory('bellybot_dock_stack_description'));mode=LaunchConfiguration('mode').perform(context);cut=LaunchConfiguration('cutaway').perform(context).lower()=='true'
 if mode not in ['entry','transfer']:raise ValueError('mode must be entry or transfer')
 bag_height=float(LaunchConfiguration('bag_height').perform(context));bag_mass=float(LaunchConfiguration('bag_mass').perform(context))
 name='entry_resolved' if mode=='entry' else ('transfer_cutaway' if cut else 'transfer_resolved')
 paths=os.pathsep.join([str(p/'models'),str(bp.parent),os.environ.get('GZ_SIM_RESOURCE_PATH','')])
 return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',paths),ExecuteProcess(cmd=['gz','sim','-v','3',str(p/'worlds'/f'{name}.sdf')],output='screen'),Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(p/'config/aircraft_bridge.yaml')}],output='screen'),Node(package='bellybot_aircraft_environment',executable='aircraft_controller.py',parameters=[{'use_sim_time':True,'mode':mode,'bag_height':bag_height,'bag_mass':bag_mass}],output='screen')]
def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('mode',default_value='transfer'),DeclareLaunchArgument('cutaway',default_value='false'),DeclareLaunchArgument('bag_height',default_value='0.38'),DeclareLaunchArgument('bag_mass',default_value='32.0'),OpaqueFunction(function=setup)])
