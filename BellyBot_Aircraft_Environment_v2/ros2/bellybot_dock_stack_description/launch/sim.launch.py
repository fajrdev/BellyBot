from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,AppendEnvironmentVariable,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def setup(context):
    p=Path(get_package_share_directory('bellybot_dock_stack_description'))
    world=LaunchConfiguration('world').perform(context)
    if world not in ['bench','cargo_hold','floor_pickup','batch_transfer']:raise ValueError('world must be bench, cargo_hold, floor_pickup or batch_transfer')
    sim=Path(get_package_share_directory('ros_gz_sim'))
    return [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(p.parent)),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':f'-r {p / "gazebo" / (world+".sdf")}'}.items()),
      Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(p/'config/bridge.yaml')}],output='screen'),
      Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',
        parameters=[{'robot_description':(p/'urdf/bellybot.urdf').read_text(),'use_sim_time':True}],output='screen'),
      Node(package='robot_state_publisher',executable='robot_state_publisher',name='infeed_state_publisher',
        parameters=[{'robot_description':(p/'urdf/infeed.urdf').read_text(),'use_sim_time':True}],
        remappings=[('joint_states','infeed_joint_states'),('robot_description','infeed_robot_description')]),
      Node(package='tf2_ros',executable='static_transform_publisher',name='infeed_world_tf',
        arguments=['--x','0','--y','0','--z','0','--frame-id','odom','--child-frame-id','infeed_base'],parameters=[{'use_sim_time':True}]),
      Node(package='bellybot_dock_stack_description',executable='supervisor.py',parameters=[{'use_sim_time':True}],output='screen'),
      Node(package='rviz2',executable='rviz2',arguments=['-d',str(p/'rviz/simulation.rviz')],
        parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')))]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('world',default_value='bench'),
      DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
