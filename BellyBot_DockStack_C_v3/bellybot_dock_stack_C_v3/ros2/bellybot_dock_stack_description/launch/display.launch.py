from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('bellybot_dock_stack_description'))
    desc=(p/'urdf/display.urdf').read_text();demo=LaunchConfiguration('demo')
    return LaunchDescription([
      DeclareLaunchArgument('demo',default_value='false'),
      Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':desc}],output='screen'),
      Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',
           parameters=[{'robot_description':desc,'rate':60,'zeros':{'infeed_extension_joint':.4}}],condition=UnlessCondition(demo)),
      Node(package='bellybot_dock_stack_description',executable='joint_demo.py',condition=IfCondition(demo)),
      Node(package='rviz2',executable='rviz2',arguments=['-d',str(p/'rviz/dock_stack.rviz')])])
