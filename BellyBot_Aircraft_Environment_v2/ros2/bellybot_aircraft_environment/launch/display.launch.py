from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('bellybot_aircraft_environment'))
    return LaunchDescription([
      Node(package='robot_state_publisher',executable='robot_state_publisher',name='environment_state_publisher',
        parameters=[{'robot_description':(p/'urdf/environment.urdf').read_text()}]),
      Node(package='rviz2',executable='rviz2',arguments=['-d',str(p/'rviz/environment.rviz')])])
