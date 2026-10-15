"""Live RViz for the work configuration; launch alongside aircraft mode:=transfer."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 p=Path(get_package_share_directory('bellybot_aircraft_environment'))
 return LaunchDescription([Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(p/'urdf/aircraft_system.urdf').read_text(),'use_sim_time':True}]),Node(package='rviz2',executable='rviz2',arguments=['-d',str(p/'rviz/environment.rviz')],parameters=[{'use_sim_time':True}])])
