from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    lidar_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch',
                         'stl27l.launch.py')))

    rosbridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch',
                         'rosbridge_websocket_launch.xml')))

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_minimal_description'), 'launch',
                         'robot_description.launch.py')))

    wheel_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_control_demo_example_2'), 'launch',
                         'diffbot.launch.py')))
    
    nav2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_minimal'), 'launch',
                         'navigation_sim_slam.launch.py')))

    ld.add_action(lidar_sensor)
    ld.add_action(rosbridge)
    ld.add_action(robot_description)
    ld.add_action(wheel_controller)
    ld.add_action(nav2_slam)

    return ld
