import launch
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_minimal')

    open_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={'headless': "False", "use_rviz" : "False"}.items()
        )

    open_rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', '/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'],
    )

    robot_localization_node = launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'configs/ekf.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

    launch_slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
        )

    launch_nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'params_file': os.path.join(
                pkg_share, 'configs/nav2_params.yaml')}.items(),
        )

    return launch.LaunchDescription([
        # open_robot,
        # robot_localization_node,
        launch_slam_toolbox,
        launch_nav2,
        open_rviz
    ])
