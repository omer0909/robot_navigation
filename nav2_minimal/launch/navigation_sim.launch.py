import launch
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_minimal')
    map_path = os.path.join(os.path.expanduser("~"), 'test_map.yaml')

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

    launch_map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
        }],
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}])

    launch_amcl = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_map_topic': True,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'global_frame_id': 'map',
        }],
        remappings=[('/initialpose', 'initialpose'),
                     ('/pose', 'pose')]
    )

    robot_localization_node = launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'configs/ekf.yaml'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

    nav2_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'nav2_bringup'), 'launch', 'localization_launch.py')
            ),
            launch_arguments={"namespace" : "",
                              'map': map_path,
                              'use_sim_time': "True",
                              'params_file': os.path.join(
                pkg_share, 'configs/nav2_params.yaml')}.items(),
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
        open_robot,
        # launch_map_server,
        # start_lifecycle_manager_cmd,
        # launch_amcl,
        # robot_localization_node,
        nav2_localization,
        launch_nav2,
        open_rviz
    ])
