import os
import launch
import launch_ros

def generate_launch_description():
    map_path = os.path.join(os.path.expanduser("~"), 'test_map')

    return launch.LaunchDescription([launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            output='screen',
            arguments=['-f', map_path],
            parameters=[{'save_map': True}]),
        ])
