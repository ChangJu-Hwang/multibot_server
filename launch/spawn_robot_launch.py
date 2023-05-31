from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-entity', launch.substitutions.LaunchConfiguration('robot_name'),
                '-file', launch.substitutions.LaunchConfiguration('sdf_file'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-Y', launch.substitutions.LaunchConfiguration('Y')
            ]
        )
    ])