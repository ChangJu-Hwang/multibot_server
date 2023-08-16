import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import launch_ros.actions

import yaml

def generate_launch_description():
    # Get the launch directory
    multibot_server_dir = get_package_share_directory('multibot_server')
    multibot_robot_dir  = get_package_share_directory('multibot_robot')

    # Launch argument setting
    lifecycle_nodes = ['map_server']
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = True
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    delcare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Rviz, Gazebo) clock if true'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(multibot_server_dir, 'worlds', 'testbed.world'),
    #     description='Full path to world model file to load'
    # )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(multibot_server_dir, 'worlds', 'modified_testbed.world'),
        description='Full path to world model file to load'
    )

    # rviz
    rviz_config_dir = os.path.join(
        multibot_server_dir,
        'rviz',
        'multibot_server.rviz'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Map server
    map_server_param_path = os.path.join(
        multibot_server_dir,
        'maps',
        'map_server_params.yaml'
    )

    configured_params = RewrittenYaml(
        source_file=map_server_param_path,
        root_key='',
        param_rewrites={},
        convert_types=True
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        # parameters=[{'yaml_filename': map_server_config_path}])
        parameters=[configured_params,
                    {'autostart': autostart},
                    ]
    )

    with open(os.path.join(multibot_server_dir, 'maps', 'map_server_params.yaml')) as map_server_params:
        map_server_params = yaml.load(map_server_params, Loader=yaml.Loader)
        map_param_dir = map_server_params['map_server']['ros__parameters']['yaml_filename']

        with open(os.path.join(map_param_dir)) as map_params:
            map_params = yaml.load(map_params, Loader=yaml.Loader)
            map_origin = map_params['origin']

    world_map_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[str(map_origin[0]), str(map_origin[0]), '0', '0', '0', '0', 'world', 'map']
    )

    # lifecycle manager
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
    )

    # Gazebo
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
		cwd=[multibot_server_dir], output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
		condition=IfCondition(PythonExpression(['not ', headless])),
		cmd=['gzclient'],
		cwd=[multibot_server_dir], output='screen'
    )

    spawn_robots_cmds = []
    with open(os.path.join(multibot_server_dir, 'task', 'multibot_task.yaml')) as multibot_task:
        agents = yaml.load_all(multibot_task, Loader=yaml.FullLoader)
    
        for agent in agents:
            spawn_robots_cmds.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(multibot_robot_dir, 'launch',
                                                            'multibot_robot_simul_launch.py')),
                    launch_arguments={
                        'robot_namespace': agent['name'],
                        'robot_name': agent['name'],
                        'sdf_file': os.path.join(multibot_robot_dir, 'models',
                                                 agent['type'], 'model.sdf'),
                        'x': str(agent['start']['x']),
                        'y': str(agent['start']['y']),
                        'Y': str(agent['start']['theta']),
                        'linear_tolerance': '0.10',
                        'angular_tolerance': '0.018',
                        'Kx': '0.25',
                        'Ky': '1.00',
                        'Ktheta': '1.27'
                    }.items()
                )
            )

    # Server Node
    multibot_server_cmd = Node(
        package='multibot_server',
        executable='server',
        name='server',
        output='screen',
        parameters=[os.path.join(multibot_server_dir, 'agents', 'agentConfig.yaml'),
                    {'task_fPath': os.path.join(multibot_server_dir, 'task', 'multibot_task.yaml')}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(delcare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(world_map_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    ld.add_action(multibot_server_cmd)

    return ld