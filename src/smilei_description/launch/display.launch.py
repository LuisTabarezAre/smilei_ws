import os
import xacro
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------
    # Launch arguments
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_name = 'smilei'
    description_pkg = 'smilei_description'
    description_path = get_package_share_directory(description_pkg)

    # -----------------------------
    # Gazebo resource paths
    # (equivalente a example_9)
    # -----------------------------
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path += ':' + str(Path(description_path).parent)
    gz_resource_path += ':' + os.path.join(description_path, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    # -----------------------------
    # Gazebo
    # -----------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['empty.sdf -r']
        }.items(),
    )

    # -----------------------------
    # Robot description (XACRO)
    # -----------------------------
    xacro_file = os.path.join(
        description_path,
        'urdf',
        'smilei.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_sim': 'true'}
    ).toxml()

    # -----------------------------
    # robot_state_publisher
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # -----------------------------
    # Spawn robot
    # -----------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', robot_name,
            '-z', '0.05'
        ],
    )

    # -----------------------------
    # ros2_control spawners
    # -----------------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    left_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller_left',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    right_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller_right',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # -----------------------------
    # RViz
    # -----------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-d',
            os.path.join(description_path, 'rviz', 'rviz_config.rviz')
        ],
    )

    # -----------------------------
    # Launch sequence (CRÍTICO)
    # -----------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        gazebo,
        robot_state_publisher,
        spawn_robot,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[
                    left_controller,
                    right_controller
                ],
            )
        ),

        rviz,
    ])
