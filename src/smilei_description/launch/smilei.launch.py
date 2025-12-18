import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path


from launch import LaunchDescription, LaunchContext
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from nav2_common.launch import RewrittenYaml
import xacro

from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    robot_name = "smilei"

    urdf_path = os.path.join(
        get_package_share_path("smilei_description"),
        "urdf",
        "smilei.urdf.xacro",
    )

    yaml_path = os.path.join(
        get_package_share_directory("smilei_description"),
        "config",
        "smily_controllers.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    namespace = "/" + robot_name
    param_substitutions = {"use_sim_time": use_sim_time}

    configure_params = RewrittenYaml(
        yaml_path,
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    context = LaunchContext()
    controller_param = configure_params.perform(context)
    controller_type="1"

    robot_doc = xacro.process_file(
        urdf_path,
        mappings={
            "namespace": namespace,
            "sim_gazebo": "1",
            "controller_type": controller_type,
            "use_sim_time": use_sim_time,
            "prefix": "",
            "simulation_controllers": controller_param,
        },
    )

    robot_description = robot_doc.toprettyxml(indent=" ")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_params = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        output="screen",
        remappings=remappings,
        parameters=[robot_params],
    )

    robot_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            namespace + "/robot_description",
            "-entity",
            robot_name,
            "-robot_namespace",
            namespace,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-Y",
            "0.0",
            "-unpause",
        ],
        output="screen",
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        namespace=namespace,
        executable="joint_state_publisher",
        name="joint_state_publisher",
        remappings=remappings,
    )
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    load_position_controller_right = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "forward_position_controller_right",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    load_position_controller_left = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "forward_position_controller_left",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_position_controller_right],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_position_controller_right,
                    on_exit=[load_position_controller_left],
                )
            ),
            gazebo,
            robot_state_publisher,
            joint_state_publisher_node,
            robot_spawn_entity
        ]
    )
