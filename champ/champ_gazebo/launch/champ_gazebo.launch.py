import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def printGT(context, *args, **kwargs):
    spawner_print = LaunchConfiguration('ros_control_file').perform(context)
    print('Champ Gazebo Launch File')
    print(f"Spawner_path: {spawner_print}")

def generate_launch_description():

    # Set the GZ_SIM_SYSTEM_PLUGIN_PATH
    plugin_path = "/opt/ros/jazzy/lib/"  # Replace with the actual path
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = plugin_path

    robot_name = LaunchConfiguration("robot_name")
    # use_sim_time = LaunchConfiguration("use_sim_time")
    # gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    paused = LaunchConfiguration("paused")
    lite = LaunchConfiguration("lite")
    ros_control_file = LaunchConfiguration("ros_control_file")

    gz_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find(
        "champ_gazebo"
    )
    pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_description").find("champ_description")
    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_config"
    ).find("champ_config")
        

    # ----------------------------------------- Defaults ----------------------------------------- #
    default_model_path = os.path.join(pkg_share, "urdf/champ.urdf.xacro")
    declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")
    default_bridge_params = os.path.join(get_package_share_directory('champ_gazebo'), 'params', 'champ_gz_bridge.yaml')
    
    # ----------------------------------- Parameter declaration ---------------------------------- #
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="champ")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_paused = DeclareLaunchArgument("paused", default_value="False")
    declare_lite = DeclareLaunchArgument("lite", default_value="False")
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(gz_pkg_share, "config/ros_control.yaml"),
    )
       
    declare_bridge_params = DeclareLaunchArgument("bridge_params", default_value=default_bridge_params, description="Path to the Gazebo ROS bridge parameters file")
    
    
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")

    # TODO as for right now, running contact sensor results in RTF being reduced by factor of 2x.
    # So it needs to be fixed before using that. Unsure what it does actually because even without it
    # Champ seems to be all right
    contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")},links_config],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    # Controllers
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--switch-timeout", "200",
                    "--activate-as-group",
                   "-p", LaunchConfiguration("ros_control_file"),
                    "joint_state_broadcaster", "joint_group_effort_controller"
        ]
    )

    # ROS / GZ topic bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': LaunchConfiguration('bridge_params')},
        ],
        output='screen',
    )

    # joint_group_position_controller
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_headless,
            declare_paused,
            declare_lite,
            declare_ros_control_file,
            declare_description_path,
            declare_bridge_params,
            controller_spawner,
            contact_sensor,
            start_gazebo_ros_bridge_cmd, # ROS Bridge
            OpaqueFunction(function=printGT)
        ]
    )