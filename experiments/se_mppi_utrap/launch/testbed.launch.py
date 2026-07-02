"""Launch the AMR testbed: gz sim + ros_gz_bridge + robot spawn + RSP."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("se_mppi_utrap")
    world = os.path.join(pkg_share, "worlds", "testbed.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "amr.urdf.xacro")
    bridge_yaml = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")

    robot_desc = xacro.process_file(xacro_file).toxml()

    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "amr",
            "-string", robot_desc,
            "-x", "-3.0",
            "-y", "0.0",
            "-z", "0.01",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([gz_sim, rsp, spawn, bridge])
