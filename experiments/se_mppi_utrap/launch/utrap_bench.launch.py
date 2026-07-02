"""U-trap benchmark launch: gz testbed world + bridge + robot + Nav2.

Unlike ``testbed.launch.py`` (sim only, for teleop), this file wires the full
SE-MPPI benchmark stack so the L11 harness (`experiments/runner/ros_launcher.py`)
can drive it exactly as it drives the tb3_sandbox path:

    gz sim (testbed.sdf)  +  ros_gz_bridge  +  robot_state_publisher  +  spawn
    +  nav2_bringup/bringup_launch.py  (map_server + AMCL + Nav2, use_sim_time)

The U-trap walls live in the *world* (sensed by lidar) but NOT in ``map`` (the
localization map is an empty box), so NavFn plans a straight line through the
pocket and only the local controller's escape logic differentiates stock MPPI
from SE-MPPI — the honest local-minimum the paper's headline scenario needs.

Arguments (all the harness needs; sensible defaults for manual use):
    map           full path to the localization map yaml (empty-box utrap_loc)
    params_file   full Nav2 params yaml (the resolved ablation overlay)
    x_pose/y_pose/yaw   robot spawn (from the scenario start)
    headless      True => gz server only (no GUI); default True
    use_rviz      accepted for parity with tb3_simulation_launch (optional RViz)
    use_sim_time  default True (Gazebo clock)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def _setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("se_mppi_utrap")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world").perform(context) or os.path.join(
        pkg_share, "worlds", "testbed.sdf")
    default_map = os.path.join(pkg_share, "maps", "utrap_loc.yaml")
    map_yaml = LaunchConfiguration("map").perform(context) or default_map
    params_file = LaunchConfiguration("params_file").perform(context)
    x_pose = LaunchConfiguration("x_pose").perform(context)
    y_pose = LaunchConfiguration("y_pose").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = os.path.join(pkg_share, "urdf", "amr.urdf.xacro")
    bridge_yaml = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
    robot_desc = xacro.process_file(xacro_file).toxml()

    # gz server (always) — mirrors nav2's tb3 invocation `gz sim -r -s <world>`.
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v1", world], output="screen")
    # gz GUI only when not headless.
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        condition=UnlessCondition(headless),
        launch_arguments={"gz_args": "-g -v1"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc,
                     "use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "amr", "-string", robot_desc,
                   "-x", x_pose, "-y", y_pose, "-z", "0.01", "-Y", yaw],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": use_sim_time}],
        output="screen",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "True",
        }.items(),
    )

    return [gz_server, gz_gui, rsp, spawn, bridge, nav2]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("se_mppi_utrap")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="",
                              description="gz world sdf (default: testbed.sdf)"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(pkg_share, "maps",
                                              "utrap_loc.yaml"),
            description="localization map yaml (empty box for the U-trap)"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "..", "..", "sim",
                                       "nav2_se_loopback.yaml"),
            description="full Nav2 params yaml (resolved ablation overlay)"),
        DeclareLaunchArgument("x_pose", default_value="-3.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("headless", default_value="True"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        OpaqueFunction(function=_setup),
    ])
