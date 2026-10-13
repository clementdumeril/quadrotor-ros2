"""Webots + the whole control stack.

    ros2 launch quad_webots sim.launch.py            # simulator, driver, controller, mission
    ros2 launch quad_webots sim.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    share = get_package_share_directory("quad_webots")
    control_cfg = os.path.join(get_package_share_directory("quad_control"), "config", "position_controller.yaml")

    webots = WebotsLauncher(world=os.path.join(share, "worlds", "arena.wbt"))
    driver = WebotsController(
        robot_name="mavic",
        parameters=[
            {"robot_description": os.path.join(share, "resource", "mavic.urdf")},
            os.path.join(share, "config", "mavic.yaml"),
        ],
        respawn=True,
    )
    controller = Node(package="quad_control", executable="position_controller",
                      parameters=[control_cfg], output="screen")
    mission = Node(package="quad_mission", executable="mission",
                   parameters=[{"use_sim_time": True}], output="screen")
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "config", "quad.rviz")],
                parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        webots,
        driver,
        controller,
        mission,
        rviz,
        # Closing the Webots window stops everything.
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])),
    ])
