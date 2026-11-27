"""The robot stack with terrain agility and nothing else: for bring-up and testing without
Gemini. (With Gemini: start gemini_pupper as usual, give its mux the inputs list from
robot/terrain_mux.yaml, and launch robot/terrain.launch.py next to it. See the README.)

    ros2 launch robot/standalone.launch.py
    ros2 launch robot/standalone.launch.py avoid:=false     # step-over bring-up tests only

It is the Gemini launch's pattern: include the course's neural_controller launch (walking
policies, camera, joystick, e-stop) as pupper_gait_deploy installs it, route the joystick
through cmd_vel_mux, fix the camera's white balance, and add robot/terrain.launch.py.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config" / "terrain.yaml"
COURSE_CONFIG = PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "config.yaml"])


def camera_white_balance():
    """Fixed white balance on the course's camera node, set once the node is up (the same
    libcamera controls Lab 5 and the Gemini launch use)."""
    wb = (yaml.safe_load(CONFIG.read_text()) or {}).get("camera", {}).get("white_balance")
    if not wb:
        return []
    red, blue = float(wb[0]), float(wb[1])
    script = ("until ros2 param set /camera AwbEnable false; do sleep 2; done; "
              f"ros2 param set /camera ColourGains '[{red}, {blue}]'")
    return [ExecuteProcess(cmd=["bash", "-c", script], output="both")]


def generate_launch_description():
    course = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "launch.py"])),
        launch_arguments={"teleop": "False"}.items(),      # re-added below, routed through the mux
    )
    terrain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(HERE / "terrain.launch.py")),
        launch_arguments={"avoid": LaunchConfiguration("avoid"), "config": LaunchConfiguration("config")}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("avoid", default_value="true"),
        DeclareLaunchArgument("config", default_value=str(CONFIG)),
        course,
        Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
             parameters=[COURSE_CONFIG], remappings=[("cmd_vel", "teleop_cmd_vel")], output="both"),
        Node(package="cmd_vel_mux", executable="cmd_vel_mux_node", parameters=[str(HERE / "terrain_mux.yaml")],
             output="both"),
        *camera_white_balance(),
        terrain,
    ])
