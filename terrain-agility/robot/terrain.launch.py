"""Terrain agility, added to a robot stack that is already running (or being launched).

    ros2 launch robot/terrain.launch.py                    # everything
    ros2 launch robot/terrain.launch.py avoid:=false       # only the step-over side (bring-up tests)
    ros2 launch robot/terrain.launch.py config:=/path/to/terrain.yaml

Starts, as plain Python processes (no colcon build needed):
  spawn_controllers.py   loads the three terrain forward command controllers, inactive
  estop_guard.py         relaxes those controllers on the gamepad e-stop / if the step-over node dies
  stepover_node.py       plans and plays crossings on request
  terrain_node.py        camera -> obstacles -> step over / go around  (avoid:=true)

The velocity mux must list /terrain_cmd_vel for the node's steering to reach the legs:
see robot/terrain_mux.yaml and the README ("Wiring into the mux").

To include it from your own launch file (e.g. sar-pupper's sar.launch.py):

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    IncludeLaunchDescription(PythonLaunchDescriptionSource("<repo>/terrain-agility/robot/terrain.launch.py"))
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = str(HERE.parent / "config" / "terrain.yaml")


def generate_launch_description():
    config = LaunchConfiguration("config")

    def py(script, *args, **kw):
        return ExecuteProcess(cmd=["python3", "-u", str(HERE / script), *args], output="both", **kw)

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=DEFAULT_CONFIG, description="terrain.yaml to use"),
        DeclareLaunchArgument("avoid", default_value="true",
                              description="run the camera/steering node (false: step-over node only)"),
        py("spawn_controllers.py"),
        py("estop_guard.py", "--config", config, respawn=True, respawn_delay=1.0),
        py("stepover_node.py", "--config", config),
        py("terrain_node.py", "--config", config, condition=IfCondition(LaunchConfiguration("avoid")),
           respawn=True, respawn_delay=3.0),
    ])
