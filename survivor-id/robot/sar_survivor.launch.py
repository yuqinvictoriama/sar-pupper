"""Robot stack with projectile dodging AND survivor identification.

    ros2 launch robot/sar_survivor.launch.py                  # everything in dry run: watches, locks, moves nothing
    ros2 launch robot/sar_survivor.launch.py motion:=true     # Pupper will walk to a pinned survivor

It includes the dodging component's launch file (the course controllers, camera, joystick,
gemini_walk and the trick controllers, the velocity mux, the dodge node) with the person
follower left out, because the pose model needs the Hailo and the Hailo serves one program.
Then it adds three nodes:

    survivor_node.py   cameras -> people -> the arms-up signal -> the pin     (owns the Hailo)
    approach_node.py   the pin -> /survivor_cmd_vel, arrival, bark, bounce
    event_bridge.py    /survivor/event and /dodge/event -> Gemini

The mux order is in projectile-dodging/robot/mux.yaml:
joystick > dodge > terrain > SURVIVOR > Gemini > navigation > follower.

So run this *instead of* sar_dodge.launch.py or gemini_pupper.launch.py (robot/start.sh does).

Arguments:
    motion:=false       let the approach node move the robot (false = dry run)
    armed:=false        let the dodge node move the robot
    drive:=direct       direct = walk straight at the survivor; route = route mapping drives until 1.5 m
    enabled:=true       start watching for the signal straight away
    dodge:=true         false = no dodge node; the survivor node then opens the rear cameras itself
    elp:=''             topic | v4l2 | off (default: topic with the dodge node, v4l2 without it)
    record:=''          folder for the survivor node to save the frames it looks at
    config:=''          another survivor.yaml
    gemini_dir:=~/gemini_pupper
    dodge_dir:=<repo>/projectile-dodging
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def truthy(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def setup(context):
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    dodge_dir = Path(os.path.expanduser(arg("dodge_dir")))
    dodge_launch = dodge_dir / "robot" / "sar_dodge.launch.py"
    if not dodge_launch.exists():
        return [LogInfo(msg="[sar_survivor] %s not found: survivor identification is started through the "
                            "dodging component's launch file. Set dodge_dir:=..." % dodge_launch)]
    dodge = truthy(arg("dodge"))
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(dodge_launch)),
        launch_arguments={"armed": arg("armed"), "gemini_dir": arg("gemini_dir"), "follower": "false",
                          "dodge": "true" if dodge else "false"}.items())]

    env = dict(os.environ, SAR_DODGE_DIR=str(dodge_dir))
    config = arg("config")
    extra = ["-p", "config:=" + config] if config else []
    elp = arg("elp") or ("topic" if dodge else "v4l2")

    def node(script: str, *params: str):
        cmd = ["python3", "-u", str(HERE / script), "--ros-args"]
        for p in params:
            cmd += ["-p", p]
        return ExecuteProcess(cmd=cmd + extra, cwd=str(ROOT), env=env, output="both", respawn=True, respawn_delay=3.0)

    survivor = ["enabled:=" + ("true" if truthy(arg("enabled")) else "false"), "elp:=" + elp]
    if arg("record"):
        survivor.append("record:=" + arg("record"))
    approach = ["motion:=" + ("true" if truthy(arg("motion")) else "false")]
    if arg("drive"):
        approach.append("drive:=" + arg("drive"))
    actions += [node("survivor_node.py", *survivor), node("approach_node.py", *approach), node("event_bridge.py")]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motion", default_value="false",
                              description="true lets Pupper walk to a pinned survivor; false is a dry run"),
        DeclareLaunchArgument("armed", default_value="false", description="true lets the dodge node move the robot"),
        DeclareLaunchArgument("drive", default_value=""),
        DeclareLaunchArgument("enabled", default_value="true"),
        DeclareLaunchArgument("dodge", default_value="true"),
        DeclareLaunchArgument("elp", default_value=""),
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("gemini_dir", default_value=os.environ.get("GEMINI_PUPPER_DIR", "~/gemini_pupper")),
        DeclareLaunchArgument("dodge_dir", default_value=os.environ.get("SAR_DODGE_DIR", str(ROOT.parent / "projectile-dodging"))),
        OpaqueFunction(function=setup),
    ])
