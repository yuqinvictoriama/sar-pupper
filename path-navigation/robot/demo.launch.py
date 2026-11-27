"""The whole demo stack in one launch: robot, dodging, terrain agility, phrase detector, navigation.

    ros2 launch robot/demo.launch.py                      everything, nothing may move the robot
    ros2 launch robot/demo.launch.py motion:=true         navigation may walk (someone at the e-stop!)

robot/start_demo.sh runs this for you and then starts Pupper's voice and Mission Control.

What it starts
  projectile-dodging/robot/sar_dodge.launch.py   the course stack, gemini_walk, the velocity mux and the dodge node
                                                 (not armed: navigation arms it in the gauntlet). Its launch file is
                                                 used as it is, with one value changed: the mux reads
                                                 robot/demo_mux.yaml, which puts navigation above Gemini.
  terrain-agility/robot/terrain.launch.py        the step-over and walk-around nodes
  robot/keyword_node.py                          hears "You saved me, Pupper!"
  robot/nav_node.py                              the mission
  robot/sound_node.py                            the partner's sound-direction script (sound-localization/
                                                 localize_angle.py, unchanged) on /sound/bearing and /sound/status

Arguments
  motion:=false     true lets navigation publish velocity. Mission Control can switch it too
  keyword:=true     false: no phrase detector (the operator's button stands in)
  terrain:=true     false: no terrain agility (open floor tests)
  dodge:=true       false: no dodge node (frees the USB cameras)
  sound:=dji        dji: robot/sound_node.py runs the partner's script (config/demo.yaml, section dji).
                    none: nothing (send bearings with tools/fake_sound.py or Mission Control's compass).
                    Anything else is run as a shell command instead.
  gemini_dir:=~/gemini_pupper
  config:=''        another demo.yaml

Not run on the robot yet (written without ROS at hand): check each node comes up the first time.
"""
import importlib.util
import os
from pathlib import Path

from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REPO = ROOT.parent
DODGE_LAUNCH = REPO / "projectile-dodging" / "robot" / "sar_dodge.launch.py"
TERRAIN_LAUNCH = REPO / "terrain-agility" / "robot" / "terrain.launch.py"
DEMO_MUX = str(HERE / "demo_mux.yaml")


def dodge_stack():
    """sar_dodge.launch.py with the demo's mux order, or None if it is not there."""
    if not DODGE_LAUNCH.exists():
        return None
    spec = importlib.util.spec_from_file_location("sar_dodge_launch", str(DODGE_LAUNCH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.MUX_PARAMS = DEMO_MUX            # read when its setup() runs
    return module.generate_launch_description()


def sound_module(context):
    cmd = LaunchConfiguration("sound").perform(context).strip()
    if cmd.lower() in ("", "none", "off", "false"):
        return [LogInfo(msg="[demo] sound:=none : the sound module is not started here. Start it yourself, or send "
                            "bearings by hand with tools/fake_sound.py or Mission Control's compass.")]
    if cmd.lower() == "dji":
        args = ["--ros-args", "-p", "config:=" + LaunchConfiguration("config").perform(context)]
        return [ExecuteProcess(cmd=["python3", "-u", str(HERE / "sound_node.py"), *args], cwd=str(ROOT),
                               output="both", respawn=True, respawn_delay=3.0)]
    return [ExecuteProcess(cmd=["bash", "-c", cmd], output="both", respawn=True, respawn_delay=3.0)]


def generate_launch_description():
    config = LaunchConfiguration("config")

    def py(script, *args, **kw):
        return ExecuteProcess(cmd=["python3", "-u", str(HERE / script), *args], cwd=str(ROOT), output="both", **kw)

    actions = [
        DeclareLaunchArgument("motion", default_value="false", description="true lets navigation walk the robot"),
        DeclareLaunchArgument("keyword", default_value="true"),
        DeclareLaunchArgument("terrain", default_value="true"),
        DeclareLaunchArgument("dodge", default_value="true"),
        DeclareLaunchArgument("sound", default_value=os.environ.get("PUPPER_SOUND_CMD", "dji")),
        DeclareLaunchArgument("gemini_dir", default_value=os.environ.get("GEMINI_PUPPER_DIR", "~/gemini_pupper")),
        DeclareLaunchArgument("config", default_value=""),
    ]
    stack = dodge_stack()
    if stack is None:
        actions.append(LogInfo(msg="[demo] %s not found: no robot stack started from here" % DODGE_LAUNCH))
    else:
        actions.append(IncludeLaunchDescription(
            LaunchDescriptionSource(stack),
            launch_arguments={"armed": "false", "follower": "false", "dodge": LaunchConfiguration("dodge"),
                              "gemini_dir": LaunchConfiguration("gemini_dir")}.items()))
    if TERRAIN_LAUNCH.exists():
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(TERRAIN_LAUNCH)),
                                                condition=IfCondition(LaunchConfiguration("terrain"))))
    actions += [
        py("keyword_node.py", condition=IfCondition(LaunchConfiguration("keyword")), respawn=True, respawn_delay=3.0),
        py("nav_node.py", "--ros-args", "-p", ["motion:=", LaunchConfiguration("motion")], "-p", ["config:=", config],
           respawn=True, respawn_delay=2.0),
        OpaqueFunction(function=sound_module),
    ]
    return LaunchDescription(actions)
