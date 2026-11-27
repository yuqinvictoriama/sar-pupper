"""Robot stack with the dodge reflex.

    ros2 launch robot/sar_dodge.launch.py                 # dodge node in dry run
    ros2 launch robot/sar_dodge.launch.py armed:=true     # the robot will move to dodge

It starts what Gemini Pupper's launch file starts (the course controllers,
camera, joystick, the gemini_walk controller and trick controllers, the person
follower), with one change: the velocity mux gets /dodge_cmd_vel as an input,
below the joystick and above everything else (robot/mux.yaml). Then it adds
the dodge node.

So run this *instead of* gemini_pupper.launch.py. Gemini's own
robot/start.sh notices the robot stack is already up and only starts its
server (robot/start.sh here does both for you).

Arguments:
    armed:=false          let the dodge node move the robot
    gemini_dir:=~/gemini_pupper
    follower:=false       start the person follower too (it also reads and decodes the fisheye pictures)
    fast_mux:=true        robot/fast_mux.py (passes a command on at once); false = the monorepo mux (50 ms tick)
    fisheye_fps:=0        ask the fisheye for this frame rate (0 = leave it alone)
    fisheye_jpeg_quality:=85  JPEG quality of the fisheye pictures (the camera node's own is 95; 0 = leave it alone)
    fisheye_exposure_us:=0  fix the fisheye exposure (0 = automatic)
    config:=''            another dodge.yaml
    dodge:=true           false = everything except the dodge node (frees the USB cameras for the tools)
"""
import importlib.util
import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MUX_PARAMS = str(HERE / "mux.yaml")
COURSE_CONFIG = PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "config.yaml"])


def truthy(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def load_gemini_launch(gemini_dir: Path):
    """Gemini Pupper's launch file as a module, so its controller and follower setup is reused, not copied."""
    path = gemini_dir / "robot" / "launch" / "gemini_pupper.launch.py"
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location("gemini_pupper_launch", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fisheye_settings(context):
    """Fixed white balance (the lab's values), and optionally frame rate and exposure, on the course camera node."""
    cfg_path = LaunchConfiguration("config").perform(context) or str(ROOT / "config" / "dodge.yaml")
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    gains = cfg.get("cameras", {}).get("fisheye", {}).get("colour_gains", [1.4, 3.3])
    cmds = ["until ros2 param set /camera AwbEnable false; do sleep 2; done",
            "ros2 param set /camera ColourGains '[%s, %s]'" % (float(gains[0]), float(gains[1]))]
    fps = float(LaunchConfiguration("fisheye_fps").perform(context) or 0)
    if fps > 0:
        us = int(round(1e6 / fps))
        cmds.append("ros2 param set /camera FrameDurationLimits '[%d, %d]'" % (us, us))
    quality = int(float(LaunchConfiguration("fisheye_jpeg_quality").perform(context) or 0))
    if quality > 0:
        # Smaller pictures: less to encode in the camera node, to send, and to decode in the dodge node.
        cmds.append("ros2 param set /camera jpeg_quality %d" % max(40, min(100, quality)))
    exposure = int(float(LaunchConfiguration("fisheye_exposure_us").perform(context) or 0))
    if exposure > 0:
        cmds.append("ros2 param set /camera AeEnable false")
        cmds.append("ros2 param set /camera ExposureTime %d" % exposure)
    return [ExecuteProcess(cmd=["bash", "-c", "; ".join(cmds)], output="both")]


def setup(context):
    actions = []
    gemini_dir = Path(os.path.expanduser(LaunchConfiguration("gemini_dir").perform(context)))
    gemini = load_gemini_launch(gemini_dir)

    # The course stack: controllers, camera, joystick e-stop. Its teleop node is
    # re-added below so the joystick goes through the mux.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("neural_controller"), "launch", "launch.py"])),
        launch_arguments={"teleop": "False"}.items()))
    actions.append(Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
                        parameters=[COURSE_CONFIG], remappings=[("cmd_vel", "teleop_cmd_vel")], output="both"))

    if truthy(LaunchConfiguration("fast_mux").perform(context)):
        actions.append(ExecuteProcess(
            cmd=["python3", "-u", str(HERE / "fast_mux.py"), "--ros-args", "--params-file", MUX_PARAMS],
            output="both", respawn=True, respawn_delay=1.0))
    else:
        actions.append(Node(package="cmd_vel_mux", executable="cmd_vel_mux_node", parameters=[MUX_PARAMS], output="both"))

    actions += fisheye_settings(context)

    if gemini is None:
        actions.append(LogInfo(msg="[sar_dodge] %s not found: starting without gemini_walk, trick controllers and "
                                   "the person follower. Dodging then needs a course controller (X button), which "
                                   "caps speed at 0.49 m/s." % gemini_dir))
    else:
        actions.append(Node(package="topic_tools", executable="throttle", name="joint_state_throttler",
                            parameters=[gemini.PARAMS], arguments=["messages"], output="both"))
        if truthy(LaunchConfiguration("follower").perform(context)):
            actions += gemini.person_follower()
        actions += gemini.gemini_controllers()

    if not truthy(LaunchConfiguration("dodge").perform(context)):
        actions.append(LogInfo(msg="[sar_dodge] dodge:=false: dodge node not started"))
        return actions
    config = LaunchConfiguration("config").perform(context)
    armed = "true" if truthy(LaunchConfiguration("armed").perform(context)) else "false"
    cmd = ["python3", "-u", str(HERE / "dodge_node.py"), "--ros-args", "-p", "armed:=" + armed]
    if config:
        cmd += ["-p", "config:=" + config]
    actions.append(ExecuteProcess(cmd=cmd, cwd=str(ROOT), output="both", respawn=True, respawn_delay=3.0))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("armed", default_value="false",
                              description="true lets the dodge node move the robot; false is a dry run"),
        DeclareLaunchArgument("gemini_dir", default_value=os.environ.get("GEMINI_PUPPER_DIR", "~/gemini_pupper")),
        DeclareLaunchArgument("follower", default_value="false"),
        DeclareLaunchArgument("fast_mux", default_value="true"),
        DeclareLaunchArgument("fisheye_jpeg_quality", default_value="85"),
        DeclareLaunchArgument("fisheye_fps", default_value="0"),
        DeclareLaunchArgument("fisheye_exposure_us", default_value="0"),
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("dodge", default_value="true"),
        OpaqueFunction(function=setup),
    ])
