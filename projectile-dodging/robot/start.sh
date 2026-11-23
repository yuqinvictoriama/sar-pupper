#!/bin/bash
# Start the robot stack with the dodge reflex.
#
#   robot/start.sh                  robot stack + dodge node in DRY RUN (reports, never moves)
#   robot/start.sh --armed          the dodge node may move the robot (someone at the e-stop!)
#   robot/start.sh --gemini         also start Gemini Pupper's server (add --kiosk for the on-robot app)
#   robot/start.sh --no-follower    do not start the person follower (leaves the Hailo free)
#   robot/start.sh --fast-mux       use robot/fast_mux.py instead of the monorepo mux
#   robot/start.sh --fisheye-fps 60 ask the fisheye camera for 60 frames a second
#   robot/start.sh --fisheye-exposure 4000   fix the fisheye exposure at 4000 microseconds (default: automatic)
#   robot/start.sh --no-dodge       robot stack only, USB cameras left free (for calibrating and recording)
#
# Stop everything with robot/stop.sh. Logs: /tmp/sar-dodge/robot.log
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
GEMINI_DIR=${GEMINI_PUPPER_DIR:-$HOME/gemini_pupper}
LOG=/tmp/sar-dodge; mkdir -p $LOG /tmp/gemini-pupper
ARMED=false; GEMINI=0; KIOSK=""; FOLLOWER=true; FASTMUX=false; FPS=0; EXPOSURE=0; DODGE=true
while [ $# -gt 0 ]; do
  case "$1" in
    --armed) ARMED=true ;;
    --gemini) GEMINI=1 ;;
    --kiosk) GEMINI=1; KIOSK="--kiosk" ;;
    --no-follower) FOLLOWER=false ;;
    --fast-mux) FASTMUX=true ;;
    --no-dodge) DODGE=false ;;
    --fisheye-fps) shift; FPS=${1:-0} ;;
    --fisheye-exposure) shift; EXPOSURE=${1:-0} ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
  shift
done
# The same ROS environment Gemini Pupper's start.sh uses, so the two can talk.
ROS="source /opt/ros/jazzy/setup.bash; source $HOME/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1"

if pgrep -f "[r]os2_control_node" >/dev/null; then
  echo "A robot stack is already running. Stop it first (robot/stop.sh), or the mux will not know about the dodge node."
  exit 1
fi
echo "[1/2] starting robot stack (dodge node: $DODGE, armed: $ARMED); log $LOG/robot.log"
rm -f /tmp/gemini-pupper/controllers_ready
setsid nohup bash -c "$ROS; exec ros2 launch $HERE/sar_dodge.launch.py armed:=$ARMED gemini_dir:=$GEMINI_DIR follower:=$FOLLOWER fast_mux:=$FASTMUX fisheye_fps:=$FPS fisheye_exposure_us:=$EXPOSURE dodge:=$DODGE" \
  > $LOG/robot.log 2>&1 < /dev/null &
if [ -d "$GEMINI_DIR" ]; then
  for i in $(seq 1 240); do [ -f /tmp/gemini-pupper/controllers_ready ] && break; sleep 1; done
  if [ -f /tmp/gemini-pupper/controllers_ready ]; then echo "      $(cat /tmp/gemini-pupper/controllers_ready)"
  else echo "      controllers still loading after 4 min; see $LOG/robot.log"; fi
else
  sleep 20
fi
if [ $DODGE = true ]; then
  if grep -q "dodge node up" $LOG/robot.log; then echo "      $(grep -m1 'dodge node up' $LOG/robot.log | sed 's/.*dodge node up/dodge node up/')"
  else echo "      dodge node has not reported yet; see $LOG/robot.log"; fi
fi

if [ $GEMINI = 1 ]; then
  echo "[2/2] starting Gemini Pupper's server"
  "$GEMINI_DIR/robot/start.sh" $KIOSK
else
  echo "[2/2] done. Watch it with:  bash robot/watch.sh    Stop with:  bash robot/stop.sh"
fi
