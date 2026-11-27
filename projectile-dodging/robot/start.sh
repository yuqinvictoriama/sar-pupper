#!/bin/bash
# Start the robot stack with the dodge reflex.
#
#   robot/start.sh                  robot stack + dodge node in DRY RUN (reports, never moves)
#   robot/start.sh --armed          the dodge node may move the robot (someone at the e-stop!)
#   robot/start.sh --gemini         also start Gemini Pupper's server (add --kiosk for the on-robot app)
#   robot/start.sh --follower       also start the person follower (off by default: the demo does not use it, and it
#                                   reads and decodes the fisheye pictures too, which takes processor time from dodging)
#   robot/start.sh --slow-mux       use the monorepo's velocity mux instead of robot/fast_mux.py. The monorepo's passes a
#                                   dodge command on at its next 50 ms tick (40-43 ms measured, 2026-11-26); ours at once
#   robot/start.sh --fisheye-fps 60 ask the fisheye camera for 60 frames a second
#   robot/start.sh --fisheye-quality 85   JPEG quality of the fisheye pictures (default 85; the camera node's own is 95,
#                                   which makes pictures twice the size to send and decode)
#   robot/start.sh --fisheye-exposure 4000   fix the fisheye exposure at 4000 microseconds (default: automatic)
#   robot/start.sh --no-dodge       robot stack only, USB cameras left free (for calibrating and recording)
#   robot/start.sh --no-view        do not start Pupper Dodging View (the browser page of saved throws, port 8077)
#
# Stop everything with robot/stop.sh. Logs: /tmp/sar-dodge/robot.log (and dodging_view.log for Pupper Dodging View)
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
GEMINI_DIR=${GEMINI_PUPPER_DIR:-$HOME/gemini_pupper}
LOG=/tmp/sar-dodge; mkdir -p $LOG /tmp/gemini-pupper
ARMED=false; GEMINI=0; KIOSK=""; FOLLOWER=false; FASTMUX=true; FPS=0; EXPOSURE=0; DODGE=true; VIEW=true; QUALITY=85
while [ $# -gt 0 ]; do
  case "$1" in
    --armed) ARMED=true ;;
    --gemini) GEMINI=1 ;;
    --kiosk) GEMINI=1; KIOSK="--kiosk" ;;
    --follower) FOLLOWER=true ;;
    --no-follower) FOLLOWER=false ;;
    --fast-mux) FASTMUX=true ;;
    --slow-mux) FASTMUX=false ;;
    --fisheye-quality) shift; QUALITY=${1:-85} ;;
    --no-dodge) DODGE=false ;;
    --no-view) VIEW=false ;;
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
# Other things that take processor time from the cameras. They are not stopped here: say so, and how.
for unit in llm-agent; do
  if systemctl is-active --quiet $unit 2>/dev/null; then
    echo "note: the '$unit' service is running and uses the processor and the camera pictures too."
    echo "      For dodging, stop it first:  sudo systemctl stop $unit"
  fi
done
# The motors find their zero by driving every leg joint against its end stop, the moment the stack starts.
# A leg that is held back (by the floor, a mat, a cable, the robot lying on its side) or a battery too low
# for the motors to keep up leaves that leg's zero wrong, and the robot then cannot stand evenly.
python3 "$HERE/homing_report.py" --battery-only
echo "HOMING: the legs are about to move to their end stops. Hold the robot off the floor by its body, or put it"
echo "        on its stand, so that no leg touches anything, until they stop moving (about 10 s)."
echo "[1/2] starting robot stack (dodge node: $DODGE, armed: $ARMED); log $LOG/robot.log"
rm -f /tmp/gemini-pupper/controllers_ready
setsid nohup bash -c "$ROS; exec ros2 launch $HERE/sar_dodge.launch.py armed:=$ARMED gemini_dir:=$GEMINI_DIR follower:=$FOLLOWER fast_mux:=$FASTMUX fisheye_fps:=$FPS fisheye_exposure_us:=$EXPOSURE fisheye_jpeg_quality:=$QUALITY dodge:=$DODGE" \
  > $LOG/robot.log 2>&1 < /dev/null &
if [ -d "$GEMINI_DIR" ]; then
  for i in $(seq 1 240); do [ -f /tmp/gemini-pupper/controllers_ready ] && break; sleep 1; done
  if [ -f /tmp/gemini-pupper/controllers_ready ]; then echo "      $(cat /tmp/gemini-pupper/controllers_ready)"
  else echo "      controllers still loading after 4 min; see $LOG/robot.log"; fi
else
  sleep 20
fi
python3 "$HERE/homing_report.py" --log $LOG/robot.log | sed 's/^/      /'
echo "      Look at the legs now: all four should be folded the same way. If one side sits further forward, home again"
echo "      (robot/stop.sh, then robot/start.sh with the legs free): it cannot be corrected while running."
if [ $DODGE = true ]; then
  if grep -q "dodge node up" $LOG/robot.log; then echo "      $(grep -m1 'dodge node up' $LOG/robot.log | sed 's/.*dodge node up/dodge node up/')"
  else echo "      dodge node has not reported yet; see $LOG/robot.log"; fi
fi

if [ $VIEW = true ] && ! pgrep -f "[t]ools/dodging_view.py" >/dev/null; then
  setsid nohup bash -c "$ROS; exec python3 -u $HERE/../tools/dodging_view.py" > $LOG/dodging_view.log 2>&1 < /dev/null &
  sleep 2
  echo "      Pupper Dodging View: open  http://$(hostname).local:8077  in a browser (saved throws, videos, timings)"
fi

if [ $GEMINI = 1 ]; then
  echo "[2/2] starting Gemini Pupper's server"
  "$GEMINI_DIR/robot/start.sh" $KIOSK
else
  echo "[2/2] done. Watch it with:  bash robot/watch.sh    Stop with:  bash robot/stop.sh"
fi
