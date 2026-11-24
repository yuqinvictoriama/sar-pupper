#!/bin/bash
# Start the robot stack with projectile dodging and survivor identification.
#
#   robot/start.sh                 everything in DRY RUN: it watches, locks and reports, and moves nothing
#   robot/start.sh --motion        Pupper may walk to a pinned survivor   (someone at the e-stop!)
#   robot/start.sh --armed         the dodge node may move the robot      (someone at the e-stop!)
#   robot/start.sh --gemini        also start the mission robot server, so Gemini announces events
#   robot/start.sh --kiosk         ... and the face on Pupper's own screen
#   robot/start.sh --no-dodge      no dodge node; the survivor node opens the rear cameras itself
#   robot/start.sh --record DIR    save every frame the survivor node looks at (for tools/replay.py)
#   robot/start.sh --drive route   leave the driving to route mapping until 1.5 m from the survivor
#   robot/start.sh --not-watching  start with the pose model off (turn on: robot/watch.sh enable)
#
# Stop everything with robot/stop.sh. Logs: /tmp/sar-survivor/robot.log
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$(dirname "$HERE")")
GEMINI_DIR=${GEMINI_PUPPER_DIR:-$HOME/gemini_pupper}
DODGE_DIR=${SAR_DODGE_DIR:-$REPO/projectile-dodging}
LOG=/tmp/sar-survivor; mkdir -p $LOG /tmp/gemini-pupper
MOTION=false; ARMED=false; GEMINI=0; KIOSK=""; DODGE=true; RECORD=""; DRIVE=""; ENABLED=true
while [ $# -gt 0 ]; do
  case "$1" in
    --motion) MOTION=true ;;
    --armed) ARMED=true ;;
    --gemini) GEMINI=1 ;;
    --kiosk) GEMINI=1; KIOSK="--kiosk" ;;
    --no-dodge) DODGE=false ;;
    --not-watching) ENABLED=false ;;
    --record) shift; RECORD=${1:-data/recordings} ;;
    --drive) shift; DRIVE=${1:-direct} ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
  shift
done
ROS="source /opt/ros/jazzy/setup.bash; source $HOME/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1"

[ -f "$DODGE_DIR/robot/sar_dodge.launch.py" ] || { echo "projectile-dodging not found at $DODGE_DIR (set SAR_DODGE_DIR)"; exit 1; }
grep -q "/survivor_cmd_vel" "$DODGE_DIR/robot/mux.yaml" || {
  echo "$DODGE_DIR/robot/mux.yaml has no /survivor_cmd_vel input: the approach commands would go nowhere. Update projectile-dodging."; exit 1; }
MODEL=$(python3 -c "import sys, yaml, os; c = yaml.safe_load(open(sys.argv[1])); print(c['detector']['model'])" "$HERE/../config/survivor.yaml")
[ -f "$HERE/../$MODEL" ] || [ -f "$MODEL" ] || { echo "Missing $MODEL: run  bash tools/download_model.sh  first"; exit 1; }
if pgrep -f "[r]os2_control_node" >/dev/null; then
  echo "A robot stack is already running. Stop it first (robot/stop.sh): the mux has to be started with the survivor input,"
  echo "and the person follower must not be holding the Hailo."
  exit 1
fi
if pgrep -f "[f]ollow_service.py|[h]ailo_detection" >/dev/null; then
  echo "Something else is using the Hailo (follow_service or hailo_detection). Stop it first."; exit 1
fi
[ $MOTION = true ] && echo "MOTION ON: Pupper will walk to anyone who holds both arms up. Someone at the e-stop, clear floor."
echo "[1/2] starting robot stack + dodge ($DODGE, armed: $ARMED) + survivor ID (motion: $MOTION); log $LOG/robot.log"
rm -f /tmp/gemini-pupper/controllers_ready
setsid nohup bash -c "$ROS; exec ros2 launch $HERE/sar_survivor.launch.py motion:=$MOTION armed:=$ARMED dodge:=$DODGE enabled:=$ENABLED drive:=$DRIVE record:=$RECORD gemini_dir:=$GEMINI_DIR dodge_dir:=$DODGE_DIR" \
  > $LOG/robot.log 2>&1 < /dev/null &
for i in $(seq 1 240); do [ -f /tmp/gemini-pupper/controllers_ready ] && break; sleep 1; done
if [ -f /tmp/gemini-pupper/controllers_ready ]; then echo "      $(cat /tmp/gemini-pupper/controllers_ready)"
else echo "      controllers still loading after 4 min; see $LOG/robot.log"; fi
for what in "survivor node up" "approach node up"; do
  if grep -q "$what" $LOG/robot.log; then echo "      $(grep -m1 "$what" $LOG/robot.log | sed "s/.*$what/$what/")"
  else echo "      '$what' not seen yet; see $LOG/robot.log"; fi
done
grep -m1 "cannot start the pose model" $LOG/robot.log | sed 's/.*cannot start/      PROBLEM: cannot start/'

if [ $GEMINI = 1 ]; then
  echo "[2/2] starting the mission robot server (it carries mission events to Gemini)"
  GEMINI_PUPPER_DIR="$GEMINI_DIR" bash "$REPO/personality/start.sh" $KIOSK
else
  echo "[2/2] done. Watch it with:  bash robot/watch.sh    Stop with:  bash robot/stop.sh"
  echo "      (no robot server: no bounce and no Gemini announcements. Add --gemini for those.)"
fi
