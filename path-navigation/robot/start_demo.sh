#!/bin/bash
# Start everything the demo needs, on the Pupper. One command, then one click in Mission Control.
#
#   robot/start_demo.sh                 everything; NOTHING may move the robot until you allow it in Mission Control
#   robot/start_demo.sh --motion        navigation may walk as soon as a run is started (someone at the e-stop!)
#   robot/start_demo.sh --no-voice      no Gemini, no kiosk (Pupper's lines are only shown in Mission Control)
#   robot/start_demo.sh --no-kiosk      Gemini's server without the face on Pupper's screen
#   robot/start_demo.sh --no-keyword    no phrase detector (the operator's button stands in)
#   robot/start_demo.sh --no-terrain    no terrain agility (open-floor tests)
#   robot/start_demo.sh --no-dodge      no dodge node
#   robot/start_demo.sh --no-sound      do not start the sound module (bearings from Mission Control's compass)
#   robot/start_demo.sh --sound "<cmd>" start another sound module with this shell command instead
#   (by default robot/sound_node.py runs the partner's sound-localization/localize_angle.py, unchanged)
#
# Then open Mission Control from the laptop:  http://<pupper>.local:8078
# Stop everything with robot/stop_demo.sh. Logs are in /tmp/sar-demo/.
#
# Pupper on the floor on the start mark, clear space around it, the operator holding the gamepad.
# Not run on the robot yet: the first time, read /tmp/sar-demo/robot.log as it comes up.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$(dirname "$HERE")")
GEMINI_DIR=${GEMINI_PUPPER_DIR:-$HOME/gemini_pupper}
LOG=/tmp/sar-demo; mkdir -p $LOG /tmp/gemini-pupper
MOTION=false; VOICE=1; KIOSK="--kiosk"; KEYWORD=true; TERRAIN=true; DODGE=true; SOUND=${PUPPER_SOUND_CMD:-dji}
while [ $# -gt 0 ]; do
  case "$1" in
    --motion) MOTION=true ;;
    --no-voice) VOICE=0 ;;
    --no-kiosk) KIOSK="" ;;
    --no-keyword) KEYWORD=false ;;
    --no-terrain) TERRAIN=false ;;
    --no-dodge) DODGE=false ;;
    --sound) shift; SOUND=${1:-} ;;
    --no-sound) SOUND=none ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
  shift
done
ROS="source /opt/ros/jazzy/setup.bash; source $HOME/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1"

if pgrep -f "[r]os2_control_node" >/dev/null; then
  echo "A robot stack is already running. Stop it first (robot/stop_demo.sh): the demo needs its own mux order."
  exit 1
fi

echo "[1/4] robot stack, dodging (not armed), terrain agility, phrase detector, navigation (motion: $MOTION)"
echo "      log $LOG/robot.log"
rm -f /tmp/gemini-pupper/controllers_ready
export PUPPER_SOUND_CMD="$SOUND"
setsid nohup bash -c "$ROS; exec ros2 launch $HERE/demo.launch.py motion:=$MOTION keyword:=$KEYWORD terrain:=$TERRAIN dodge:=$DODGE gemini_dir:=$GEMINI_DIR" \
  > $LOG/robot.log 2>&1 < /dev/null &
if [ -d "$GEMINI_DIR" ]; then
  for i in $(seq 1 240); do [ -f /tmp/gemini-pupper/controllers_ready ] && break; sleep 1; done
  if [ -f /tmp/gemini-pupper/controllers_ready ]; then echo "      $(cat /tmp/gemini-pupper/controllers_ready)"
  else echo "      controllers still loading after 4 min; see $LOG/robot.log"; fi
else
  sleep 20
fi
for name in "path navigation up" "dodge node up" "terrain agility ready" "step-over node ready" \
            "starting the sound module" "with 4 input channel"; do
  if grep -q "$name" $LOG/robot.log; then echo "      ok: $name"; else echo "      NOT YET: $name (see $LOG/robot.log)"; fi
done
case "$SOUND" in
  ""|none) echo "      the sound module was not started here (--no-sound): start it now, or use Mission Control's compass" ;;
  dji) grep -q "Four-channel localization needs\|Could not start four-channel input\|Could not uniquely select\|More than one input matches" $LOG/robot.log \
         && echo "      SOUND MODULE PROBLEM: $(grep -m1 "Four-channel localization needs\|Could not start four-channel input\|Could not uniquely select\|More than one input matches" $LOG/robot.log)" ;;
esac

if [ $VOICE = 1 ]; then
  echo "[2/4] Pupper's voice (Gemini, the mission prompt, the event bridge)"
  bash "$REPO/personality/start.sh" $KIOSK 2>&1 | sed 's/^/      /' \
    || echo "      the voice did not start; the run works without it (lines are shown in Mission Control)"
else
  echo "[2/4] no voice (--no-voice)"
fi

echo "[3/4] Mission Control and Pupper Dodging View"
pkill -f "[t]ools/mission_control.py" 2>/dev/null
setsid nohup bash -c "$ROS; exec python3 -u $HERE/../tools/mission_control.py --robot" > $LOG/mission_control.log 2>&1 < /dev/null &
if ! pgrep -f "[t]ools/dodging_view.py" >/dev/null && [ -f "$REPO/projectile-dodging/tools/dodging_view.py" ]; then
  setsid nohup bash -c "$ROS; exec python3 -u $REPO/projectile-dodging/tools/dodging_view.py" > $LOG/dodging_view.log 2>&1 < /dev/null &
fi
sleep 2

echo "[4/4] ready."
echo "      Mission Control:      http://$(hostname).local:8078      (the Start button is there)"
echo "      Pupper Dodging View:  http://$(hostname).local:8077"
echo "      Pupper is still lying down and nothing may move it. In Mission Control: Stand up, Allow motion, Start."
echo "      Stop everything:      bash $HERE/stop_demo.sh"
