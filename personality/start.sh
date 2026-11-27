#!/bin/bash
# Start Pupper with its search and rescue personality.
#   personality/start.sh              robot stack + robot server (open the app from a laptop through an SSH tunnel)
#   personality/start.sh --kiosk      also the face on Pupper's own screen, speaking in the mission voice
#   personality/start.sh --no-bridge  do not start the event bridge (something else posts the mission events)
# Stop everything with personality/stop.sh.
#
# This is the lab's start (gemini_pupper/robot/start.sh) with two swaps: the robot server
# serves the mission prompt and the mission-event feed, and the kiosk sets the mission voice.
# Then it starts the event bridge (it only listens and posts; it cannot move the robot) and
# runs the doctor, which says whether Pupper can talk and, if not, which link is broken.
# Pupper on the ground, clear space around it, a hand near the e-stop, as for the lab.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
GP=${GEMINI_PUPPER_DIR:-$(dirname "$(dirname "$HERE")")/gemini_pupper}
LOG=/tmp/gemini-pupper
KEY_FILE=${GEMINI_KEY_FILE:-$HOME/.config/gemini-pupper/api_key}
KIOSK=0; BRIDGE=1
for arg in "$@"; do
  case "$arg" in
    --kiosk) KIOSK=1 ;;
    --no-bridge) BRIDGE=0 ;;
    *) echo "unknown option $arg (--kiosk, --no-bridge)"; exit 1 ;;
  esac
done
ROS="source /opt/ros/jazzy/setup.bash; source $HOME/pupperv3-monorepo/ros2_ws/install/local_setup.bash; export ROS_LOCALHOST_ONLY=1"
# The server plays the robot's own voice and sets the volume; both need the user's sound
# server, which is found through this. It is not always set in a shell started over SSH.
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}

[ -f "$GP/robot/start.sh" ] || { echo "gemini_pupper not found at $GP (set GEMINI_PUPPER_DIR)"; exit 1; }
if [ $KIOSK = 1 ] && [ ! -s "$KEY_FILE" ]; then
  echo "Missing $KEY_FILE. Run $GP/robot/setup.sh first."; exit 1
fi
# Without this in the built app, Gemini is never told a mission event and Pupper says no
# call-out. Not fatal: Pupper still talks, and the robot's own voice says the call-outs.
if ! grep -qs "ws/mission" "$GP"/web/dist/assets/*.js; then
  echo "[mission] WARNING: the built web app does not listen for mission events, so Gemini will not hear them."
  if grep -qs "ws/mission" "$GP/web/services/geminiLive.ts"; then
    echo "          Its source does. Rebuild it:  bash $GP/robot/setup.sh"
  else
    echo "          web/services/geminiLive.ts here is the lab's original. Copy the changed one from the laptop"
    echo "          clone (it is not committed there), then:  bash $GP/robot/setup.sh"
  fi
fi

# 1. The lab's own start, without its kiosk: the robot stack (controllers loaded,
#    none active), then the lab's robot server.
bash "$GP/robot/start.sh" || exit 1

# 2. Swap that server for the same server with the mission prompt and the mission feed.
echo "[mission] restarting the robot server with the mission personality (log $LOG/server.log)"
pkill -f "[r]obot/server.py" 2>/dev/null
pkill -f "[p]ersonality/run.py server" 2>/dev/null
for i in $(seq 1 10); do curl -s -o /dev/null -m 1 localhost:8000/health || break; sleep 1; done
export GEMINI_PUPPER_DIR="$GP"
setsid nohup bash -c "$ROS; exec python3 -u $HERE/run.py server" > $LOG/server.log 2>&1 < /dev/null &
for i in $(seq 1 30); do curl -s -o /dev/null -m 1 localhost:8000/health && break; sleep 1; done
if ! curl -s -m 5 localhost:8000/tools | grep -q "search and rescue robot dog"; then
  echo "the mission server did not come up with the mission prompt; see $LOG/server.log"; exit 1
fi
if ! curl -s -m 5 localhost:8000/mission/recent | grep -q '"listeners"'; then
  echo "the mission server has no mission-event feed; see $LOG/server.log"; exit 1
fi
echo "[mission] Pupper is on duty: the server is serving personality/prompt.md and the mission feed"

# 3. The event bridge: ROS events (/nav/event, /dodge/event) -> the feed.
pkill -f "[p]ersonality/event_bridge.py" 2>/dev/null
if [ $BRIDGE = 1 ]; then
  setsid nohup bash -c "$ROS; exec python3 -u $HERE/event_bridge.py" > $LOG/event_bridge.log 2>&1 < /dev/null &
  echo "[mission] event bridge started (log $LOG/event_bridge.log)"
fi

if [ $KIOSK = 0 ]; then
  echo "ready. From a laptop:  ssh -N -L 8000:localhost:8000 pi@<pupper>  then open http://localhost:8000"
  echo "(pick the voice in Settings; run.py names the mission voice)"
  echo "Once the app is connected:  python3 $HERE/doctor.py --say"
  exit 0
fi

# 4. The kiosk. These lines are step 3 of gemini_pupper/robot/start.sh, with
#    run.py kiosk in place of kiosk.py. If the lab's start.sh changes, copy it again.
echo "[mission] opening the app on the robot's screen"
export WAYLAND_DISPLAY=wayland-0
sudo -n systemctl stop pupper-gui 2>/dev/null
pkill -f "[g]emini-pupper-kiosk" 2>/dev/null; pkill -f "[r]obot/kiosk.py" 2>/dev/null
pkill -f "[p]ersonality/run.py kiosk" 2>/dev/null; sleep 1
amixer -q sset Master 100% 2>/dev/null; wpctl set-volume @DEFAULT_SINK@ 1.0 2>/dev/null
setsid nohup chromium --ozone-platform=wayland --kiosk --noerrdialogs --disable-infobars --no-first-run \
  --password-store=basic --use-fake-ui-for-media-stream --autoplay-policy=no-user-gesture-required \
  --remote-debugging-port=9222 --user-data-dir=$HOME/.cache/gemini-pupper-kiosk http://localhost:8000 \
  > $LOG/chromium.log 2>&1 < /dev/null &
setsid nohup python3 -u $HERE/run.py kiosk "$KEY_FILE" > $LOG/kiosk.log 2>&1 < /dev/null &
sleep 12; tail -2 $LOG/kiosk.log

# 5. Can Pupper talk? The kiosk needs a few seconds after WAKE PUPPER to reach Gemini.
for i in $(seq 1 15); do
  curl -s -m 2 localhost:8000/mission/recent | grep -q '"listeners": *[1-9]' && break; sleep 1
done
python3 "$HERE/doctor.py" || echo "[mission] the doctor found problems (above). Pupper may be silent until they are fixed."
echo "ready. Hear it:  python3 $HERE/run.py say test      Stop with personality/stop.sh"
