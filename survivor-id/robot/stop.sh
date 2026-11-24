#!/bin/bash
# Stop survivor identification and everything robot/start.sh started.
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$(dirname "$HERE")")
DODGE_DIR=${SAR_DODGE_DIR:-$REPO/projectile-dodging}
pkill -f "[p]ersonality/run.py" 2>/dev/null
pkill -INT -f "[s]ar_survivor.launch.py"
for i in $(seq 1 5); do pgrep -f "[r]obot/survivor_node.py" >/dev/null || break; sleep 1; done   # let it release the Hailo
pkill -9 -f "[r]obot/survivor_node.py"; pkill -9 -f "[r]obot/approach_node.py"; pkill -9 -f "[r]obot/event_bridge.py"
# The dodging component's stop does the rest: Gemini's server (it relaxes the motors
# the way that server's own stop does), the robot stack, the dodge node.
bash "$DODGE_DIR/robot/stop.sh"
pkill -9 -f "[s]ar_survivor.launch.py" 2>/dev/null
exit 0
