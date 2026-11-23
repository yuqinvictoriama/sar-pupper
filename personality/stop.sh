#!/bin/bash
# Stop Pupper after personality/start.sh: the mission kiosk, then everything the
# lab's stop.sh stops (it relaxes the motors and frees port 8000, so it stops the
# mission server too).
HERE=$(cd "$(dirname "$0")" && pwd)
GP=${GEMINI_PUPPER_DIR:-$(dirname "$(dirname "$HERE")")/gemini_pupper}
pkill -f "[p]ersonality/run.py kiosk" 2>/dev/null
exec bash "$GP/robot/stop.sh"
