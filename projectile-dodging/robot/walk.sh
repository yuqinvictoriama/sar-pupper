#!/bin/bash
# Stand the robot up in the gemini_walk controller (the one dodging is tuned for).
# THE ROBOT STANDS UP. Put it on the floor, on its feet or lying flat, with space around it.
#
#   bash robot/walk.sh          stand up and walk with gemini_walk
#   bash robot/walk.sh relax    let the legs go limp (same as the joystick e-stop, but gentle)
#
# Use this when Gemini Pupper's server is not running. When it is, wake the
# robot the usual way instead (Gemini switches controllers itself).
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"
WALKERS="gemini_walk neural_controller_default neural_controller_default_alt neural_controller neural_controller_student neural_controller_three_legged"
LIST=$(ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g')
if [ -z "$LIST" ]; then echo "No controller manager found. Is the robot stack running (bash robot/start.sh)?"; exit 1; fi
ACTIVE=""
for c in $WALKERS; do
  echo "$LIST" | awk -v c="$c" '$1==c && $NF=="active" {found=1} END {exit !found}' && ACTIVE="$ACTIVE $c"
done
if [ "${1:-}" = "relax" ]; then
  [ -z "$ACTIVE" ] && { echo "already relaxed"; exit 0; }
  ros2 control switch_controllers --deactivate $ACTIVE
  exit $?
fi
if ! echo "$LIST" | awk '$1=="gemini_walk" {found=1} END {exit !found}'; then
  echo "gemini_walk is not loaded (it comes from ~/gemini_pupper). Use the joystick X button instead: dodging"
  echo "then works with the course's default controller, capped at 0.49 m/s."
  exit 1
fi
if echo " $ACTIVE " | grep -q " gemini_walk "; then echo "gemini_walk is already active"; exit 0; fi
# A low battery or a homing that stopped short is why the robot cannot get up evenly: say so before it tries.
python3 "$HERE/homing_report.py" --brief
if [ -n "$ACTIVE" ]; then
  ros2 control switch_controllers --activate gemini_walk --deactivate $ACTIVE
else
  ros2 control switch_controllers --activate gemini_walk
fi
