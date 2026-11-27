#!/bin/bash
# Stop the dodge node and the robot stack started by robot/start.sh.
GEMINI_DIR=${GEMINI_PUPPER_DIR:-$HOME/gemini_pupper}
# Gemini's server, if it is running: relax the motors the way its own stop does.
if curl -s -m 2 -o /dev/null localhost:8000/health; then
  curl -s -m 3 -X POST localhost:8000/stop/immediate >/dev/null
  curl -s -m 3 -X POST localhost:8000/walking/deactivate >/dev/null && sleep 2
  pkill -f "[r]obot/kiosk.py"; pkill -f "[g]emini-pupper-kiosk"; pkill -f "[r]obot/server.py"
fi
pkill -f "[t]ools/dodging_view.py"
pkill -INT -f "[s]ar_dodge.launch.py"
for i in $(seq 1 15); do pgrep -f "[r]os2_control_node" >/dev/null || break; sleep 1; done
pkill -9 -f "[s]ar_dodge.launch.py"; pkill -9 -f "[r]obot/dodge_node.py"; pkill -9 -f "[r]obot/fast_mux.py"
# The dodge node's camera processes end by themselves when the node goes; this is for one that did not.
sleep 0.5; pkill -9 -f "[m]ultiprocessing.spawn import spawn_main"
pkill -9 -f "[/]opt/ros/jazzy/lib/"; pkill -9 -f "[/]home/pi/pupperv3-monorepo/ros2_ws/install/"
pkill -f "[f]ollow_service.py" && sleep 2; pkill -9 -f "[f]ollow_service.py"
sudo -n systemctl start pupper-gui 2>/dev/null
echo "stopped"
