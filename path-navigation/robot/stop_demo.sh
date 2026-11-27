#!/bin/bash
# Stop everything robot/start_demo.sh started. The motors relax: Pupper settles onto the floor.
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(dirname "$(dirname "$HERE")")
# 1. Navigation first, so nothing asks for motion while the rest goes down.
pkill -INT -f "[r]obot/nav_node.py" 2>/dev/null
pkill -f "[t]ools/mission_control.py" 2>/dev/null
# 2. Pupper's voice and the lab's server (this relaxes the motors and frees port 8000).
if [ -f "$REPO/personality/stop.sh" ]; then bash "$REPO/personality/stop.sh" >/dev/null 2>&1; fi
# 3. The launch and everything under it.
pkill -INT -f "[d]emo.launch.py" 2>/dev/null
for i in $(seq 1 15); do pgrep -f "[r]os2_control_node" >/dev/null || break; sleep 1; done
pkill -9 -f "[d]emo.launch.py" 2>/dev/null
for p in "[r]obot/nav_node.py" "[r]obot/keyword_node.py" "[r]obot/terrain_node.py" "[r]obot/stepover_node.py" \
         "[r]obot/estop_guard.py" "[r]obot/spawn_controllers.py"; do pkill -9 -f "$p" 2>/dev/null; done
# 4. The dodging component's own stop: its node, camera processes, the mux, the course stack, the GUI back on.
if [ -f "$REPO/projectile-dodging/robot/stop.sh" ]; then bash "$REPO/projectile-dodging/robot/stop.sh"; else echo "stopped"; fi
