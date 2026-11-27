#!/bin/bash
# Double-click this file on a Mac (or run it): Pupper Mission Control opens in your browser,
# running the whole demo in simulation. No robot and no ROS needed, only Python 3 with numpy.
#
# On the robot it is different: robot/start_demo.sh starts Mission Control for the real run,
# and you open http://<pupper>.local:8078 from the laptop.
cd "$(dirname "$0")" || exit 1
python3 -c "import numpy" 2>/dev/null || { echo "This needs numpy:  python3 -m pip install numpy"; read -r -p "Press Return to close. "; exit 1; }
exec python3 tools/mission_control.py --sim --open
