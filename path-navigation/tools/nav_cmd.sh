#!/bin/bash
# Send the navigation node a command from a terminal (Mission Control's buttons do the same).
#
#   bash tools/nav_cmd.sh start        run the mission          bash tools/nav_cmd.sh pause | resume | end
#   bash tools/nav_cmd.sh motion_on    allow it to walk (someone at the e-stop!)      motion_off
#   bash tools/nav_cmd.sh phrase       stand in for the phrase detector: the survivor said "You saved me, Pupper!"
#   bash tools/nav_cmd.sh bearing:30   stand in for the sound module: a cry 30 degrees to the left
#   bash tools/nav_cmd.sh goto:ZONE2   start from a stage (SEEK ORIENT CONFIRM ZONE1 REFIX ZONE2 ARRIVE RESCUE)
#   bash tools/nav_cmd.sh watch        print the mission's state as it changes
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/../robot/env.sh"
if [ "${1:-}" = "watch" ]; then exec ros2 topic echo /nav/event std_msgs/msg/String --field data; fi
[ -n "${1:-}" ] || { sed -n '2,10p' "$0"; exit 1; }
exec ros2 topic pub -1 /nav/command std_msgs/msg/String "data: '$1'"
