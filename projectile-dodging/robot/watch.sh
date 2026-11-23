#!/bin/bash
# Live view of what the dodge node sees and decides.
#   robot/watch.sh            state line five times a second
#   robot/watch.sh events     only dodge events
#   robot/watch.sh arm        let it move the robot      (robot/watch.sh disarm to stop that)
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"
case "${1:-state}" in
  events) ros2 topic echo /dodge/event --field data ;;
  arm) ros2 service call /dodge/arm std_srvs/srv/SetBool "{data: true}" ;;
  disarm) ros2 service call /dodge/arm std_srvs/srv/SetBool "{data: false}" ;;
  *) python3 -u "$HERE/../tools/watch_state.py" ;;
esac
