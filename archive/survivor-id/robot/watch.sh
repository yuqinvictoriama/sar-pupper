#!/bin/bash
# Live view of what survivor identification sees and decides, and its switches.
#   robot/watch.sh               one line five times a second, plus events
#   robot/watch.sh events        only events
#   robot/watch.sh people        everyone it is tracking
#   robot/watch.sh motion on     let it walk to a pinned survivor   (robot/watch.sh motion off to stop that)
#   robot/watch.sh cancel        drop the current pin
#   robot/watch.sh enable        start watching     (robot/watch.sh disable: stop, and free the Hailo)
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/env.sh"
case "${1:-state}" in
  events) ros2 topic echo /survivor/event --field data ;;
  people) ros2 topic echo /people --field data ;;
  motion) case "${2:-}" in
            on) ros2 service call /survivor/motion std_srvs/srv/SetBool "{data: true}" ;;
            off) ros2 service call /survivor/motion std_srvs/srv/SetBool "{data: false}" ;;
            *) echo "robot/watch.sh motion on|off" ;;
          esac ;;
  cancel) ros2 service call /survivor/cancel std_srvs/srv/Trigger ;;
  enable) ros2 service call /survivor/enable std_srvs/srv/Trigger ;;
  disable) ros2 service call /survivor/disable std_srvs/srv/Trigger ;;
  *) python3 -u "$HERE/../tools/watch_state.py" ;;
esac
