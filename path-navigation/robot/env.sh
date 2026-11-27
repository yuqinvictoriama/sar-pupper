# ROS environment for the navigation tools. Use it in every new terminal on the Pupper:
#
#     source robot/env.sh
#
# It is the same environment the other components and Gemini Pupper's start.sh use.
# ROS_LOCALHOST_ONLY matters: without it the tools cannot see the robot stack's topics.
source /opt/ros/jazzy/setup.bash
source "$HOME/pupperv3-monorepo/ros2_ws/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
