#!/bin/bash
# tmux new-session -A -s dashboard
source "/home/av/catkin_ws/devel/setup.bash"


echo hello
nodes=$(rosnode list)
echo "$nodes"
