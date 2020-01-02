#!/bin/bash
# tmux new-session -A -s dashboard
source "/home/av/catkin_ws/devel/setup.bash"

cmd="locate -b '\\"$1"'"

path=$(eval $cmd)

pyfile_cmd="locate -b '\parse_launch_file.py'"
pyfile_path=$(eval $pyfile_cmd)

current_nodes=$(rosnode list)
python2 $pyfile_path $path 
