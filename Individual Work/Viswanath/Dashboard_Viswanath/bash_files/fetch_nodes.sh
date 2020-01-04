#!/bin/bash
#==============================================================================
#title           :fetch_nodes.sh
#description     :This script will make a header for a bash script.
#author		       : Anargh Viswanath
#date            :04.01.2020
#version         :1.0
#usage		       :bash fetch_nodes.sh filename.launch
#==============================================================================
# tmux new-session -A -s dashboard
source "/home/av/catkin_ws/devel/setup.bash"

cmd="locate -b '\\"$1"'"
path=$(eval $cmd)

pyfile_cmd="locate -b 'GetNodes.py'"
pyfile_path=$(eval $pyfile_cmd)

python2 $pyfile_path $path
