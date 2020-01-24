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
# Changed by Lokesh
source "/usr/share/cockpit/dashboard/files/dashboard_config.sh"
source $path

# echo $path
cmd="locate -b '\\"$1"'"
# cmd="sudo find /home -name $1"
# End of changes
path_1=$(eval $cmd)
# echo $cmd
# echo $path_1
# echo $1
# Start of changes Lokesh
#pyfile_cmd="locate -b 'GetNodes.py'"
#pyfile_path=$(eval $pyfile_cmd)
#python2 $pyfile_path $path
python2 "/usr/share/cockpit/dashboard/files/GetNodes.py" $path_1
# End of Changes
