#!/bin/bash
# Changes by Lokesh
# source '/home/zuha/catkin_ws/devel/setup.bash'
source "/usr/share/cockpit/dashboard/files/dashboard_config.sh"
source "$path"

if [ $1 == 1 ]
then
topiclist=$(rostopic list)
echo "$topiclist"
fi
