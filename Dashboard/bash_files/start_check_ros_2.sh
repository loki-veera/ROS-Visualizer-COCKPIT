#!/bin/bash
source "/home/loki/catkin_ws/devel/setup.bash"

if [ $1 == 1 ]
then
    echo 1
    core=$(roscore)
elif [ $1 == 0 ]
then
    nodes=$(rosnode \
    list)
    if [ -z "$nodes" ]
    then
        echo 0
    else
        echo 1
    fi
elif [ $1 == 2 ]
then
    core=$(killall -9 rosmaster)
    echo 2
fi