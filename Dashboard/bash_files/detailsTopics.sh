#!/bin/bash
#Created on 15-01-2020
#Author : Zuha Karim
source '/home/zuha/catkin_ws/devel/setup.bash'
if [ $1 == 1 ]
then
topicdet=$(timeout 3s rostopic echo $2)
echo "$topicdet"
fi
