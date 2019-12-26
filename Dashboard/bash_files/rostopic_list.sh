#!/bin/bash

source '/home/zuha/catkin_ws/devel/setup.bash'

#Zuha Code start
if [ $1 == 2 ]
then
    topiclist=$(rostopic list)
	echo "$topiclist" 
	
    
fi  
