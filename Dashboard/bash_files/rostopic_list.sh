#!/bin/bash

source '/home/zuha/catkin_ws/devel/setup.bash'
#Zuha Code start
if [ $1 == 2 ]
then
topiclist=$(rostopic list)
IFS=' ' # space is set as delimiter
read -ra ADDR <<< "$topiclist" # topiclist is read into an array as tokens separated by IFS
for i in "${ADDR[@]}"; do # access each element of array
    echo "$i"
done
fi  
