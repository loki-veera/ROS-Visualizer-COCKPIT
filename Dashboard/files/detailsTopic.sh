#!/bin/bash
#Created on 15-01-2020
#Author : Zuha Karim
source "/usr/share/cockpit/dashboard/files/dashboard_config.sh"
source "$path"

if [ $1 == 1 ]
then
topicdet=$(timeout 3s rostopic echo $2)
echo "$topicdet"
fi

