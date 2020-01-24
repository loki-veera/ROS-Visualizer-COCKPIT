#####
# Author : Lokesh Veermacheneni
# Created on: 03-01-2020
#####
#!/bin/bash
source "/usr/share/cockpit/dashboard/files/dashboard_config.sh"
source "$path"

# If command line argument is 1, it means
# start the ROSCORE
if [ $1 == 1 ]
then
    echo 1
    core=$(roscore)

# If command line arguemnt is 0, it means
# check whether the ROSCORE is running or not
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
    
# If command line argumet is 2, it means
# Kill the ROSCORE
elif [ $1 == 2 ]
then
    core=$(killall -9 rosmaster)
    echo 2
fi
