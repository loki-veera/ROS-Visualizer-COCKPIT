#####
# Author : Lokesh Veermacheneni
# Created on: 03-01-2020
#####
#!/bin/bash
source "/usr/share/cockpit/dashboard/files/dashboard_config.sh"
source "$path"

#cmd="locate -b '\\"$1"'"
# Command to find the file
cmd="find /home -name $1"
path=$(eval $cmd)

# Split the path at '/' to find the package name
IFS='/'
read -ra split<<<"$path"
last_str=''
package_name=''
for i in "${split[@]}";
do
    if [ "$last_str" == 'src' ]
    then
        package_name=$i
    fi
    last_str=$i
done

# Launch the file
launch_cmd="roslaunch "$package_name" "$1
echo 3
launch_status=$(eval $launch_cmd)
