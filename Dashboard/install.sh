#####
# Author : Lokesh Veermacheneni
# Created on: 03-01-2020
#####

# Check the ROS version
cmd="rosversion -d"
ret_ros=$(eval $cmd)
ros_val=0 
if [ -z "$ret_ros" ]
then
    echo Please Install ROS
else
    ros_val=1
    echo ROS is installed......
fi  

# Check the cockpit version
cmd="cockpit-bridge --version"
ret=$(eval $cmd)    
cockpit_val=0
if hash -- $ret 2> /dev/null;
then
    echo Please Install cockpit
else
    cockpit_val=1
    echo cockpit is installed......
fi

setup_bash_path=""
# If ROS and Cockpit are installed try creating the symlink
# and copy the files from $PWD/bash_files to /usr/share/cockpit/dashboard
if [ $ros_val == 1 ] 
then
    if [ $cockpit_val == 1 ]
    then
        echo "Enter the path to setup.bash"
        read setup_bash_path
        echo "Path you entered is: "
        echo $setup_bash_path
        echo "Changing the config file please wait.."
        sed -i "/path=/d" $PWD/files/dashboard_config.sh
        echo "path='"$setup_bash_path"'" >> $PWD/files/dashboard_config.sh
        cmd="mkdir -p ~/.local/share/cockpit"
        ret=$(eval $cmd)
        cmd="ln -snf $PWD ~/.local/share/cockpit/dashboard"
        ret=$(eval $cmd)
        cmd="sudo cp -R $PWD/files /usr/share/cockpit/dashboard"
        ret=$(eval $cmd)
        
    fi
fi
