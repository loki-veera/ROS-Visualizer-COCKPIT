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

if [ $ros_val == 1 ] 
then
    if [ $cockpit_val == 1 ]
    then
        cmd="mkdir -p ~/.local/share/cockpit"
        ret=$(eval $cmd)
        cmd="ln -snf $PWD ~/.local/share/cockpit/dashboard"
        ret=$(eval $cmd)
        cmd="sudo cp -R $PWD/Dashbaord/bashfiles /usr/share/cockpit/dashboard"
        ret=$(eval $cmd)
    fi
fi