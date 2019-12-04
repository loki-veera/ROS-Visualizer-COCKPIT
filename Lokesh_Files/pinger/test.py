import sys 
import os
from subprocess import Popen
import signal
import time
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
#print (sys.path)

print("Entered into Py file")
import rosgraph
print("Imported ROSGRAPH")

def run(cmd):
    print("In run method")
    return Popen(cmd, shell=False, preexec_fn=os.setsid)


def start_process(cmd, type):
    print("In start Process method")
    return run(cmd)

p_ros = start_process('/opt/ros/kinetic/bin/roscore', 'ros')
print("PGID ROS: ", os.getpgid(p_ros.pid))

