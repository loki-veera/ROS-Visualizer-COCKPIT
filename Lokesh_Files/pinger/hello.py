import sys 
import os
from subprocess import Popen
import signal
import time
import rosgraph


def run(cmd):
    return Popen(cmd, shell=False, preexec_fn=os.setsid)


def start_process(cmd, type):
    return run(cmd)


print("Output from Python")

p_ros = start_process('/opt/ros/kinetic/bin/roscore', 'ros')

if rosgraph.is_master_online():
    print("ROS is online")
    print("PGID ROS: ", os.getpgid(p_ros.pid))
else:
    print("Start ROS core give argument yes or y")