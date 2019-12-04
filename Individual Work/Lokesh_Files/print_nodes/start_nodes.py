import os
from subprocess import Popen
import signal
import time
import rospy

def run(cmd):
    return Popen(cmd, shell=False, preexec_fn=os.setsid)


def start_process(cmd, type):
    return run(cmd)

# p_ros = start_process('/opt/ros/kinetic/bin/roscore', 'ros')

# print("PGID ROS: ", os.getpgid(p_ros.pid))
nodes = rospy.get_published_topics()

for each_node in nodes:
    print(each_node)