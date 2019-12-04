import os
import subprocess
from subprocess import Popen
import signal
import time
import sys
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

# import rospy as rp
# print(type(rp))
# nodes = []
# try:
#     nodes = rp.get_published_topics()
# except Exception as e:
#     print("Exception 1")
#     print(e)
# print("print the nodes")
# print(nodes)


## Setup - 2
# import rosnode
# import rosgraph
# import argparse
# nodes = []
# try:
#     parser = argparse.ArgumentParser()
#     parser.add_argument('ROS_MASTER_URI', type=str, nargs='?', metavar='URI', help='ROS master URI to use.')
#     args = parser.parse_args()
#     ID = '/rosnode'
#     master = rosgraph.Master(ID, master_uri=args.ROS_MASTER_URI)
#     print ("Using master at {}".format(master.getUri()))
#     nodes = rosnode.get_node_names()
# except Exception as e:
#     print("Exception 2")
#     print(e)
# print(nodes)


# ## Setup - 3
# try:
#     print("Inside strated ROSCORE")
#     print(os.system("rostopic list"))
# except Exception as e:
#     print("Exception 3")
#     print(e)

# ## Try - 02
# try:
#     output = subprocess.check_output("rostopic list", shell=True)
#     print(type(output))
#     # txt = open("out.txt", "w")
#     # txt.write(output)
#     # txt.close()
# except Exception as e:
#     print("Exception 4")
#     print(e)

# ## Try - 03
# try:
#     process = subprocess.Popen("rostopic list",stdout=subprocess.PIPE, shell=True)
#     proc_stdout = process.communicate()[0].strip()
#     print("B")
#     print(proc_stdout)
#     print("A")
# except Exception as e:
#     print("Exception 5")
#     print(e)

# ## Try - 04
try:
    process = subprocess.Popen('/bin/bash', stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
    out, err = process.communicate("rostopic list")
    txt = open("/home/lokesh/pinger/out.txt", "w")
    txt.write(out)
    txt.close()
except Exception as e:
    print("Exception 6")
    print(e)