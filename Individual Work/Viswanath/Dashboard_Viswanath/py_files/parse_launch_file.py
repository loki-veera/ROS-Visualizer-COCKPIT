import re
import sys
import Tracer
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

package = 'beginner_tutorials'
launch_file = 'basic2.launch'

Tracer.get_nodes(package, launch_file)

# path = '/home/lokesh/catkin_ws/src/mir_3d_image_segmentation/ros/launch/segmentation.launch'
# path = '/home/av/catkin_ws/src/beginner_tutorials/launch/basic2.launch'
#print(sys.argv[1])
# path = sys.argv[1]
# launch_file_nodes = sorted(get_nodes(path))
# launch_file_nodes.append("My_node")
# for idx in range(len(launch_file_nodes)):
#     launch_file_nodes[idx] = '/'+launch_file_nodes[idx]
#
# # current_nodes = sorted(sys.argv[2:])
#
# current_nodes = rosnode.get_node_names()
#
# active_nodes = list(set(launch_file_nodes)&set(current_nodes))
#
# ret_node_list = []
# for idx in range(len(launch_file_nodes)):
#     if launch_file_nodes[idx] in active_nodes:
#         node = launch_file_nodes[idx]+"_1"
#         ret_node_list.append(node)
#     else:
#         node = launch_file_nodes[idx]+"_0"
#         ret_node_list.append(node)
# #print("===============")
# #print(launch_file_nodes)
# #print("===============")
# #print(current_nodes)
# #print("===============")
# #print(active_nodes)
# #print("===============")
# print(ret_node_list)
