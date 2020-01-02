#!/usr/bin/env python
import sys
from roslib.packages import find_resource
from rospkg import ResourceNotFound
from LaunchParser import LaunchFileParser
import rosnode

def get_nodes(package, launch_file):
    try:
        launch_file_paths = find_resource(package, launch_file)
    except ResourceNotFound as e:
        print('Unable to find package %s: %s' % (package, e))
        return
    if len(launch_file_paths) == 0:
        print('Unable to find launch file %s' % launch_file)
        return
    # print('Found launch files:\n%s' % (launch_file_paths))
    launch_records = []
    for path in launch_file_paths:
        launch_file_obj = LaunchFileParser(path)
    # launch_records.append(launch_file_obj)
        Node_details = launch_file_obj.print_launch_components_recursive()
        pass
    Launcher_nodes = list(Node_details.keys())
    # print('Nodes in Launcher:')
    # print(Launcher_nodes)
    Running_nodes = []
    # print(rosnode.get_node_names())
    for i in rosnode.get_node_names():
        # print(i[1:])
        Running_nodes.append(i[1:])
    # print('Nodes running')
    # print(Running_nodes)
    Active_Launcher_nodes = [i+'_1' for i in Launcher_nodes if i in Running_nodes]
    # print(Active_Launcher_nodes)
    Inactive_Launcher_nodes = [i+'_0' for i in Launcher_nodes if i not in Running_nodes]
    print(Active_Launcher_nodes + Inactive_Launcher_nodes)
    return


# if __name__ == '__main__':
#     assert len(sys.argv) == 3, 'script accept exactly 2 arguments: ros package name and launch file'
#     main(sys.argv)
