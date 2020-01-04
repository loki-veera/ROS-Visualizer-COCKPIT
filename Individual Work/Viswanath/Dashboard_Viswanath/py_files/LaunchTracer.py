#!/usr/bin/env python
"""Traces ROS Launch file nodes and checks their running status.

Performs checking of the nodes of a ROS Launch file. The nodes
are returned followed by _1 or _0 highlighting whether they are active or
inactive.
"""

__author__ = "Anargh Viswanath"
__credits__ = ["Minh Nguyen", "Lokesh Veeramacheneni"]
__license__ = "MPL"
__version__ = "1.0.1"
__maintainer__ = "Anargh Viswanath"
__email__ = "vanargh@gmail.com"
__status__ = "Development"
__created_on__ = "04.01.2020"
__last_modified__ = "04.01.20"

import sys
from roslib.packages import find_resource
from rospkg import ResourceNotFound
from LaunchParser import LaunchFileParser
import rosnode

def get_nodes(package, launch_file):
    """Function to get status of all nodes inside a launch file.

    Parameters
    ----------
    package : string
        This parameter specifies package name of launch file.
    launch_file : string
        This parameter specifies the name of launch file.

    Returns
    -------
    Launcher_nodes_status : list
        The launcher nodes are returned followed by _1 or _0 highlighting
        whether they are active or inactive.
    """
    try:
        launch_file_paths = find_resource(package, launch_file)
    except ResourceNotFound as e:
        return ['Package not found']
    if len(launch_file_paths) == 0:
        return ['Launch file not found']

    Launcher_node_details = get_launcher_node_details(launch_file_paths)
    Launcher_nodes = list(Launcher_node_details.keys())
    Running_nodes = get_running_nodes()
    Launcher_nodes_status = get_launcher_nodes_status(Launcher_nodes, Running_nodes)
    return Launcher_nodes_status

def get_launcher_node_details(launch_file_paths):
    """Function to get all nodes inside a launch file.

    Parameters
    ----------
    launch_file_paths : list
        List of matching paths for the given package and launch file.

    Returns
    -------
    Node_details : dictionary
        Details of all nodes as a dictionary with params within the launch file
        and the launch files within the launch file.
    """
    launch_records = []
    for path in launch_file_paths:
        launch_file_obj = LaunchFileParser(path)
        Node_details = launch_file_obj.get_launch_components_recursive()
        pass
    return Node_details

def get_running_nodes():
    """Function to get all presently active nodes.

    Parameters
    ----------

    Returns
    -------
    Running_nodes : list
        List of all presently active nodes.
    """
    Running_nodes = []
    for i in rosnode.get_node_names():
        Running_nodes.append(i[1:])
    return Running_nodes

def get_launcher_nodes_status(Launcher_nodes, Running_nodes):
    """Function to identify the activity state of all nodes in launch file.

    Parameters
    ----------
    Launcher_nodes : list
        List of all nodes within the launch file.
    Running_nodes : list
        List of all presently active nodes.

    Returns
    -------
    Active_Launcher_nodes + Inactive_Launcher_nodes : list
        The launcher nodes are identified whether they are active or inactive
        and are returned followed by _1 or _0 highlighting their state.
    """
    Active_Launcher_nodes = [i+'_1' for i in Launcher_nodes if i in Running_nodes]
    Inactive_Launcher_nodes = [i+'_0' for i in Launcher_nodes if i not in Running_nodes]
    return Active_Launcher_nodes + Inactive_Launcher_nodes
