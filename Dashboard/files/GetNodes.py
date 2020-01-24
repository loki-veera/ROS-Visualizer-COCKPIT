#!/bin/env python
"""Prints the ROS Launch file nodes along with their running status.

Used for getting the status of the nodes of a ROS Launch file. The nodes
are printed followed by _1 or _0 highlighting whether they are active or
inactive. Path of the launch file has to provided as command line argument
and is used for getting the name of the respective package for operation.
"""

__author__ = "Anargh Viswanath"
__credits__ = ["Minh Nguyen", "Lokesh Veeramacheneni"]
__license__ = "MPL"
__version__ = "1.0.1"
__maintainer__ = "Anargh Viswanath"
__email__ = "vanargh@gmail.com"
__status__ = "Development"
__created_on__ = "04.01.20"
__last_modified__ = "04.01.20"

import re
import os
import glob
import sys
from LaunchTracer import get_nodes

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

def get_package_name(path):
    """Function to get package name and launch file name.

    Parameters
    ----------
    path : string
        This parameter specifies the path of the launch file.

    Returns
    -------
    package : string
        This parameter specifies package name of launch file.
    launch_file : string
        This parameter specifies the name of launch file.
    """
    # Path is split into a list
    split_path = path.split('/')
    # Finding the package name
    for i,j in enumerate(split_path):
        if j == 'src':
            package = split_path[i+1]
            break
    # Finding the launch file name
    launch_file = split_path[-1]
    return package, launch_file

def main():
    """Main function of the file to print status of nodes.

    Parameters
    ----------

    Returns
    -------

    """
    # path = "abc"
    # for root, dirs, files in os.walk('/home'):
    #     if sys.argv[1] in files:
    #         print("Exits")
    #         print(root)
    #         print(root+sys.argv[1])
    #         path = root+sys.argv[1]
    #         break
    #     else:
    #         print(files)
    #         path = "nt exists"
    # print(path)
    # Getting path of the launch file as command line argument
    path = sys.argv[1]
    # Getting package and launch file names
    package_name, launch_file_name = get_package_name(path)
    # Getting nodes and their status as list
    nodes = get_nodes(package_name, launch_file_name)
    # Printing the nodes' details as list
    print(nodes)

if __name__ == '__main__':
    main()
