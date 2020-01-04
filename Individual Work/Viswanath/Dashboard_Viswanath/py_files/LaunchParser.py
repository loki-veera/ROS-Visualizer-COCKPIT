#!/bin/env python
"""Parses through ROS Launch file to get details.

Used for getting the information from a ROS Launch file. The nodes' details
are returned. Path of the launch file has to provided as command line argument
and is used for getting the name of the respective package for operation.
"""

__author__ = "Minh Nguyen"
__credits__ = ["Anargh Viswanath"]
__license__ = "MPL"
__version__ = "1.0.1"
__maintainer__ = "Minh Nguyen"
__email__ = " "
__status__ = "Development"
__created_on__ = " "
__last_modified__ = "04.01.2020"

import os
import re
import sys
import xml.etree.ElementTree as ET
from roslib.packages import get_pkg_dir, get_dir_pkg, \
                            InvalidROSPkgException


INDENT = '  '

class LaunchRootParser(object):
    """Class for root parsing through a launch file.

    """
    def __init__(self, launch_root, arg_dict={}):
        self.includes = []
        self.groups = {}
        self.nodes = {}
        self.rosparams = []
        self.arg_dict = arg_dict
        self.default_args = []
        self.conditions = {}                # TODO
        self.meaningless_conditions = []    # TODO
        self.issues = {}
        self.parse_launch_root(launch_root)
        pass

    def add_issue(self, issue):
        if issue in self.issues:
            self.issues[issue] += 1
        else:
            self.issues[issue] = 1
            pass
        pass

    def parse_launch_root(self, launch_root):
        arguments = launch_root.findall('arg')
        for arg in arguments:
            name, default_value, value = self.parse_argument(arg)
            if default_value is None:
                self.add_issue('launch root argument \'%s\' does not have a' \
                               ' default value' % name)
                continue
            if name not in self.arg_dict:
                self.arg_dict[name] = default_value
                self.default_args.append(name)
                pass
            pass

        node_elements = launch_root.findall('node')
        for node_element in node_elements:
            self.parse_node(node_element)
            pass

        group_elements = launch_root.findall('group')
        for group_element in group_elements:
            self.parse_group(group_element)
            pass

        include_elements = launch_root.findall('include')
        for include_element in include_elements:
            self.parse_include(include_element)
            pass

        rosparam_elements = launch_root.findall('rosparam')
        for rosparam_element in rosparam_elements:
            self.parse_rosparam(rosparam_element)
            pass
        return

    def parse_rosparam(self, rosparam_element):
        file_name = self.parse_xml_value(rosparam_element.get('file'))
        if file_name is None:
            self.add_issue('error: rosparam element does not have a file attribute')
            return

        command = self.parse_xml_value(rosparam_element.get('command'))
        if command is None:
            self.add_issue('error: rosparam element does not have a command attribute')
            return

        ns = self.parse_xml_value(rosparam_element.get('ns'))

        self.rosparams.append({'file': file_name, 'command': command, 'ns': ns})
        return

    def parse_node(self, node_element):
        node_name = self.parse_xml_value(node_element.get('name'))
        if node_name is None:
            sys.stderr.write('node element missing a name argument')
            return

        package = self.parse_xml_value(node_element.get('pkg'))
        if package is None:
            self.add_issue('error: node %s has no pkg attribute' % node_name)
            return

        node_type = self.parse_xml_value(node_element.get('type'))
        if node_type is None:
            self.add_issue('error: node %s has no type attribute' % node_type)
            return

        param_elements = node_element.findall('param')
        params = []
        for param_element in param_elements:
            name = self.parse_xml_value(param_element.get('name'))
            if name is None:
                self.add_issue('error: node %s has param without name attribute'
                               % node_name)
                continue
            value = self.parse_xml_value(param_element.get('value'))
            if value is None:
                self.add_issue('error: param %s of node %s has no value attribute'
                               % (name, node_name))
                pass
            param_type = self.parse_xml_value(param_element.get('type'))
            if param_type is None:
                self.add_issue('error: param %s of node %s has no type attribute'
                               % (name, node_name))
                pass
            params.append({'name': name, 'value': value, 'type': param_type})
            pass

        remap_elements = node_element.findall('remap')
        remaps = []
        for remap_element in remap_elements:
            from_attribute = self.parse_xml_value(remap_element.get('from'))
            if from_attribute is None:
                self.add_issue('error: node %s has remap without from attribute'
                               % node_name)
                continue
            to_attribute = self.parse_xml_value(remap_element.get('to'))
            if to_attribute is None:
                self.add_issue('error: node %s has remap without to attribute'
                               % node_name)
                continue
            remaps.append({'from': from_attribute, 'to': to_attribute})
            pass

        self.nodes[node_name] = {'package': package, 'type': node_type,
                                 'params': params, 'remaps': remaps}
        pass

    def parse_include(self, include_element):
        include_file = self.parse_xml_value(include_element.get('file'))
        if include_file is not None:
            include_arguments = include_element.findall('arg')
            arg_dict = {}
            for arg in include_arguments:
                name, default_value, value = self.parse_argument(arg)
                if default_value is not None:
                    self.add_issue('include argument \'%s\' use \'default\' key' % name)
                    arg_dict[name] = default_value
                    pass
                if value is not None:
                    arg_dict[name] = value
                    pass
                pass
            self.includes.append(LaunchFileParser(include_file, arg_dict))
        return

    def parse_group(self, group_element):
        ns = group_element.get('ns')
        group_key = len(self.groups)
        if ns is not None:
            group_key = ns
            pass
        self.groups[group_key] = LaunchRootParser(group_element, self.arg_dict)
        pass

    def parse_xml_value(self, value_string):
        if value_string is None:
            return None
        value_string = self.parse_env_keyword(value_string)
        value_string = self.parse_find_keyword(value_string)
        value_string = self.parse_arg_keyword(value_string)
        return value_string

    def parse_arg_keyword(self, string):
        match = re.match(r'.*\$\(arg (\S+)\).*', string)
        while match is not None:
            arg_name = match.group(1)
            if arg_name not in self.arg_dict:
                self.add_issue('argument \'%s\' not defined' % (arg_name))
                return None
            sub_string = r'\$\(arg %s\)' % arg_name
            string = re.sub(sub_string, self.arg_dict[arg_name], string)
            match = re.match(r'.*\$\(arg (\S+)\).*', string)
            pass
        return string

    def parse_find_keyword(self, string):
        match = re.match(r'.*\$\(find (\S+)\).*', string)
        if match is not None:
            package_name = match.group(1)
            try:
                package_path = get_pkg_dir(package_name)
            except InvalidROSPkgException:
                self.add_issue('cannot find package %s' % (package_name))
                return None

            string = re.sub(r'\$\(find \S+\)', package_path, string)
        return string

    def parse_env_keyword(self, string):
        match = re.match(r'.*\$\(optenv (\S+).*\).*', string)
        if match is not None:
            env_variable = match.group(1)
            if env_variable not in os.environ:
                self.add_issue('environment variable %s is not set' % env_variable)
                return None
            string = re.sub(r'\$\(optenv \S+.*\)',
                            os.environ[env_variable], string)
            pass

        match = re.match(r'.*\$\(env (\S+).*\).*', string)
        if match is not None:
            env_variable = match.group(1)
            if env_variable not in os.environ:
                self.add_issue('environment variable %s is not set' % env_variable)
                return None
            string = re.sub(r'\$\(env \S+.*\)',
                            os.environ[env_variable], string)
            pass
        return string

    def parse_argument(self, argument):
        name = argument.get('name')
        assert name is not None, 'invalid launch file: arg element must have name attribute'
        default_value = self.parse_xml_value(argument.get('default'))
        value = self.parse_xml_value(argument.get('value'))
        return name, default_value, value

    def get_launch_components_recursive(self):
        """Function to return the nodes inside the launch file.

        Parameters
        ----------

        Returns
        -------
        self.nodes : dictionary
            This has the details of the nodes as a dictionary with params.
        """
        return self.nodes

    def get_includes_recursive(self, prefix=''):
        """Function to return the nodes inside the included components of
        launch file.

        Parameters
        ----------
        prefix : string
            To provide as indentation in argumennts while calling recursively.

        Returns
        -------
        self.nodes : dictionary
            This has the details of the nodes as a dictionary with params.
        """
        Nodes = {}
        for include_object in self.includes:
            Merge_node_details(include_object.get_launch_components_recursive(prefix + INDENT), Nodes)
            pass
        for group_object in self.groups.itervalues():
            Merge_node_details(group_object.get_includes_recursive(prefix + INDENT), Nodes)
            pass
        return Nodes
    pass

class LaunchFileParser(LaunchRootParser):
    """Class for LaunchFileParser to parse through ROS launch file and return
    nodes.

    """
    def __init__(self, launch_file_path, arg_dict={}):
        try:
            launch_root = ET.parse(launch_file_path).getroot()
        except IOError as e:
            sys.stderr.write('IOError parsing %s: %s' % (launch_file_path, e))
            sys.exit(1)
        self.path = launch_file_path
        self.package_name = get_dir_pkg(launch_file_path)[1]
        if self.package_name is None:
            sys.stderr.write('no package found for file %s' % (self.path))
            self.package_name = ''
            pass
        super(LaunchFileParser, self).__init__(launch_root, arg_dict=arg_dict)
        pass

    def get_launch_components_recursive(self, prefix=''):
        """Function to parse and return the nodes inside the launch file.

        Parameters
        ----------

        Returns
        -------
        Total_nodes : dictionary
            Details of all nodes within the launch file and the launch files
            within the launch file.
        """
        Total_nodes = {}
        Merge_node_details(super(LaunchFileParser, self).get_launch_components_recursive(), Total_nodes)
        Merge_node_details(super(LaunchFileParser, self).get_includes_recursive(prefix), Total_nodes)
        return Total_nodes
    pass


def Merge_node_details(node1, node2):
    """Function to merge.

    Parameters
    ----------
    node1 : dictionary
        This has the details of the nodes as a dictionary with params.
    node2 : dictionary
        This has the details of the nodes as a dictionary with params.

    Returns
    -------
    node2.update(node1) : dictionary
        Merging details of both nodes as one dictionary.
    """
    return(node2.update(node1))
