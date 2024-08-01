#!/usr/bin/env python
# coding=utf-8
from __future__ import print_function
"""
Print the robot model based on nifti_robot.xacro, configured by the
`parts_config/*` parameters on the param server or from robot.yaml.

Usage:

    rosrun nifti_robot_description print_robot_model.py
        [_use_yaml_files:=True]  # if True, take parts config from robot.yaml and default-robot.yaml
        [_use_ros_params:=True]  # if True, take parts config from ROS params (and possibly override values from YAML files)
        [_pretty_xml:=True]  # if True, the URDF will be pretty-printed
        [_xacro_file:=path/to.xacro]  # path to robot XACRO definition; defaults to nifti_robot_description/urdf/nifti_robot.xacro
        [_yaml_files:=[path/to/default-robot.yaml,path/to/robot.yaml]]  # paths to robot.yaml files as YAML list (only needed when _use_yaml_files is True); defaults to [nifti_robot_description/conf/robot/default-robot.yaml,nifti_robot_description/conf/robot/robot.yaml]
        [_additional_xacro_args:={}]  # additional arg tags that should be added to the Xacro document. Keys are arg names, values are their defaults.
        [[part:=value ]...]  # additional parts configs (override all previous values)
"""

import os
import re
import socket
import sys
import yaml

import rospkg
import rospy
from rosgraph.names import load_mappings
from rospy.client import load_command_line_node_params
from rosparam import construct_angle_degrees, construct_angle_radians

from nifti_robot_description.render_xacro import render_xacro_file


def sorted_dict_str(d):
    res ="{"
    res += ", ".join("'%s': %r" % (key, value) for key, value in sorted(d.items()))
    res += "}"
    return res


if __name__ == '__main__':

    try:
        nifti_robot_description_path = rospkg.RosPack().get_path('nifti_robot_description')
    except rospkg.ResourceNotFound as e:
        sys.stderr.write("Rospack is not working. Have you sourced the ROS environment in this shell? Error: %s\n"
                         % str(e))
        sys.exit(1)

    # parse program arguments (from ROS-like parameter mappings)

    node_params = load_command_line_node_params(sys.argv)

    use_yaml_files = bool(node_params.get('use_yaml_files', True))
    use_ros_params = bool(node_params.get('use_ros_params', True))
    pretty_xml = bool(node_params.get('pretty_xml', True))

    robot_xacro = node_params.get('xacro_file', os.path.join(nifti_robot_description_path, 'urdf', 'nifti_robot.xacro'))

    sys.stderr.write("Rendering XACRO file %s\n" % (robot_xacro,))

    yaml_files = node_params.get('yaml_files', [
        os.path.join(nifti_robot_description_path, 'conf', 'robot', 'default-robot.yaml'),
        os.path.join(nifti_robot_description_path, 'conf', 'robot', 'robot.yaml')
    ])

    additional_yaml_files = node_params.get('additional_yaml_files', [])
    yaml_files += additional_yaml_files

    # validate program args

    if not use_ros_params and not use_yaml_files:
        sys.stderr.write("Either use_ros_params or use_yaml_files has to be set to True\n")
        sys.exit(1)

    if not os.path.exists(robot_xacro) or not os.path.isfile(robot_xacro):
        sys.stderr.write("The provided robot XACRO file %s doesn't exist.\n" % robot_xacro)
        sys.exit(2)

    if use_yaml_files and len(yaml_files) == 0:
        sys.stderr.write("use_yaml_files is True, but no YAML file was given.\n")
        sys.exit(3)

    if use_yaml_files:
        for i in range(len(yaml_files)):
            yaml_file = yaml_files[i]
            if len(yaml_file) == 0:
                sys.stderr.write('WARN: empty YAML file nr. %i\n' % i)
            else:
                if not os.path.exists(yaml_file) or not os.path.isfile(yaml_file):
                    sys.stderr.write("The provided robot YAML file %s doesn't exist.\n" % yaml_file)
                    sys.exit(4)

    # load ROS parameters if needed

    ros_params = None
    if use_ros_params:
        try:
            ros_params = rospy.get_param_names()
        except socket.error as e:
            # parameter server (roscore) not running
            sys.stderr.write("roscore is not running and use_ros_params is True")
            sys.exit(5)

    # read parts config
    parts_config = dict()
    rover_params = dict()
    omnicam_transforms = list()
    calibrated_joints = list()

    # 1) from YAML files
    if use_yaml_files:
        # read the config from robot.yaml

        # parse deg() and rad() the same way rosparam does
        yaml.add_constructor(u'!radians', construct_angle_radians)
        yaml.add_constructor(u'!degrees', construct_angle_degrees)

        # allow both !degrees 180, !radians 2*pi
        pattern = re.compile(r'^deg\([^\)]*\)$')
        yaml.add_implicit_resolver(u'!degrees', pattern, first="deg(")
        pattern = re.compile(r'^rad\([^\)]*\)$')
        yaml.add_implicit_resolver(u'!radians', pattern, first="rad(")

        # load config from all YAML files
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as robot_yaml:
                    robot_yaml_values = yaml.load(robot_yaml)
                if robot_yaml_values is None:
                    sys.stderr.write("WARN: File %s is empty.\n" % yaml_file)
                else:
                    parts_config.update(robot_yaml_values['parts_config'])

                    if "rover" in robot_yaml_values:
                        rover_params.update(robot_yaml_values["rover"])

                    if "omnicam_sensors_calib" in robot_yaml_values:
                        new_transforms = robot_yaml_values["omnicam_sensors_calib"]["transforms"]
                        if isinstance(new_transforms, list) or type(new_transforms) != type(omnicam_transforms):
                            omnicam_transforms = new_transforms
                        else:
                            omnicam_transforms.update(new_transforms)

                    # this key is deprecated
                    if "static_transforms" in robot_yaml_values:
                        new_transforms = robot_yaml_values["static_transforms"]
                        if isinstance(new_transforms, list) or type(new_transforms) != type(calibrated_joints):
                            calibrated_joints = new_transforms
                        else:
                            calibrated_joints.update(new_transforms)

                    if "calibrated_joints" in robot_yaml_values:
                        new_transforms = robot_yaml_values["calibrated_joints"]
                        if isinstance(new_transforms, list) or type(new_transforms) != type(calibrated_joints):
                            calibrated_joints = new_transforms
                        else:
                            calibrated_joints.update(new_transforms)

            except IOError as e:
                sys.stderr.write("Could not open %s. Error: %s\n" % (yaml_file, str(e)))
                sys.exit(7)

            sys.stderr.write("Robot parts config read from %s resulting in the following values: %s\n" % (
                yaml_file, sorted_dict_str(parts_config)))

    # 2) read all parts_config/ parameters from the parameter server
    if use_ros_params:
        namespace = rospy.get_namespace()
        parts_config_matcher = re.compile('^' + namespace + 'parts_config/')
        rover_matcher = re.compile('^' + namespace + 'rover/')
        for param in ros_params:
            if parts_config_matcher.match(param):
                value = rospy.get_param(param)
                # names of the params correspond to <arg> tags in the Xacro
                param_name = parts_config_matcher.sub('', param)
                parts_config[param_name] = value
            if rover_matcher.match(param):
                value = rospy.get_param(param)
                # names of the params correspond to <arg> tags in the Xacro
                param_name = rover_matcher.sub('', param)
                rover_params[param_name] = value

        sys.stderr.write("Robot parts config %s from parameter server with the following values: %s\n" % (
            "overridden" if use_yaml_files else "taken", sorted_dict_str(parts_config)))

        omnicam_transforms = rospy.get_param("omnicam_sensors_calib/transforms", [])
        calibrated_joints = rospy.get_param("calibrated_joints", [])
        # key 'static_transforms' is deprecated
        if len(calibrated_joints) == 0:
            calibrated_joints = rospy.get_param("static_transforms", [])

    # 3) if other arguments are provided, treat them as ROS key:=value stuff and
    # give them precedence over what is on the parameter server
    mappings = load_mappings(sys.argv)
    parts_config.update(mappings)

    # Xacro only accepts True and False bool values, not true/false or 0/1
    for key, value in parts_config.items():
        if value == "1" or value == "true":
            parts_config[key] = "True"
        elif value == "0" or value == "false":
            parts_config[key] = "False"

    if len(mappings.keys()) > 0:
        sys.stderr.write("Robot parts config overridden from print_robot_model.py CLI args %r resulting in the following values: %s\n" % (
            mappings, sorted_dict_str(parts_config)))

    transforms = dict()
    for transforms_list in (omnicam_transforms, calibrated_joints):
        if isinstance(transforms_list, list):
            for t in transforms_list:
                transforms[t[-1]] = t
        else:
            transforms.update(transforms_list)

    additional_xacro_args = node_params.get('additional_xacro_args', {})

    # Finally, render the given Xacro file

    try:
        print(render_xacro_file(robot_xacro, parts_config, rover_params, transforms, additional_xacro_args, prettyxml=pretty_xml))
    except Exception as e:
        sys.stderr.write("Error converting XACRO to URDF: %s, %r\n" % (str(e), e))
        sys.exit(9)

