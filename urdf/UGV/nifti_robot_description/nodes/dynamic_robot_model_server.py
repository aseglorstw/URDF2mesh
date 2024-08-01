#!/usr/bin/env python

import os
import re
from threading import Thread

import rospy
import rospkg
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.encoding import encode_description

from nifti_robot_description.cfg import RobotPartsConfig

from nifti_robot_description.render_xacro import render_xacro_file

from tf.transformations import euler_from_quaternion, quaternion_from_euler


class DynamicRobotModelServer(object):
    """
    Dynamic reconfigure server handling the robot model generation.

    Before running this server, load the robot.yaml params to the ROS param server.
    This script then reads all `parts_config/*` entries on the param server and uses them to configure the model from
    urdf/nifti_robot.xacro (their names should correspond to names in the <arg> tags in the xacro file).

    All parts of the TRADR system for which it makes sense to somehow alter their state whenever a change to the robot
    body is done should not directly read the /nifti_robot_description and /parts_config/* values, but should instead
    create a dynamic reconfigure client and receive updates to the parameters on the fly.

    IMPLEMENTATION NOTE:
    This node uses a hack to dynamic_reconfigure to allow not only for dynamic values, but also for dynamic list of
    parameters. The implementation should work flawlessly in Python, but it seems that C++ support would be a bit
    more difficult and it has not been done yet. The core part of the hack is in method _augment_config_type.
    """

    def __init__(self):
        """
        Set up the dynamic model server, read params from param server, and generate an initial robot model.
        """
        super(DynamicRobotModelServer, self).__init__()

        self.__float_range = (-10.0, 10.0)
        self.__int_range = (-10000, 10000)

        xacro_file = os.path.join(rospkg.RosPack().get_path('nifti_robot_description'), 'urdf', 'nifti_robot.xacro')
        self._xacro_file = rospy.get_param('~xacro_file', xacro_file)
        self._additional_xacro_args = rospy.get_param('~additional_xacro_args', {})

        self._tf_publisher_client = None
        self._tf_publisher_client_init_thread = Thread(target=self._load_tf_publisher_client)
        self._tf_publisher_client_init_thread.start()

        self._last_config = None
        self._parts_config_cache = None

        self._on_change_callbacks = dict()  # tuple[changed param names]: function with parameter config

        self._augment_config_type(RobotPartsConfig)
        self._server = Server(RobotPartsConfig, self.change_callback)

        self._last_config = self._server.config

    def _load_tf_publisher_client(self):
        try:
            self._tf_publisher_client = Client("nifti_tf_publisher")
            rospy.loginfo("TF publisher client initialized")
        except rospy.ROSInterruptException:
            pass

    def change_callback(self, config, level):
        """
        Called whenever the config is changed externally.

        :param dynamic_reconfigure.Config config: The new config
        :param int level: Not used
        :return: An updated config (value errors corrected etc.)
        :rtype: dynamic_reconfigure.Config
        """

        try:
            calibrated_joints = dict()
            for param in ("omnicam_sensors_calib/transforms", "static_transforms", "calibrated_joints"):
                transforms = self._get_calibrated_joints(param, convert_to_list=False)
                if isinstance(transforms, list):
                    transforms_dict = dict(zip([t[-1] for t in transforms], transforms))
                else:
                    transforms_dict = transforms
                try:
                    for name, transform in transforms_dict.iteritems():
                        transform[0] = getattr(config, name + "_pos_x")
                        transform[1] = getattr(config, name + "_pos_y")
                        transform[2] = getattr(config, name + "_pos_z")
                        if len(transform) == 8:
                            transform[3] = getattr(config, name + "_rpy_r")
                            transform[4] = getattr(config, name + "_rpy_p")
                            transform[5] = getattr(config, name + "_rpy_y")
                        elif len(transform) == 9:
                            transform[3] = getattr(config, name + "_quat_x")
                            transform[4] = getattr(config, name + "_quat_y")
                            transform[5] = getattr(config, name + "_quat_z")
                            transform[6] = getattr(config, name + "_quat_w")
                        calibrated_joints[name] = transform
                except KeyError, e:
                    rospy.logerr("Could not add static transform: " + str(e))
                rospy.set_param(param, transforms)

            rover_params = rospy.get_param("rover")
            robot_description = render_xacro_file(self._xacro_file, config, rover_params, calibrated_joints, self._additional_xacro_args)

            config.robot_description = robot_description
            rospy.set_param('nifti_robot_description', robot_description)

            rospy.loginfo("Robot model updated, it has %i bytes." % len(robot_description))

            for key in self._get_parts_config().keys():
                value = getattr(config, key)
                rospy.set_param("parts_config/%s" % key, value)
                self._parts_config_cache[key] = value

            # send the update also to nifti_tf_publisher which uses a different
            # dynreconf message
            if self._tf_publisher_client is not None:
                self._tf_publisher_client.update_configuration(
                    {'robot_description': robot_description})

            # on change callbacks are not called on the initial change during _server construction
            if getattr(self, "_server", None) is not None:
                for names, callback in self._on_change_callbacks.iteritems():
                    trigger = False
                    for name in names:
                        if getattr(config, name) != getattr(self._last_config, name):
                            trigger = True
                            break
                    if trigger:
                        callback(config)

            self._last_config = config

        except Exception, e:
            rospy.logerr('XACRO to URDF conversion did not succeed: %s, %r' % (str(e), e))
            if self._last_config is not None:
                config = self._last_config

        return config

    def _augment_config_type(self, config_type):
        """
        Main part of the hack that allows us to have a dynamic list of dynamic parameters.

        It simulates what the compiled `*Config.py` files do, but the list of parameters is not statically written, it is
        instead built from all parameters found in the `parts_config/*` section on the parameter server.

        :param type config_type: The `*Config` base type which will be extended.
        """
        config_type.all_level = 0

        parts_config = self._get_parts_config(use_cache=False)
        parts_keys = list(parts_config.keys())
        parts_keys.sort()

        for key in parts_keys:
            value = parts_config[key]

            value_type = type(value)

            # a corner case which is often present in our robot.yaml configs
            if key.startswith("has_") and (value_type == float or value_type == int):
                value_type = bool
                value = True if value != 0.0 else False
                rospy.logwarn("You should update your robot.yaml file so that '%s' has a 'True' or 'False' as value, "
                              "and not integers/floats." % key)

            self._add_parameter(config_type, config_type.config_description, key, value, value_type)

        group_id = 1
        omnicam_metagroup = self._create_group("omnicam", group_id)
        for cam in self._get_calibrated_joints('omnicam_sensors_calib/transforms'):
            group_id += 1
            cam_name = cam[-1]
            cam_group = self._create_group(cam_name, group_id, parent_group_id=1, parent_group_name="omnicam")
            self._add_transform_parameters(config_type, cam_group, cam_name, cam)
            omnicam_metagroup['groups'].append(cam_group)
        config_type.config_description['groups'].append(omnicam_metagroup)

        calibrated_joints = self._get_calibrated_joints('calibrated_joints', convert_to_list=False)
        # key 'static_transforms' is deprecated
        if len(calibrated_joints) == 0:
            calibrated_joints = self._get_calibrated_joints('static_transforms', convert_to_list=False)
        if isinstance(calibrated_joints, list):
            calibrated_joints = dict(zip([j[-1] for j in calibrated_joints], calibrated_joints))
        for transform_name, transform in calibrated_joints.iteritems():
            group_id += 1
            transform_group = self._create_group(transform_name, group_id)
            self._add_transform_parameters(config_type, transform_group, transform_name, transform)
            config_type.config_description['groups'].append(transform_group)

    def _add_parameter(self, config_type, container, param_name, value, value_type=None, const=False):
        if value_type is None:
            value_type = type(value)
        elif not isinstance(value, value_type):
            raise RuntimeError("Value %r (type %s) is not an instance of type %s" % (value, type(value), value_type))

        description = self._get_parameter_description(param_name, value, value_type, const)
        container['parameters'].append(description)
        config_type.level[param_name] = 0
        config_type.defaults[param_name] = value
        config_type.min[param_name] = description['min']
        config_type.max[param_name] = description['max']
        config_type.type[param_name] = description['type']
        setattr(config_type, param_name, value)

    @staticmethod
    def _create_group(name, group_id, parent_group_id=0, parent_group_name="Default", group_type='tab'):
        return {
            'upper': name.upper(),
            'lower': name.lower(),
            'name': name,
            'parent': parent_group_id,
            'parentname': parent_group_name,
            'groups': [],
            'parameters': [],
            'type': group_type,
            'id': group_id,
            'state': True
        }

    def _get_parameter_description(self, key, value, value_type, const=False):
        description = {
            'description': key,
            'name': key,
            'edit_method': '',
            'level': 0,
            'type': value_type.__name__,
            'default': value,
        }

        if value_type == bool:
            description['min'] = value if const else False
            description['max'] = value if const else True

        elif value_type == float:
            description['min'] = value if const else self.__float_range[0]
            description['max'] = value if const else self.__float_range[1]
            description['type'] = 'double'

        elif value_type == int:
            description['min'] = value if const else self.__int_range[0]
            description['max'] = value if const else self.__int_range[1]

        elif value_type == str:
            description['min'] = value if const else ''
            description['max'] = value if const else ''

        else:
            raise RuntimeError("Unknown parameter type: " + str(value_type))

        return description

    def _add_transform_parameters(self, config_type, group, name, transform):
        self._add_parameter(config_type, group, name + "_pos_x", transform[0], float)
        self._add_parameter(config_type, group, name + "_pos_y", transform[1], float)
        self._add_parameter(config_type, group, name + "_pos_z", transform[2], float)

        quat_x_name = name + "_quat_x"
        quat_y_name = name + "_quat_y"
        quat_z_name = name + "_quat_z"
        quat_w_name = name + "_quat_w"
        rpy_r_name = name + "_rpy_r"
        rpy_p_name = name + "_rpy_p"
        rpy_y_name = name + "_rpy_y"

        if len(transform) == 9:
            self._add_parameter(config_type, group, quat_x_name, transform[3], float)
            self._add_parameter(config_type, group, quat_y_name, transform[4], float)
            self._add_parameter(config_type, group, quat_z_name, transform[5], float)
            self._add_parameter(config_type, group, quat_w_name, transform[6], float)
            rpy = euler_from_quaternion(transform[3:7])
            self._add_parameter(config_type, group, rpy_r_name, rpy[0], float, const=True)
            self._add_parameter(config_type, group, rpy_p_name, rpy[1], float, const=True)
            self._add_parameter(config_type, group, rpy_y_name, rpy[2], float, const=True)

            def on_change_cb(config):
                quat = (
                    getattr(config, quat_x_name), getattr(config, quat_y_name),
                    getattr(config, quat_z_name), getattr(config, quat_w_name)
                )
                rpy = euler_from_quaternion(quat)
                config_type.min[rpy_r_name] = config_type.max[rpy_r_name] = config_type.defaults[rpy_r_name] = rpy[0]
                config_type.min[rpy_p_name] = config_type.max[rpy_p_name] = config_type.defaults[rpy_p_name] = rpy[1]
                config_type.min[rpy_y_name] = config_type.max[rpy_y_name] = config_type.defaults[rpy_y_name] = rpy[2]
                setattr(config, rpy_r_name, rpy[0])
                setattr(config, rpy_p_name, rpy[1])
                setattr(config, rpy_y_name, rpy[2])
                group["parameters"][7]["min"] = group["parameters"][7]["max"] = rpy[0]
                group["parameters"][8]["min"] = group["parameters"][8]["max"] = rpy[1]
                group["parameters"][9]["min"] = group["parameters"][9]["max"] = rpy[2]
                self._server.description = encode_description(config_type)
                self._server.descr_topic.publish(self._server.description)

            names = (quat_x_name, quat_y_name, quat_z_name, quat_w_name)
            callback = on_change_cb
            self._on_change_callbacks[names] = callback
        elif len(transform) == 8:
            self._add_parameter(config_type, group, rpy_r_name, transform[3], float)
            self._add_parameter(config_type, group, rpy_p_name, transform[4], float)
            self._add_parameter(config_type, group, rpy_y_name, transform[5], float)
            quat = quaternion_from_euler(*transform[3:6]).tolist()
            self._add_parameter(config_type, group, quat_x_name, quat[0], float, const=True)
            self._add_parameter(config_type, group, quat_y_name, quat[1], float, const=True)
            self._add_parameter(config_type, group, quat_z_name, quat[2], float, const=True)
            self._add_parameter(config_type, group, quat_w_name, quat[3], float, const=True)

            def on_change_cb(config):
                rpy = (
                    getattr(config, rpy_r_name), getattr(config, rpy_p_name),
                    getattr(config, rpy_y_name)
                )
                quat = quaternion_from_euler(*rpy).tolist()
                config_type.min[quat_x_name] = config_type.max[quat_x_name] = config_type.defaults[quat_x_name] = quat[0]
                config_type.min[quat_y_name] = config_type.max[quat_y_name] = config_type.defaults[quat_y_name] = quat[1]
                config_type.min[quat_z_name] = config_type.max[quat_z_name] = config_type.defaults[quat_z_name] = quat[2]
                config_type.min[quat_w_name] = config_type.max[quat_w_name] = config_type.defaults[quat_w_name] = quat[3]
                setattr(config, quat_x_name, quat[0])
                setattr(config, quat_y_name, quat[1])
                setattr(config, quat_z_name, quat[2])
                setattr(config, quat_w_name, quat[3])
                group["parameters"][6]["min"] = group["parameters"][6]["max"] = quat[0]
                group["parameters"][7]["min"] = group["parameters"][7]["max"] = quat[1]
                group["parameters"][8]["min"] = group["parameters"][8]["max"] = quat[2]
                group["parameters"][9]["min"] = group["parameters"][9]["max"] = quat[3]
                self._server.description = encode_description(config_type)
                self._server.descr_topic.publish(self._server.description)

            names = (rpy_r_name, rpy_p_name, rpy_y_name)
            callback = on_change_cb
            self._on_change_callbacks[names] = callback

    def _get_parts_config(self, use_cache=True):
        """
        Read all `parts_config/*` entries from the parameter server and return a dict, where keys are the part names
        (without the "namespace" prefix).

        :return: All parts configuration.
        :rtype: dict
        """
        if use_cache and self._parts_config_cache is not None:
            return self._parts_config_cache

        namespace = rospy.get_namespace()
        parts_config_matcher = re.compile('^' + namespace + 'parts_config/')
        parts_config = {}
        for param in rospy.get_param_names():
            if parts_config_matcher.match(param):
                value = rospy.get_param(param)
                param_name = parts_config_matcher.sub('', param)
                parts_config[param_name] = value

        self._parts_config_cache = parts_config
        return parts_config

    @staticmethod
    def _get_calibrated_joints(param_name, convert_to_list=True):
        """
        Read all `calibrated_joints/*` entries from the parameter server and return a dict, where keys are
        child link names.

        :param str param_name: Name of the ROS parameter where to find the transforms.
        :param bool convert_to_list: If True, the transforms will be changed to list even when they are a dict on the param server.

        :return: All static transforms.
        :rtype: dict|list
        """
        namespace = rospy.get_namespace()
        transforms = rospy.get_param('%s%s' % (namespace, param_name), [])

        if convert_to_list and isinstance(transforms, dict):
            transforms = transforms.values()

        return transforms


if __name__ == '__main__':
    rospy.init_node('dynamic_robot_model_server')
    server = DynamicRobotModelServer()
    rospy.spin()
