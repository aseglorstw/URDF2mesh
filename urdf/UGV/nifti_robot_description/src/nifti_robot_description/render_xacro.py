import os
import tempfile
import sys
from xml import dom

import rospkg
import xacro

from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply


def _remove_comments_from_xml_doc(doc):
    """
    Remove all XML comments from the given parsed document.

    :param xml.doc.minidom.Document doc: The document to remove comments from.
    """
    for tag in doc.getElementsByTagName("*"):
        for n in tag.childNodes:
            if n.nodeType is dom.Node.COMMENT_NODE:
                n.parentNode.removeChild(n)


def collect_payload_xacros():
    """
    Search through all available ROS packages for those that define a TRADR
    robot payload component.

    Such packages have to `exec_depend` on this package and export this tag:
    `<nifti_robot_description payload_xacro="${prefix}/path/to.xacro" />`

    :return: List of absolute paths to payload xacro files.
    :rtype: list
    """
    payload_xacros = []
    payload_packages = []
    this_package = rospkg.get_package_name(__file__)
    rp = rospkg.RosPack()
    for p in rp.get_depends_on(this_package, implicit=False):
        manifest = rp.get_manifest(p)
        for e in manifest.exports:
            try:
                if e.__dict__['tag'] == this_package:
                    payload_packages.append(p)
                    xacro_file = e.get('payload_xacro')
                    if xacro_file:
                        xacro_file = xacro_file.replace('${prefix}', rp.get_path(p))
                    payload_xacros.append(xacro_file)
            except Exception as e:
                sys.stderr.write(str(e) + "\n")

    if len(payload_packages) > 0:
        sys.stderr.write("Found the following TRADR robot payload packages: %r\n" % (payload_packages,))
    else:
        sys.stderr.write("WARN: Found no TRADR robot payload packages!")

    return payload_xacros


def process_rover_params(rover_params):
    """
    Parse rover parameters from robot config and return them as a xacro snippet
    exposing these parameters for use in nifti_robot.xacro.

    :return: XML fragment with Xacro arg tags
    :rtype: str
    """
    xml = '<?xml version="1.0"?>'
    xml += '<root xmlns:xacro="http://www.ros.org/wiki/xacro">'
    for param in rover_params:
        value = rover_params[param]
        xml += '<xacro:arg name="rover_%s" default="%f" />' % (param, value)
    xml += '</root>'

    return xml


def process_calibrated_joints(calibrated_joints):
    """
    Parse the given static transforms and turn them into a xacro snippet
    exposing these transforms as parameters for use in nifti_robot.xacro.

    :param dict calibrated_joints: Dict of 8- or 9-element lists.

    :return: XML fragment with Xacro arg tags
    :rtype: str
    """
    xml = '<?xml version="1.0"?>'
    xml += '<root xmlns:xacro="http://www.ros.org/wiki/xacro">'
    for name, transform in calibrated_joints.iteritems():
        pos = transform[0:3]
        if len(transform) == 8:
            rpy = transform[3:6]
            quat = quaternion_from_euler(*rpy).tolist()
        elif len(transform) == 9:
            quat = transform[3:7]
            rpy = euler_from_quaternion(quat)
        else:
            raise RuntimeError("Unexpected transform: %r" % (transform, ))

        # gazebo has cameras with x pointing into image... :(
        gzrpy = euler_from_quaternion(quaternion_multiply(
            quat,
            quaternion_from_euler(pi/2, -pi/2, 0)
        ))

        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_pos_x", pos[0])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_pos_y", pos[1])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_pos_z", pos[2])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_rpy_r", rpy[0])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_rpy_p", rpy[1])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_rpy_y", rpy[2])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_gzrpy_r", gzrpy[0])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_gzrpy_p", gzrpy[1])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_gzrpy_y", gzrpy[2])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_quat_x", quat[0])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_quat_y", quat[1])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_quat_z", quat[2])
        xml += '<xacro:arg name="%s" default="%f" />' % (name + "_quat_w", quat[3])

    xml += '</root>'

    return xml


def process_additional_xacro_args(additional_xacro_args):
    """
    Generate Xacro snippet with additional xacro arg tags.
    """
    xml = '<?xml version="1.0"?>'
    xml += '<root xmlns:xacro="http://www.ros.org/wiki/xacro">'
    for arg, value in additional_xacro_args.iteritems():
        xml += '<xacro:arg name="' + str(arg) + '" default="' + str(value) + '" />'
    xml += '</root>'

    return xml


def add_arg_tags_to_doc(doc, args_xml):
    """
    Add <xacro:arg> tags to the beginning of the given XML document.

    :param xml.dom.minidom.Document doc: The document to alter.
    :param str args_xml: The XML fragment to add.
    """
    subdoc = dom.minidom.parseString(args_xml)
    for arg in subdoc.documentElement.getElementsByTagName('xacro:arg'):
        doc.documentElement.insertBefore(arg, doc.documentElement.firstChild)


def render_xacro_file(xacro_file, config, rover_params, calibrated_joints, additional_xacro_args=None, prettyxml=False):
    """
    Render the given robot xacro file to a URDF string.
    :param str xacro_file: Path to the robot xacro file.
    :param dict config: Reconfigurable parts config.
    :param dict calibrated_joints: Dict of 8- or 9-element lists containing static transform definitions for calibrated joints.
    :param dict additional_xacro_args: Additional args to be added to the Xacro file. Keys are arg names, values are their defaults.
    :param bool prettyxml: If true, the resulting string will be pretty-formatted.
    :return: The configured URDF string.
    """
    doc = xacro.parse(None, xacro_file)
    _remove_comments_from_xml_doc(doc)

    rover_xml = process_rover_params(rover_params)
    add_arg_tags_to_doc(doc, rover_xml)

    calibrated_joints_xml = process_calibrated_joints(calibrated_joints)
    add_arg_tags_to_doc(doc, calibrated_joints_xml)

    if additional_xacro_args is not None:
        additional_xacro_args_xml = process_additional_xacro_args(additional_xacro_args)
        add_arg_tags_to_doc(doc, additional_xacro_args_xml)

    mappings = dict()
    mappings.update(config)
    mappings['payload_xacros'] = collect_payload_xacros()

    xacro.process_doc(doc, mappings=dict([(key, str(val)) for (key, val) in mappings.iteritems()]), in_order=True)

    return doc.toprettyxml() if prettyxml else doc.toxml()
