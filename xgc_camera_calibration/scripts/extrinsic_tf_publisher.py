#!/usr/bin/env python3

import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from xgc_camera_calibration.solver import load_extrinsic
from xgc_camera_calibration.transforms import split_parent_to_optical_pose


def make_transform(parent_frame, child_frame, translation, quaternion):
    message = TransformStamped()
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


def main():
    rospy.init_node("xgc_camera_extrinsic_tf")
    extrinsic_file = rospy.get_param("~extrinsic_file", "")
    if not extrinsic_file:
        rospy.logfatal("~extrinsic_file is required and must point to a runtime calibration asset")
        return 2
    try:
        document = load_extrinsic(extrinsic_file)
        parent_frame = rospy.get_param("~parent_frame", document.get("parent_frame", "map"))
        optical_frame = rospy.get_param(
            "~optical_frame", document.get("child_frame", "usb_cam_optical_frame")
        )
        camera_link_frame = rospy.get_param("~camera_link_frame", "usb_cam_link")
        offsets = tuple(
            float(rospy.get_param("~{}_offset".format(axis), 0.0)) for axis in ("x", "y", "z")
        )
        optical_translation = document["translation_array"] + offsets
        chain = split_parent_to_optical_pose(
            optical_translation, document["quaternion_xyzw_array"]
        )
        parent_to_link = make_transform(
            parent_frame,
            camera_link_frame,
            chain["parent_t_link"],
            chain["parent_q_link_xyzw"],
        )
        link_to_optical = make_transform(
            camera_link_frame,
            optical_frame,
            chain["link_t_optical"],
            chain["link_q_optical_xyzw"],
        )
    except Exception as error:
        rospy.logfatal("Could not load camera extrinsic %s: %s", extrinsic_file, error)
        return 1

    static = bool(rospy.get_param("~static", True))
    rospy.loginfo(
        "Publishing camera extrinsic chain %s -> %s -> %s from %s",
        parent_frame,
        camera_link_frame,
        optical_frame,
        extrinsic_file,
    )
    if static:
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        stamp = rospy.Time.now()
        parent_to_link.header.stamp = stamp
        link_to_optical.header.stamp = stamp
        broadcaster.sendTransform([parent_to_link, link_to_optical])
        rospy.spin()
        return 0

    broadcaster = tf2_ros.TransformBroadcaster()
    optical_broadcaster = tf2_ros.StaticTransformBroadcaster()
    link_to_optical.header.stamp = rospy.Time.now()
    optical_broadcaster.sendTransform(link_to_optical)
    rate = rospy.Rate(float(rospy.get_param("~publish_rate", 10.0)))
    while not rospy.is_shutdown():
        parent_to_link.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(parent_to_link)
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
