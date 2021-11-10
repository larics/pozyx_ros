#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Quaternion


class PozyxMain(object):
    def __init__(self):
        # Load the parameters.
        tags = rospy.get_param('/pozyx/tags')
        anchors = rospy.get_param('/pozyx/anchors')
        desired_rate = rospy.get_param('~desired_rate')

        # Initialize the publishers.
        msg = Header()
        pubs = []
        for tag in tags:
            pubs.append(rospy.Publisher('{}/do_positioning'.format(tag['name']), Header, queue_size=1))

        # Send a static broadcast of anchors' TF frames.
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_tf = TransformStamped()
        for anchor in anchors:
            static_tf.header.stamp = rospy.Time.now()
            static_tf.header.frame_id = 'pozyx'
            static_tf.child_frame_id = anchor['id']
            static_tf.transform.translation.x = anchor['coordinates'][0] / 1000
            static_tf.transform.translation.y = anchor['coordinates'][1] / 1000
            static_tf.transform.translation.z = anchor['coordinates'][2] / 1000
            static_tf.transform.rotation = Quaternion(0, 0, 0, 1)
            broadcaster.sendTransform(static_tf)

        # And another one for transform between world and pozyx.
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = 'world'
        static_tf.child_frame_id = 'pozyx'
        static_tf.transform.translation.x = 0
        static_tf.transform.translation.y = 0
        static_tf.transform.translation.z = 0
        static_tf.transform.rotation = Quaternion(0, 0, 0, 1)
        broadcaster.sendTransform(static_tf)

        # Continously send requests for positioning to all tags.
        current_tag = 0
        rate = rospy.Rate(desired_rate * len(tags))
        while not rospy.is_shutdown():
            msg.stamp = rospy.get_rostime()
            pubs[current_tag].publish(msg)
            current_tag = (current_tag + 1) % len(tags)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("PozyxMain")

    try:
        node = PozyxMain()
    except rospy.ROSInterruptException:
        pass
