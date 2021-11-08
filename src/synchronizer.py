#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Trigger, TriggerRequest


class Synchronizer(object):
    def __init__(self):
        # IDs or names of the tags to position.
        tags = []
        for tag in rospy.get_param('/pozyx/tags'):
            if tag['name']:
                tags.append(tag['name'])
            elif tag['id']:
                tags.append(tag['id'])
            else:
                rospy.logfatal("Tag missing ID or name. Cannot proceed")

        # Service calls to trigger the positioning on each tag.
        self.triggers = {name: rospy.ServiceProxy('/{}/do_positioning'.format(name), Trigger, persistent=True)
                         for name in tags}

        desired_rate = rospy.get_param('~desired_rate', 10)
        rate = rospy.Rate(desired_rate)
        while not rospy.is_shutdown():
            start = rospy.get_time()
            for tag in tags:
                try:
                    do_positioning = rospy.ServiceProxy('{}/do_positioning'.format(tag), Trigger)
                    req = TriggerRequest()
                    resp = do_positioning(req)
                except (rospy.ServiceException, rospy.ROSException) as e:
                    rospy.logerr('Service call failed with error: %s', e)
            rospy.sleep(0.001)

            elapsed = rospy.get_time() - start
            if elapsed > 1 / desired_rate:
                rospy.logwarn("Loop took %.4f seconds. That is %.4f too much.", elapsed, elapsed - 1 / desired_rate)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("synchronizer")

    try:
        node = Synchronizer()
    except rospy.ROSInterruptException:
        pass
