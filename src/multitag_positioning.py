#!/usr/bin/env python

import rospy
import pypozyx
from pypozyx import *
from pypozyx.tools.version_check import perform_latest_version_check
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from numpy import matmul
import numpy as np

class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx):
        self.pozyx = pozyx
        
        self.pub_position = rospy.Publisher('pozyx/measured', TransformStamped, queue_size=1)

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")
        self.pozyx.printDeviceInfo()
        print("")
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")

    def loop(self):
        """Performs positioning and prints the results."""
        pwc = TransformStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = 'pozyx'
        position = Coordinates()
        pwc.transform.rotation =  pypozyx.Quaternion()

        status = self.pozyx.getCoordinates(position)

        if status == POZYX_SUCCESS:
            self.pozyx.getQuaternion(pwc.transform.rotation)

            #matrix = np.matrix([[-0.272008, 0.962232, 0.010988], [-0.962246, -0.272092, 0.007021], [0.009745, -0.008663, 0.999915]])
            vector = np.matrix([[position.x * 0.001], [position.y * 0.001], [position.z * 0.001]])

            #novi_vector = matmul(matrix,vector)

            pwc.transform.translation.x = vector.item(0)
            pwc.transform.translation.y = vector.item(1)
            pwc.transform.translation.z = vector.item(2)

            self.pub_position.publish(pwc)
        else:
            self.printPublishErrorCode("positioning", self.tag_id)


    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            print("Error % s, local error code %s" % (operation, str(error_code)))

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                sleep(0.025)


if __name__ == "__main__":

    rospy.init_node('pozyx_ros')

    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself.
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
        
    frequency = int(rospy.get_param('~frequency', 50))
    
    pozyx = PozyxSerial(serial_port)

    r = MultitagPositioning(pozyx)
    r.setup()

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        r.loop()
        rate.sleep()
