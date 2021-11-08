#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros

import pypozyx
from pypozyx import PozyxConstants, POZYX_SUCCESS
from pypozyx.definitions.bitmasks import *
from pypozyx.definitions.registers import *
from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx.structures.device_information import DeviceDetails

from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, TriggerResponse


class SynchronizedPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, osc_udp_client, tags, anchors,
                 algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                 dimension=PozyxConstants.DIMENSION_3D,
                 height=1000,
                 filter_type=PozyxConstants.FILTER_TYPE_FIR,
                 filter_strength=2, ranging_protocol=0):

        # Set class variables.
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.name = rospy.get_namespace()
        self.tag_id = tags[0]
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.filter_type = filter_type
        self.filter_strength = filter_strength
        self.ranging_protocol = ranging_protocol

        self.pub_imu = rospy.Publisher('pozyx/imu', Imu, queue_size=1)
        self.pub_position = rospy.Publisher('pozyx/measured', PoseWithCovarianceStamped, queue_size=1)

        # Initialize the TF broadcaster.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Set up a service for running the positioning event.
        rospy.Service('do_positioning', Trigger, self.loop)

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(pypozyx.version))
        print("")
        print(" - System will manually calibrate the tags")
        print("")
        print(" - System will then auto start positioning")
        print("")
        self.pozyx.printDeviceInfo()
        print("")
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(pypozyx.version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.pozyx.setPositionAlgorithm(self.algorithm, self.dimension, self.tag_id)
        self.pozyx.setPositionFilter(self.filter_type, self.filter_strength, self.tag_id)
        self.pozyx.setRangingProtocol(self.ranging_protocol, self.tag_id)
        self.printPublishAnchorConfiguration()

    def test(self):
        # FIXME: self-test works only for the connected tag, not remote
        # Even tough positioning works normally
        data = DeviceDetails()
        self.pozyx.getRead(POZYX_WHO_AM_I, data, remote_id=None)

        if not data[3] & POZYX_ST_RESULT_ACC:
            rospy.logerr("Self-test failed: ACCELEROMETER")
        if not data[3] & POZYX_ST_RESULT_MAGN:
            rospy.logerr("Self-test failed: MAGNETOMETER")
        if not data[3] & POZYX_ST_RESULT_GYR:
            rospy.logerr("Self-test failed: GYRO")
        if not data[3] & POZYX_ST_RESULT_MCU:
            rospy.logerr("Self-test failed: MCU")
        if not data[3] & POZYX_ST_RESULT_PRES:
            rospy.logerr("Self-test failed: PRESSURE")
        if not data[3] & POZYX_ST_RESULT_UWB:
            rospy.logerr("Self-test failed: UWB")
        if data[3] != 0b111111:
            quit()

        if data[4] != 0:
            rospy.logerr("Self-test error: 0x%0.2x", data[4])
            quit()

        print("SELF-TEST PASSED!")

    def loop(self, req):
        """Performs positioning and prints the results."""
        position = pypozyx.Coordinates()
        status_p = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, self.tag_id)

        position_error = pypozyx.PositionError()
        status_e = self.pozyx.getPositionError(position_error, remote_id=self.tag_id)

        if status_p == POZYX_SUCCESS:
            # Prepare a ROS message for current position.
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = 'pozyx'
            # Package Pozyx's orientation.
            pose.pose.pose.orientation = pypozyx.Quaternion()
            self.pozyx.getQuaternion(pose.pose.pose.orientation)
            # Package Pozyx's position.
            pose.header.child_frame_id = self.name
            pose.pose.pose.position.x = position.x * 0.001
            pose.pose.pose.position.y = position.y * 0.001
            pose.pose.pose.position.z = position.z * 0.001
            # Get covariances.
            pose.pose.covariance[:6] = map(lambda x: x * 0.001, position_error.data)
            # Publish Pozyx's pose.
            self.pub_position.publish(pose)

            # Prepare a ROS message for transform.
            tf = TransformStamped()
            tf.header = pose.header
            tf.child_frame_id = self.name
            # Package Pozyx's orientation.
            tf.transform.rotation = pose.pose.pose.orientation
            # Package Pozyx's position.
            tf.transform.translation.x = pose.pose.pose.position.x
            tf.transform.translation.y = pose.pose.pose.position.y
            tf.transform.translation.z = pose.pose.pose.position.z
            # Broadcast Pozyx's transform.
            self.tf_broadcaster.sendTransform(tf)

            # # Prepare a ROS message for current IMU readings.
            # imu = Imu()
            # imu.header.stamp = rospy.get_rostime()
            # imu.header.frame_id = 'pozyx'
            # imu.orientation = pypozyx.Quaternion()
            # imu.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            # imu.angular_velocity = pypozyx.AngularVelocity()
            # imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            # imu.linear_acceleration = pypozyx.LinearAcceleration()
            # imu.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            # # Package IMU data.
            # self.pozyx.getQuaternion(imu.orientation)
            # self.pozyx.getAngularVelocity_dps(imu.angular_velocity)
            # self.pozyx.getLinearAcceleration_mg(imu.linear_acceleration)
            # # Convert from mg to m/s2
            # imu.linear_acceleration.x = imu.linear_acceleration.x * 0.0098
            # imu.linear_acceleration.y = imu.linear_acceleration.y * 0.0098
            # imu.linear_acceleration.z = imu.linear_acceleration.z * 0.0098 + 9.81
            # # Convert from Degree/second to rad/s
            # imu.angular_velocity.x = imu.angular_velocity.x * 0.01745
            # imu.angular_velocity.y = imu.angular_velocity.y * 0.01745
            # imu.angular_velocity.z = imu.angular_velocity.z * 0.01745
            # # Publish the message.
            # self.pub_imu.publish(imu)

            self.printPosition(position, self.tag_id)
            return TriggerResponse(True, '')
        else:
            error = self.printPublishErrorCode("positioning", self.tag_id)
            return TriggerResponse(False, error)

    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = pypozyx.SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            error_text = "Error %s on ID %s, %s" % \
                         (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code))
            print(error_text)

            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error_%s" % operation, [network_id, error_code[0]])

            return error_text
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            error_text = "Error % s, local error code %s" % (operation, str(error_code))
            print(error_text)

            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])

            return error_text

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""

        status = self.pozyx.clearDevices(self.tag_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.tag_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.tag_id)
        self.printPublishConfigurationResult(status, self.tag_id)

        # Enable this if you want to save the configuration to the devices.
        if save_to_flash:
            self.pozyx.saveAnchorIds(self.tag_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], self.tag_id)

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                pypozyx.sleep(0.025)

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)

    def printPosition(self, position, tag_id):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        if tag_id is None:
            tag_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % tag_id, pos=position))


def main():
    # Initialize ROS node.
    rospy.init_node('pozyx_ros')

    # Get all ROS parameters.
    config = {
        'height':           int(rospy.get_param('~height', 1000)),
        'algorithm':        int(rospy.get_param('~algorithm', 4)),
        'dimension':        int(rospy.get_param('~dimension', 3)),
        'filter_type':      int(rospy.get_param('~filter_type', 1)),
        'filter_strength':  int(rospy.get_param('~filter_strength', 2)),
        'ranging_protocol': int(rospy.get_param('~ranging_protocol', 2))
    }

    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # Shortcut to not have to find out the port yourself.
    serial_port = pypozyx.get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # Get anchor positions.
    anchors = []
    for anchor in rospy.get_param('/pozyx/anchors'):
        anchors.append(pypozyx.DeviceCoordinates(int(anchor['id'], 16), 1, pypozyx.Coordinates(*anchor['coordinates'])))

    tags = [None]

    # Create PyPozyx object and object for positioning.
    pozyx = pypozyx.PozyxSerial(serial_port)

    mtp = SynchronizedPositioning(pozyx, None, tags, anchors, **config)
    mtp.setup()
    mtp.test()

    rospy.spin()


if __name__ == "__main__":
    main()
