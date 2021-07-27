#!/usr/bin/env python

import rospy
import tf2_ros
import pypozyx
from pypozyx import *
from pypozyx.definitions.bitmasks import *
from pypozyx.definitions.registers import *
from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx.structures.device_information import DeviceDetails
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from sensor_msgs.msg import Imu
from numpy import matmul
import numpy as np

class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, osc_udp_client, tags, anchors,
                 algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                 dimension=PozyxConstants.DIMENSION_3D,
                 height=1000,
                 filter_type=PozyxConstants.FILTER_TYPE_FIR,
                 filter_strength = 2, ranging_protocol = 0):

        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.tag_id = tags[0]  # TODO: implement real multi-tag positioning.
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.filter_type = filter_type
        self.filter_strength = filter_strength
        self.ranging_protocol = ranging_protocol
        self.pub_position = rospy.Publisher('pozyx/measured', TransformStamped, queue_size=1)
        self.pub_imu = rospy.Publisher('pozyx/imu', Imu, queue_size=1)


        # Initialize the TF broadcaster.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.anchor_transforms = {}
        for anchor in self.anchors:
            anchor_str_id = "0x{:04X}".format(anchor.network_id)
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'pozyx'
            t.child_frame_id = anchor_str_id
            t.transform.translation.x = anchor.data[2] / 1000.0
            t.transform.translation.y = anchor.data[3] / 1000.0
            t.transform.translation.z = anchor.data[4] / 1000.0
            t.transform.rotation = Quaternion(0, 0, 0, 1)
            self.anchor_transforms[anchor_str_id] = t


    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")
        print(" - System will manually calibrate the tags")
        print("")
        print(" - System will then auto start positioning")
        print("")
        self.pozyx.printDeviceInfo()
        print("")
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.pozyx.setPositionAlgorithm(self.algorithm, self.dimension, self.tag_id)
        self.pozyx.setPositionFilter(self.filter_type, self.filter_strength, self.tag_id)
        self.pozyx.setRangingProtocol(self.ranging_protocol, self.tag_id)
        self.printPublishAnchorConfiguration()

    def test(self):
        data = DeviceDetails()
        pozyx.getRead(POZYX_WHO_AM_I, data, remote_id=self.tag_id)

        if not data[3] & POZYX_ST_RESULT_ACC:
            rospy.logerror("Self-test failed: ACCELEROMETER")
        if not data[3] & POZYX_ST_RESULT_MAGN:
            rospy.logerror("Self-test failed: MAGNETOMETER")
        if not data[3] & POZYX_ST_RESULT_GYR:
            rospy.logerror("Self-test failed: GYRO")
        if not data[3] & POZYX_ST_RESULT_MCU:
            rospy.logerror("Self-test failed: MCU")
        if not data[3] & POZYX_ST_RESULT_PRES:
            rospy.logerror("Self-test failed: PRESSURE")
        if not data[3] & POZYX_ST_RESULT_UWB:
            rospy.logerror("Self-test failed: UWB")
        if data[3] != 0b111111:
            quit()

        if data[4] != 0:
            rospy.logerror("Self-test error: 0x%0.2x", data[4])
            quit()

        print("SELF-TEST PASSED!")

    def loop(self):
        """Performs positioning and prints the results."""
        pwc = TransformStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = 'pozyx'
        pwc.transform.rotation =  pypozyx.Quaternion()        
        
        position = Coordinates()
        status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, self.tag_id)

        if status == POZYX_SUCCESS:
            # Package Pozyx's orientation.
            self.pozyx.getQuaternion(pwc.transform.rotation)

            # Package Pozyx's position.
            pwc.child_frame_id = 'tag'
            pwc.transform.translation.x = position.x * 0.001
            pwc.transform.translation.y = position.y * 0.001
            pwc.transform.translation.z = position.z * 0.001

            # Publish Pozyx's pose and transform.
            self.pub_position.publish(pwc)
            self.tf_broadcaster.sendTransform(pwc)

        else:
            self.printPublishErrorCode("positioning", self.tag_id)

        # Package and send Pozyx's IMU data.
        imu = Imu()
        imu.header.stamp = rospy.get_rostime()
        imu.header.frame_id = 'pozyx'
        imu.orientation =  pypozyx.Quaternion()
        imu.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        imu.angular_velocity = pypozyx.AngularVelocity()
        imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        imu.linear_acceleration = pypozyx.LinearAcceleration()
        imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        self.pozyx.getQuaternion(imu.orientation)
        self.pozyx.getAngularVelocity_dps(imu.angular_velocity)
        self.pozyx.getLinearAcceleration_mg(imu.linear_acceleration)

        #Convert from mg to m/s2
        imu.linear_acceleration.x = imu.linear_acceleration.x * 0.0098
        imu.linear_acceleration.y = imu.linear_acceleration.y * 0.0098
        imu.linear_acceleration.z = imu.linear_acceleration.z * 0.0098 + 9.81

        #Convert from Degree/second to rad/s
        imu.angular_velocity.x = imu.angular_velocity.x * 0.01745
        imu.angular_velocity.y = imu.angular_velocity.y * 0.01745
        imu.angular_velocity.z = imu.angular_velocity.z * 0.01745

        self.pub_imu.publish(imu)

        # Send anchor transforms
        for transform in self.anchor_transforms.values():
            transform.header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(transform)


    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error_%s" % operation, [network_id, error_code[0]])
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            print("Error % s, local error code %s" % (operation, str(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(self.tag_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.tag_id)
        if len(anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(anchors),
                                                           remote_id=self.tag_id)
        # enable these if you want to save the configuration to the devices.
        if save_to_flash:
            self.pozyx.saveAnchorIds(self.tag_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], self.tag_id)

        self.printPublishConfigurationResult(status, self.tag_id)

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                sleep(0.025)

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)


if __name__ == "__main__":

    rospy.init_node('pozyx_ros')

    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # Shortcut to not have to find out the port yourself.
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # Get all ROS parameters.
    config = {}
    config['height']    = int(rospy.get_param('~height', 1000))
    config['algorithm'] = int(rospy.get_param('~algorithm', 4))
    config['dimension'] = int(rospy.get_param('~dimension', 3))
    config['filter_type']      = int(rospy.get_param('~filter_type', 1))
    config['filter_strength']  = int(rospy.get_param('~filter_strength', 2))
    config['ranging_protocol'] = int(rospy.get_param('~ranging_protocol', 2))

    # Get anchor positions.
    anchors = []
    for anchor in rospy.get_param('~anchors'):
        anchors.append(DeviceCoordinates(int(anchor['id'], 16), 1, Coordinates(*anchor['coordinates'])))
        
    # IDs of the tags to position, add None to position the local tag as well.
    tags = [None]

    pozyx = PozyxSerial(serial_port)

    mtp = MultitagPositioning(pozyx, None, tags, anchors, **config)
    mtp.setup()
    mtp.test()

    frequency = int(rospy.get_param('~frequency', 50))
    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        mtp.loop()
        rate.sleep()
