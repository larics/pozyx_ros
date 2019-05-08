#!/usr/bin/env python

import rospy
import pypozyx
from pypozyx import *
from pypozyx.tools.version_check import perform_latest_version_check
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, osc_udp_client, tag_id, anchors, algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                 dimension=PozyxConstants.DIMENSION_3D, height=1000, filter_type=PozyxConstants.FILTER_TYPE_FIR, filter_strength = 2, ranging_protocol = 0):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.tag_id = tag_id
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.filter_type = filter_type
        self.filter_strength = filter_strength
        self.ranging_protocol = ranging_protocol
        self.pub_position = rospy.Publisher('pozyx/measured', TransformStamped, queue_size=1)
        self.pub_imu = rospy.Publisher('pozyx/imu', Imu, queue_size=1)

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

    def loop(self):
        """Performs positioning and prints the results."""
        pwc = TransformStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = 'pozyx'
        position = Coordinates()
        pwc.transform.rotation =  pypozyx.Quaternion()

        cov = pypozyx.PositionError()

        status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, self.tag_id)

        if status == POZYX_SUCCESS:
            self.pozyx.getQuaternion(pwc.transform.rotation)
            '''self.pozyx.getPositionError(cov)

            cov_row1 =[cov.x, cov.xy, cov.xz, 0, 0, 0]
            cov_row2 =[cov.xy, cov.y, cov.yz, 0, 0, 0]
            cov_row3 =[cov.xz, cov.yz, cov.z, 0, 0, 0]
            cov_row4 =[0, 0, 0, 0, 0, 0]
            cov_row5 =[0, 0, 0, 0, 0, 0]
            cov_row6 =[0, 0, 0, 0, 0, 0]

            pwc.pose.covariance = cov_row1 + cov_row2 + cov_row3 + cov_row4 + cov_row5 + cov_row6'''

            pwc.transform.translation.x = position.x * 0.001
            pwc.transform.translation.y = position.y * 0.001
            pwc.transform.translation.z = position.z * 0.001

            self.pub_position.publish(pwc)

            '''imu = Imu()
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

            self.pub_imu.publish(imu)'''

            #al = SingleRegister()
            #self.pozyx.getUpdateInterval(al)
            #print al.value

        else:
            self.printPublishErrorCode("positioning", self.tag_id)
            print "Nije dobro"



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

    # shortcut to not have to find out the port yourself.
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    number_of_anchors = int(rospy.get_param('~number_of_anchors', 4))
    anchor_id = []
    anchor_coordinates = []

    for i in range(number_of_anchors):
        anchor_id.append(int(rospy.get_param('~anchor' + str(i) + '_id', 0x6E7A), 16))
        anchor_coordinates.append(eval(rospy.get_param('~anchor' + str(i) + '_coordinates', '0, 0, 0')))

    frequency = int(rospy.get_param('~frequency', 50))
    algorithm = int(rospy.get_param('~algorithm', 4))
    dimension = int(rospy.get_param('~dimension', 3))
    height    = int(rospy.get_param('~height', 1000))
    filter_type    = int(rospy.get_param('~filter_type', 1))
    filter_strength    = int(rospy.get_param('~filter_strength', 2))
    ranging_protocol    = int(rospy.get_param('~ranging_protocol', 2))

    # IDs of the tags to position, add None to position the local tag as well.
    #tag_ids = [None, 0x6951]

    # necessary data for calibration
    anchors = []

    for i in range(number_of_anchors):
        print anchor_coordinates[i][0]
        anchors.append(DeviceCoordinates(anchor_id[i], 1, Coordinates(int(anchor_coordinates[i][0]), int(anchor_coordinates[i][1]), int(anchor_coordinates[i][2]))))

    pozyx = PozyxSerial(serial_port)

    r = MultitagPositioning(pozyx, None, None, anchors, algorithm, dimension, height, filter_type, filter_strength, ranging_protocol)
    r.setup()

    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        r.loop()
        rate.sleep()