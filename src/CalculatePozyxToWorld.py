#!/usr/bin/env python

import rospy
import rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from numpy import *
from math import sqrt
import sys, termios, tty, os, time

class PointsSubscriber():
    def __init__(self):
        self.pozyx_odometry_sub = rospy.Subscriber("pozyx_odometry", Odometry, self.pozyxOdometryCallback)
        self.pozyx_transform_sub = rospy.Subscriber("pozyx_transform", TransformStamped, self.pozyxtransformCallback)

        self.world_odometry_sub = rospy.Subscriber("world_odometry", Odometry, self.worldOdometryCallback)
        self.world_transform_sub = rospy.Subscriber("world_transform", TransformStamped, self.worldtransformCallback)

        self.pozyx_points = mat([0.0, 0.0, 0.0])
        self.world_points = mat([0.0, 0.0, 0.0])

    def pozyxOdometryCallback(self, msg):
        self.pozyx_points.itemset(0, msg.pose.pose.position.x)
        self.pozyx_points.itemset(1, msg.pose.pose.position.y)
        self.pozyx_points.itemset(2, msg.pose.pose.position.z)

    def pozyxtransformCallback(self, msg):
        print "pozyx transform"

    def worldOdometryCallback(self, msg):
        self.world_points.itemset(0, msg.pose.pose.position.x)
        self.world_points.itemset(1, msg.pose.pose.position.y)
        self.world_points.itemset(2, msg.pose.pose.position.z)

    def worldtransformCallback(self, msg):
        print "world transform"

    def getPozyxPoints(self):
        return self.pozyx_points

    def getWorldPoints(self):
        return self.world_points

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB
    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
        print "Reflection detected"
        Vt[2,:] *= -1
        R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    #print t

    return R, t


if __name__ == "__main__":
    rospy.init_node('calculate_pozyx_to_word')
    
    data = PointsSubscriber()

    poyzx_points_visualize_pub = rospy.Publisher('pozyx_visualization_points', Marker, queue_size=1)
    world_points_visualize_pub = rospy.Publisher('world_visualization_points', Marker, queue_size=1)
    transformed_points_visualize_pub = rospy.Publisher('transformed_visualization_points', Marker, queue_size=1)

    frequency = int(rospy.get_param('~frequency', 20))
    max_number_of_points = int(rospy.get_param('~number_of_points', 500))
    output_file = rospy.get_param('~output_file', "transformation_file.txt")

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('pozyx_ros')

    point_number = 0

    rate = rospy.Rate(frequency)

    A = mat(random.rand(max_number_of_points,3));
    B = mat(random.rand(max_number_of_points,3));

    print("---------------------------------------------\n")
    print("Pres ANY KEY to start calibration method!\n")
    print("---------------------------------------------\n")
    char = getch()
    print("Calibration started!\n")

    pozyx_marker = Marker()
    world_marker = Marker()

    while not rospy.is_shutdown():

        pozyx_points = data.getPozyxPoints()
        world_points = data.getWorldPoints()

        A.itemset((point_number, 0), pozyx_points.item(0))
        A.itemset((point_number, 1), pozyx_points.item(1))
        A.itemset((point_number, 2), pozyx_points.item(2))

        B.itemset((point_number, 0), world_points.item(0))
        B.itemset((point_number, 1), world_points.item(1))
        B.itemset((point_number, 2), world_points.item(2))


        pozyx_marker.header.stamp = rospy.Time()
        pozyx_marker.header.frame_id = "map"
        pozyx_marker.header.seq = point_number
        pozyx_marker.ns = "pozyx"
        pozyx_marker.type = 7
        pozyx_marker.action = 0
        pozyx_marker.scale.x = 0.2
        pozyx_marker.scale.y = 0.2
        pozyx_marker.scale.z = 0.2
        pozyx_marker.lifetime = rospy.Duration(0)
        pozyx_marker.text = "pozyx"
        color = ColorRGBA()
        color.r = 0.91
        color.g = 0.43
        color.b = 0.15
        color.a = 1.0

        p = Point()
        p.x = pozyx_points.item(0)
        p.y = pozyx_points.item(1)
        p.z = pozyx_points.item(2)
        pozyx_marker.points.append(p)
        pozyx_marker.colors.append(color)


        world_marker.header.stamp = rospy.Time()
        world_marker.header.frame_id = "map"
        world_marker.header.seq = point_number
        world_marker.ns = "world"
        world_marker.type = 7
        world_marker.action = 0
        world_marker.scale.x = 0.2
        world_marker.scale.y = 0.2
        world_marker.scale.z = 0.2
        world_marker.lifetime = rospy.Duration(0)
        pozyx_marker.text = "world"
        color = ColorRGBA()
        color.r = 0.56
        color.g = 0.23
        color.b = 0.6
        color.a = 1.0

        p = Point()
        p.x = world_points.item(0)
        p.y = world_points.item(1)
        p.z = world_points.item(2)
        world_marker.points.append(p)
        world_marker.colors.append(color)

        world_points_visualize_pub.publish(world_marker)
        poyzx_points_visualize_pub.publish(pozyx_marker)

        point_number = point_number + 1
        if (point_number >= max_number_of_points):
         break

        rate.sleep()

    # recover the transformation
    ret_R, ret_t = rigid_transform_3D(A, B)

    A2 = (ret_R*A.T) + tile(ret_t, (1, max_number_of_points))
    A2 = A2.T

    # Find the error
    err = A2 - B

    err = multiply(err, err)
    err = sum(err)
    rmse = sqrt(err/max_number_of_points);

    
    transformed_marker = Marker()
    transformed_marker.header.stamp = rospy.Time()
    transformed_marker.header.frame_id = "map"
    transformed_marker.header.seq = point_number - 1
    transformed_marker.ns = "transformed_pozyx"
    transformed_marker.type = 7
    transformed_marker.action = 0
    transformed_marker.scale.x = 0.2
    transformed_marker.scale.y = 0.2
    transformed_marker.scale.z = 0.2
    transformed_marker.lifetime = rospy.Duration(0)
    pozyx_marker.text = "transformed"
    color = ColorRGBA()
    color.r = 0.3
    color.g = 0.12
    color.b = 0.78
    color.a = 1.0
    
    for i in range(max_number_of_points):
        p = Point()
        p.x = A2.item((i, 0))
        p.y = A2.item((i, 1))
        p.z = A2.item((i, 2))
        transformed_marker.points.append(p)
        transformed_marker.colors.append(color)

    transformed_points_visualize_pub.publish(transformed_marker)

    print("---------------------------------------------\n")
    print "Rotation calculated"
    print ret_R
    print ""

    print "Translation calculated"
    print ret_t
    print ""

    print "RMSE:", rmse
    print "If RMSE is near zero, the function is correct!"

    file = open(package_path + "/config/" + output_file,"w") 
    file.write("Rotation matrix: \n");
    file.write("[%f, %f, %f,\n" % (ret_R.item(0), ret_R.item(1), ret_R.item(2)))
    file.write("[%f, %f, %f,\n" % (ret_R.item(3), ret_R.item(4), ret_R.item(5)))
    file.write("[%f, %f, %f]\n" % (ret_R.item(6), ret_R.item(7), ret_R.item(8)))

    file.write("Translation matrix: \n");
    file.write("[%f,\n" % (ret_t.item(0)))
    file.write("[%f,\n" % (ret_t.item(1)))
    file.write("[%f]\n" % (ret_t.item(2)))  
    file.close() 