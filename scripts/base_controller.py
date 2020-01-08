#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf_conversions


class RobotBase(object):
    def __init__(self):
        rospy.init_node("robot_base")
        rospy.loginfo("Initial base controller")

        cmdTopic = rospy.get_param("~cmd_topic", "cmd_vel")
        cmdFreq = float(rospy.get_param("~cmd_freq", "5"))

        odomTopic = rospy.get_param("~odom_topic", "odom")
        odomFreq = float(rospy.get_param("~odom_freq", "10"))

        self.odomFrame = rospy.get_param("~odom_frame_id", "odom")
        self.odomChildframe = rospy.get_param(
            "~base_frame_id", "base_footprint")
        self.pubOdomFrame = bool(
            rospy.get_param("~publish_odom_frame", "True"))

        self.odom_pub = rospy.Publisher(odomTopic, Odometry, queue_size=1)
        rospy.Subscriber(cmdTopic, Twist, self.cmdvelCB, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/cmdFreq), self.commandCB)
        rospy.Timer(rospy.Duration(1.0/odomFreq), self.odomCB)

        self.goal_trans_x = 0.0
        self.goal_trans_y = 0.0
        self.goal_rotat_z = 0.0

        self.pose = Pose2D()
        self.twist = Pose2D()
        self.curr_time = rospy.Time.now()
        self.prev_time = self.curr_time

    def spin(self):
        rospy.spin()

    def cmdvelCB(self, msg):
        self.goal_trans_x = msg.linear.x
        self.goal_trans_y = msg.linear.y
        self.goal_rotat_z = msg.angular.z

    def commandCB(self, event):
        pass

    def odomCB(self, event):
        pass

    def pubOdom(self):
        self.curr_time = rospy.Time.now()
        dt = (self.curr_time - self.prev_time).to_sec()
        self.prev_time = self.curr_time

        self.pose.x += self.twist.x * dt
        self.pose.y += self.twist.y * dt
        self.pose.theta += self.twist.theta * dt

        odom = Odometry()

        odom.header.stamp = self.curr_time
        odom.header.frame_id = self.odomFrame
        odom.child_frame_id = self.odomChildframe
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        q = tf_conversions.transformations.quaternion_from_euler(
            0, 0, self.pose.theta)

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.twist.x
        odom.twist.twist.linear.y = self.twist.y
        odom.twist.twist.angular.z = self.twist.theta

        self.odom_pub.publish(odom)

        odom_tf = TransformStamped()
        odom_tf.header.stamp = self.curr_time
        odom_tf.header = odom.header
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.transform.translation = odom.pose.pose.position
        odom_tf.transform.rotation = odom.pose.pose.orientation

        if self.pubOdomFrame:
            tf2_ros.TransformBroadcaster().sendTransform(odom_tf)


class DiffBase(RobotBase):
    def __init__(self, wheel_radius, wheel_sep):
        super(DiffBase, self).__init__()
        self.wheel_radius = wheel_radius
        self.wheel_sep = wheel_sep


class Omni3Base(RobotBase):
    def __init__(self):
        pass


class Omni4Base(RobotBase):
    def __init__(self):
        pass


class MecanumBase(RobotBase):
    def __init__(self, wheel_radius, wheel_sep_width, wheel_sep_length,  driveFunc, odomFunc):
        super(MecanumBase, self).__init__()

        self.wheel_sep_width = wheel_sep_width / 2.0
        self.wheel_sep_length = wheel_sep_length / 2.0
        self.wheel_radius = wheel_radius

        self.goal_drive_wheel = [0.0, 0.0, 0.0, 0.0]
        self.drive_wheel = [0.0, 0.0, 0.0, 0.0]
        self.drive_func = driveFunc
        self.odom_func = odomFunc

    def commandCB(self, event):
        ux = self.goal_trans_x
        uy = self.goal_trans_y
        uz = (self.wheel_sep_width + self.wheel_sep_length) * self.goal_rotat_z

        self.goal_drive_wheel[0] = (ux - uy - uz) / self.wheel_radius
        self.goal_drive_wheel[1] = -1 * (ux + uy + uz) / self.wheel_radius
        self.goal_drive_wheel[2] = (ux + uy - uz) / self.wheel_radius
        self.goal_drive_wheel[3] = -1 * (ux - uy + uz) / self.wheel_radius

        self.drive_func(self.goal_drive_wheel)
        # self.drive_wheel = self.goal_drive_wheel

    def odomCB(self, event):
        self.drive_wheel = self.odom_func()
        self.twist.x = (self.drive_wheel[0] - self.drive_wheel[1] +
                        self.drive_wheel[2] - self.drive_wheel[3])*self.wheel_radius/4.0
        self.twist.y = (-self.drive_wheel[0] - self.drive_wheel[1] +
                        self.drive_wheel[2]+self.drive_wheel[3])*self.wheel_radius/4.0
        self.twist.theta = (-self.drive_wheel[0]-self.drive_wheel[1]-self.drive_wheel[2] -
                            self.drive_wheel[3])*self.wheel_radius/(4.0 * (self.wheel_sep_width+self.wheel_sep_length))

        self.pubOdom()


class AckermanBase(RobotBase):
    def __init__(self):
        pass
