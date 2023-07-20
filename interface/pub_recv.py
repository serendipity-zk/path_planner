#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from shapely import simplify
from shapely.geometry import LineString


class PathMonitor:
    def __init__(self):
        # Initialize the node
        rospy.init_node('path_monitor', anonymous=True)
        
        # Publishers
        self.init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscriber
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)

        # State variable and path storage
        self.path_ready = False
        self.path = []

    def path_callback(self, data):
        if data.poses.__len__() > 0:
            self.path = [[pose.pose.position.x, pose.pose.position.y] for pose in data.poses]
            self.path_ready = True
        else :
            self.path_ready = False

    def pub_init_pos(self, x, y, heading):
        quaternion = quaternion_from_euler(0, 0, heading)

        # Create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        # Set pose values
        msg.pose.pose.position.x = x + 0.001
        msg.pose.pose.position.y = y + 0.001
        msg.pose.pose.orientation = Quaternion(*quaternion)

        # Covariance (placeholder)
        msg.pose.covariance = [0.0] * 36
        self.init_pose_pub.publish(msg)
        # while not self.path_ready:
        #     rospy.sleep(0.1)
        # self.path_ready = False

    def pub_goal(self, x, y, heading):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x + 0.001
        goal.pose.position.y = y + 0.001
        quaternion = quaternion_from_euler(0, 0, heading)
        goal.pose.orientation = Quaternion(*quaternion)
        self.goal_pub.publish(goal)

    def get_path(self):
        if self.path_ready:
            self.path_ready = False
        return self.path

if __name__ == '__main__':
    monitor = PathMonitor()
    monitor.pub_init_pos(0, 0, 0)
    monitor.pub_goal(10, 10, 0)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
