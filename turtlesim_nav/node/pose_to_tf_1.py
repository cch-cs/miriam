#!/usr/bin/env python2
import rospy
import tf
from turtlesim.msg import Pose

if __name__ == "__main__":
    rospy.init_node("pose_to_tf")
    print("init")

    def callback_1(msg):
        br_1 = tf.TransformBroadcaster()
        br_1.sendTransform((msg.x, msg.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                         rospy.Time.now(),
                         "turtlebot1/base_link",
                         "map")
    def callback_2(msg):
        br_2 = tf.TransformBroadcaster()
        br_2.sendTransform((msg.x, msg.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                         rospy.Time.now(),
                         "turtlebot2/base_link",
                         "map")

    rospy.Subscriber("/turtlebot1/pose", Pose, callback_1)
    rospy.Subscriber("/turtlebot2/pose", Pose, callback_2)
    rospy.spin()
