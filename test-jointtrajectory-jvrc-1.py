#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointtrajectory_publisher():
    pub = rospy.Publisher('/JVRC_1/set_joint_trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('jvrc1test', anonymous=True)
    r = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        now = rospy.get_time()
        msg = JointTrajectory()
        msg.joint_names = ["NECK_Y"]
        msg.points = []
        v = [-0.5, 0, 0.5][i % 3]
        p = JointTrajectoryPoint()
        p.time_from_start = rospy.rostime.Duration(0)
        p.positions = [v]
        msg.points.append(p)
        rospy.loginfo("hello world %s, %s" % (v, now))
        pub.publish(msg)
        i += 1
        r.sleep()

if __name__ == '__main__':
    try:
        jointtrajectory_publisher()
    except rospy.ROSInterruptException:
        pass
