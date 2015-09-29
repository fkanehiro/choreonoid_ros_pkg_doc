#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointtrajectory_publisher():
    pub = rospy.Publisher('/HRP1/set_joint_trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('pa10test', anonymous=True)
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        now = rospy.get_time()
        msg = JointTrajectory()
        msg.joint_names = ["J1", "J2"]
        msg.points = []
        for i in [0, 1]:
            p = JointTrajectoryPoint()
            p.time_from_start = rospy.rostime.Duration(2)
            p.positions = [i, i]
            msg.points.append(p)
        rospy.loginfo("hello world %s" % now)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        jointtrajectory_publisher()
    except rospy.ROSInterruptException:
        pass
