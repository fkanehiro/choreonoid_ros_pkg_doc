#!/usr/bin/env python
#
#

import sys
import os
import math
import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#
# Functions.
#

#
def publish_jvrc1_jointtrajectory(jname='NECK_Y', jdegree=0):
    rad = float('{0:.4f}'.format(math.radians(float(jdegree))))
    pub = rospy.Publisher('/JVRC_1/torque_control/set_joint_trajectory', JointTrajectory, latch=True, queue_size=10)
    rospy.init_node('jvrc1_choreonoid_ros_test', anonymous=True)
    msg = JointTrajectory()
    msg.joint_names = [ jname ]
    msg.points = []
    p = JointTrajectoryPoint()
    p.time_from_start = rospy.rostime.Duration(0)
    p.positions = [ rad ]

    msg.points.append(p)
    rospy.loginfo(msg)
    pub.publish(msg)
    rospy.Rate(1).sleep()

#
def check_jvrc1_joint_name(jname=''):
    names = { 
        'R_HIP_P': 0,      'R_HIP_R': 0,      'R_HIP_Y': 0,      'R_KNEE': 0,       'R_ANKLE_R': 0,
        'R_ANKLE_P': 0,    'L_HIP_P': 0,      'L_HIP_R': 0,      'L_HIP_Y': 0,      'L_KNEE': 0,
        'L_ANKLE_R': 0,    'L_ANKLE_P': 0,    'WAIST_Y': 0,      'WAIST_P': 0,      'WAIST_R': 0,
        'NECK_Y': 0,       'NECK_R': 0,       'NECK_P': 0,       'R_SHOULDER_P': 0, 'R_SHOULDER_R': 0,
        'R_SHOULDER_Y': 0, 'R_ELBOW_P': 0,    'R_ELBOW_Y': 0,    'R_WRIST_R': 0,    'R_WRIST_Y': 0,
        'R_UTHUMB': 0,     'R_LTHUMB': 0,     'R_UINDEX': 0,     'R_LINDEX': 0,     'R_ULITTLE': 0,
        'R_LLITTLE': 0,    'L_SHOULDER_P': 0, 'L_SHOULDER_R': 0, 'L_SHOULDER_Y': 0, 'L_ELBOW_P': 0,
        'L_ELBOW_Y': 0,    'L_WRIST_R': 0,    'L_WRIST_Y': 0,    'L_UTHUMB': 0,     'L_LTHUMB': 0,
        'L_UINDEX': 0,     'L_LINDEX': 0,     'L_ULITTLE': 0,    'L_LLITTLE': 0
        } 

    return jname in names

#
def usage():
    name = os.path.basename(sys.argv[0])

    print 'usage: ' + name + ' [joint name] [joint angle (degree)]'
    print ''
    print 'Set joint angle for JVRC-1 robot.'
    print ''
    print '[joint name] setting choose one of them:'
    print '  R_HIP_P      R_HIP_R      R_HIP_Y      R_KNEE       R_ANKLE_R,'
    print '  R_ANKLE_P    L_HIP_P      L_HIP_R      L_HIP_Y      L_KNEE'
    print '  L_ANKLE_R    L_ANKLE_P    WAIST_Y      WAIST_P      WAIST_R'
    print '  NECK_Y       NECK_R       NECK_P       R_SHOULDER_P R_SHOULDER_R'
    print '  R_SHOULDER_Y R_ELBOW_P    R_ELBOW_Y    R_WRIST_R    R_WRIST_Y'
    print '  R_UTHUMB     R_LTHUMB     R_UINDEX     R_LINDEX     R_ULITTLE'
    print '  R_LLITTLE    L_SHOULDER_P L_SHOULDER_R L_SHOULDER_Y L_ELBOW_P'
    print '  L_ELBOW_Y    L_WRIST_R    L_WRIST_Y    L_UTHUMB     L_LTHUMB'
    print '  L_UINDEX     L_LINDEX     L_ULITTLE    L_LLITTLE'
    print ''
    print '[joint angle] setting is range from 360.0 to -360.0.'
    print ''
    print 'e.g. if you want to change the joint angle of the NECK_Y to dgree of 40.'
    print '  ' + name + ' NECK_Y 40.0'
    print ''

    return

# 
# Main.  
#

if __name__ == '__main__':
    is_error = True

    try:
        if len(sys.argv) == 3:
            s = sys.argv[1]
            f = float(sys.argv[2])

            if (check_jvrc1_joint_name(s) and (f <= 360.0 and f >= -360.0)):
                publish_jvrc1_jointtrajectory(jname=s, jdegree=f)
                is_error = False
    except ValueError:
        pass
    except rospy.ROSInterruptException:
        pass

    if (is_error):
        usage()
        sys.exit(1)

    sys.exit(0)
