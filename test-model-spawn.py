#!/usr/bin/env python

import time
import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose

ns = '/World/'

def modelspawn(name, fname, pose):
    f = open(fname,'r')
    model_xml = f.read()
    rospy.wait_for_service(ns + 'spawn_vrml_model')
    try:
        spawnsrv = rospy.ServiceProxy(ns + 'spawn_vrml_model', SpawnModel)
        spawnsrv(name, model_xml, name, pose, None)
    except rospy.ServiceException, e:
        print "Service call failed %s" % e

def modeldelete(name):
    rospy.wait_for_service(ns + 'delete_model')
    try:
        deletesrv = rospy.ServiceProxy(ns + 'delete_model', DeleteModel)
        deletesrv(name)
    except rospy.ServiceException, e:
        print "Service call failed %s" % e

if __name__ == '__main__':
    print "spawn model"
    modelspawn("test", "/usr/share/OpenHRP-3.1/sample/model/box.wrl", Pose())
    time.sleep(10)
    print "delete model"
    modeldelete("test")
