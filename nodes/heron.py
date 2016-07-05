#!/usr/bin/python

import roslib; roslib.load_manifest('heron_node')
import rospy

from heron_node.twist_subscriber import TwistSubscriber

class Heron(object):
    def __init__(self):
        rospy.init_node('chameleon_twist')
	TwistSubscriber()

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
  Heron().spin()
