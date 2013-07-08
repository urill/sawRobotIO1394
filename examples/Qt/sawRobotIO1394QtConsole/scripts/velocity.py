#!/usr/bin/env python

import roslib; roslib.load_manifest('sawRobotIO1394QtConsole')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sawROS.msg import vctDoubleVec


class Velcity():
    def __init__(self):
        # ros init
        rospy.init_node('talker')
        self.pub_ = rospy.Publisher('/MTMR/joint_velocity/joint1', Float64)
        self.sub_ = rospy.Subscriber('/MTMR/joint_velocity',
                                     vctDoubleVec,
                                     self.callback)
        # class variable
        self.vel_ = -1.0
        self.rate_ = rospy.Rate(50.0)

    def run(self):
        while not rospy.is_shutdown():
#            self.pub_.publish(Float64(self.vel_))
            self.rate_.sleep()
            pass

    def callback(self, data):
#        rospy.loginfo("%s: %f", rospy.get_time(), data.data[0])
        self.vel_ = data.data[4]
        self.pub_.publish(Float64(self.vel_))
        pass
                
if __name__ == '__main__':
    try:
        vel = Velcity()
        vel.run()
    except rospy.ROSInterruptException:
        pass


