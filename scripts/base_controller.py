#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_pick_and_place')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Controller():
    def __init__(self):
        rospy.init_node('base_controller')
        #self.pub = rospy.Publisher('cmd_vel', Twist)
        self.pub = rospy.Publisher('~cmd_vel', String)
        self.sub = rospy.Subscriber('pysub', String, self.callback)

    def callback(self, msg):
        rospy.loginfo("callback")
        cmd = Twist()
        cmd.linear.x = float(1)
        self.pub.publish(String('hehe'))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.run()

