#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_pick_and_place')
import rospy
from geometry_msgs.msg import Twist
import tf
import math
#from std_msgs.msg import String

LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.1
LEFT = (0, 1, 0)
RIGHT = (0, -1, 0)
FORWARDS = (1, 0, 0)
BACKWARDS = (-1, 0, 0)

class Controller():
    def __init__(self):
        rospy.init_node('base_controller')
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self.tf_listener = tf.TransformListener()
        #self.pub = rospy.Publisher('~cmd_vel', String)
        #self.sub = rospy.Subscriber('pysub', String, self.callback)

    def callback(self, msg):
        rospy.loginfo("callback")
        #cmd = Twist()
        #cmd.linear.x = float(1)
        #self.pub.publish(String('hehe'))

    def go(trans):
        #Norm of trans
        L = math.sqrt(sum([x**2 for x in trans]))
        norm = [x/L for x in trans]
        vel = [x*LINEAR_SPEED for x in norm]
        time = L/LINEAR_SPEED

        cmd = Twist()
        cmd.linear.x = vel[0]
        cmd.linear.y = vel[1]
        cmd.linear.z = vel[2]
        self.pub.publish(cmd)
        rospy.sleep(time)

        #Stop
        cmd = Twist()
        self.pub.publish(cmd)

    def run(self):
        #rospy.spin()
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.tf_listener.waitForTransform('/tf1','/tf2',now,rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/tf1','/tf2', now)
            except (tf.Exception, tf.LookupException):
                continue

            print trans
            print rot

if __name__ == '__main__':
    controller = Controller()
    controller.run()

