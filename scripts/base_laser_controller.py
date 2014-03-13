#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_pick_and_place')
import rospy
from geometry_msgs.msg import Twist
import tf
import math
#from std_msgs.msg import String

LINEAR_SPEED = 0.05
#ANGULAR_SPEED = 1.2
LEFT = (0, 1, 0)
RIGHT = (0, -1, 0)
FORWARDS = (1, 0, 0)
BACKWARDS = (-1, 0, 0)

class Controller():
    def __init__(self):
        rospy.init_node('base_laser_controller')
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self.tf_listener = tf.TransformListener()
        #self.pub = rospy.Publisher('~cmd_vel', String)
        #self.sub = rospy.Subscriber('pysub', String, self.callback)

    def callback(self, msg):
        rospy.loginfo("callback")
        #cmd = Twist()
        #cmd.linear.x = float(1)
        #self.pub.publish(String('hehe'))

    def go_x(self, trans):

        vel = trans*LINEAR_SPEED
        
        cmd = Twist()
        cmd.linear.x = vel
        cmd.linear.y = 0
        self.pub.publish(cmd)
        rospy.sleep(0.5)
    
    def go_y(self, trans):
        vel = trans*LINEAR_SPEED
        
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = vel*5
        self.pub.publish(cmd)
        rospy.sleep(0.5)
    
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        self.pub.publish(cmd)

    def run(self):
        #rospy.spin()
        trans_prec = 1
        while abs(trans_prec) > 0.3:
            try:
                #now = rospy.Time.now()
                self.tf_listener.waitForTransform('/base_laser_front_link','/object',rospy.Time(0),rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/base_laser_front_link','/object', rospy.Time(0))
                print trans
                #if trans_prec == trans[2]:
                 #   if trans_prec <= 0.75:
                  #      self.arrived = True
                   #     print "Robot en position"
                    #    self.stop()
                #else:
                self.go_x(trans[0])
                trans_prec = trans[0]
            except (tf.Exception, tf.LookupException):
                print "Error"
                self.stop()
        while abs(trans_prec) > 0.007:
            try:
                #now = rospy.Time.now()
                self.tf_listener.waitForTransform('/base_laser_front_link','/object',rospy.Time(0),rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/base_laser_front_link','/object', rospy.Time(0))
                print trans
                #if trans_prec == trans[2]:
                 #   if trans_prec <= 0.75:
                  #      self.arrived = True
                   #     print "Robot en position"
                    #    self.stop()
                #else:
                self.go_y(trans[1])
                trans_prec = trans[1]
            except (tf.Exception, tf.LookupException):
                print "Error"
                self.stop()

        trans_prec = 1
        while trans_prec > 0.225:
            try:
                #now = rospy.Time.now()
                self.tf_listener.waitForTransform('/base_laser_front_link','/object',rospy.Time(0),rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/base_laser_front_link','/object', rospy.Time(0))
                print trans
                #if trans_prec == trans[2]:
                 #   if trans_prec <= 0.75:
                  #      self.arrived = True
                   #     print "Robot en position"
                    #    self.stop()
                #else:
                self.go_x(trans[0])
                trans_prec = trans[0]
            except (tf.Exception, tf.LookupException):
                print "Error"
                self.stop()
        self.stop()        
        print "Robot en position"

if __name__ == '__main__':
    controller = Controller()
    controller.run()

