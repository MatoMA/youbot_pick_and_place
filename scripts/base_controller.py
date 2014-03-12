#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_pick_and_place')
import rospy
from geometry_msgs.msg import Twist
import tf
import math
#from std_msgs.msg import String

LINEAR_SPEED = 0.08
ANGULAR_SPEED = 1.2
LEFT = (0, 1, 0)
RIGHT = (0, -1, 0)
FORWARDS = (1, 0, 0)
BACKWARDS = (-1, 0, 0)

class Controller():
    def __init__(self):
        rospy.init_node('base_controller')
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self.tf_listener = tf.TransformListener()
        self.arrived = False
        self.sens = 1
        #self.pub = rospy.Publisher('~cmd_vel', String)
        #self.sub = rospy.Subscriber('pysub', String, self.callback)

    def callback(self, msg):
        rospy.loginfo("callback")
        #cmd = Twist()
        #cmd.linear.x = float(1)
        #self.pub.publish(String('hehe'))

    def go(self, trans):
        #Norm of trans
        L = math.sqrt(sum([x**2 for x in trans]))

        if L > 0.75:
            norm = [x/L for x in trans]
            vel = [x*LINEAR_SPEED for x in norm]
            time = L/LINEAR_SPEED

            cmd = Twist()
            cmd.linear.x = 1.5*vel[2]
            cmd.linear.y = -6.0*vel[0]
            self.pub.publish(cmd)
            rospy.sleep(0.1)
            
        else:
            self.arrived = True
            print "Robot en position"

        #Stop
            self.stop()
        
    def rot(self, angle):
        cmd = Twist()
        vel = angle*ANGULAR_SPEED
        cmd.angular.z = vel
        self.pub.publish(cmd)
        rospy.sleep(0.1)
    
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        self.pub.publish(cmd)

    def run(self):
        #rospy.spin()
        trans_prec = 0
        while (not self.arrived and not rospy.is_shutdown()):
            try:
                #now = rospy.Time.now()
                self.tf_listener.waitForTransform('/base_cam_link','/ar_marker_1',rospy.Time(0),rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/base_cam_link','/ar_marker_1', rospy.Time(0))
                print trans
                if trans_prec == trans[2]:
                    if trans_prec <= 0.75:
                        self.arrived = True
                        print "Robot en position"
                        self.stop()
                else:
                    self.go(trans)
                    trans_prec = trans[2]
            except (tf.Exception, tf.LookupException):
                print "Error"
                self.stop()
                self.init(self.sens)

    def init(self, sens):
        found = False
        while (not found and not rospy.is_shutdown()):
            try:
                self.tf_listener.waitForTransform('/base_cam_link','/ar_marker_1',rospy.Time(0),rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/base_cam_link','/ar_marker_1', rospy.Time(0))
                found = True
            except (tf.Exception):
                self.rot(sens*0.2)
                print "Recherche"
                continue
        self.stop()
        self.sens = -self.sens
        self.run()

if __name__ == '__main__':
    controller = Controller()
    controller.init(controller.sens)

