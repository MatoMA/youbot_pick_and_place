#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_pick_and_place')
import rospy
from geometry_msgs.msg import Twist
from brics_actuator import JointPositions, JointValue
import tf
import math

gripper_l = JointValue()
gripper_l.joint_uri = 'gripper_finger_joint_l'
gripper_l.unit = 'm'
gripper_r = JointValue()
gripper_r.joint_uri = 'gripper_finger_joint_r'
gripper_r.unit = 'm'
joint_1 = JointValue()
joint_1.joint_uri = 'arm_joint_1'
joint_1.unit = 'rad'
joint_2 = JointValue()
joint_2.joint_uri = 'arm_joint_2'
joint_2.unit = 'rad'
joint_3 = JointValue()
joint_3.joint_uri = 'arm_joint_3'
joint_3.unit = 'rad'
joint_4 = JointValue()
joint_4.joint_uri = 'arm_joint_4'
joint_4.unit = 'rad'
joint_5 = JointValue()
joint_5.joint_uri = 'arm_joint_5'
joint_5.unit = 'rad'

class ArmController():
    def __init__(self):
        rospy.init_node("arm_controller")
        self.armPub = rospy.Publisher('/arm_1/arm_controller/position_command', JointPositions)
        self.gripperPub = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)
        self.state = 'start'

    def poseStart(self):
        if self.state == 'home':
            joint_1.value = 0.01006921
            joint_2.value = 0.01006921
            joint_3.value = -0.0157081
            joint_4.value = 0.0221391
            joint_5.value = 0.11062
            pos = JointPositions()
            pos.positions = (joint_1, joint_2, joint_3, joint_4, joint_5)
            self.armPub.publish(pos)
            self.state = 'start'

    def poseHome(self):
        if self.state == 'start' or self.state == 'grap1' or self.state == 'grap2':
            joint_1.value = 2.9
            joint_2.value = 1.1
            joint_3.value = -2.5
            joint_4.value = 0.023
            joint_5.value = 3
            pos = JointPositions()
            pos.positions = (joint_1, joint_2, joint_3, joint_4, joint_5)
            self.armPub.publish(pos)
            self.state = 'home'

    def poseGrap1(self):
        if self.state == 'home':
            joint_1.value = 2.9544
            joint_2.value = 2.617
            joint_3.value = -1.0732
            joint_4.value = 0.32213
            joint_5.value = 3
            pos = JointPositions()
            pos.positions = (joint_1, joint_2, joint_3, joint_4, joint_5)
            self.armPub.publish(pos)
            self.state = 'grap1'

    def poseGrap2(self):
        if self.state == 'home':
            joint_1.value = 2.9
            joint_2.value = 1.5
            joint_3.value = -2.0
            joint_4.value = 2.5
            joint_5.value = 3
            pos = JointPositions()
            pos.positions = (joint_1, joint_2, joint_3, joint_4, joint_5)
            self.armPub.publish(pos)
            self.state = 'grap2'

    def openGripper(self):
        gripper_l.value = 0.0115
        gripper_r.value = 0.0115
        pos = JointPositions()
        pos.positions = (gripper_l, gripper_r)
        self.gripperPub.publish(pos)

    def closeGripper(self):
        gripper_l.value = 0.0025
        gripper_r.value = 0.0025
        pos = JointPositions()
        pos.positions = (gripper_l, gripper_r)
        self.gripperPub.publish(pos)

LINEAR_SPEED = 0.05
#ANGULAR_SPEED = 1.2
LEFT = (0, 1, 0)
RIGHT = (0, -1, 0)
FORWARDS = (1, 0, 0)
BACKWARDS = (-1, 0, 0)

class BaseController():
    def __init__(self):
        rospy.init_node('base_laser_controller')
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self.tf_listener = tf.TransformListener()

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

    def goToObject(self):
        trans_prec = 1
        while abs(trans_prec) > 0.3:
            try:
                #now = rospy.Time.now()
                self.tf_listener.waitForTransform('/base_laser_front_link','/object',rospy.Time(0),rospy.Duration(1))
                (trans, rot) = self.tf_listener.lookupTransform('/base_laser_front_link','/object', rospy.Time(0))
                print trans
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
                self.go_x(trans[0])
                trans_prec = trans[0]
            except (tf.Exception, tf.LookupException):
                print "Error"
                self.stop()
        self.stop()
        print "Robot en position"

    def go(self, trans):
        #Norm of trans
        L = math.sqrt(sum([x**2 for x in trans]))

        if L > 0.75:
            norm = [x/L for x in trans]
            vel = [x*LINEAR_SPEED for x in norm]

            cmd = Twist()
            cmd.linear.x = 1.5*vel[2]
            cmd.linear.y = -6.0*vel[0]
            #print cmd
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

if __name__ == '__main__':
    controller = Controller()
    controller.run()

