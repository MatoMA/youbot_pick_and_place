#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_pick_and_place')
import rospy
import sys
import select, termios, tty, signal
import moveit_commander, moveit_msgs.msg
from brics_actuator.msg import JointPositions, JointValue
import geometry_msgs.msg

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
        #moveit_commander.roscpp_initialize(sys.argv)
        #self.robotCommander = moveit_commander.RobotCommander()
        #self.scene = moveit_commander.PlanningSceneInterface()
        #self.arm = moveit_commander.MoveGroupCommander("arm")

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
        if self.state == 'start' or self.state == 'grap':
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
            self.state = 'grap'

    def poseGrap2(self):
        if self.state == 'home':
            joint_1.value = 2.9544
            joint_2.value = 2.617
            joint_3.value = -1.0732
            joint_4.value = 0.32213
            joint_5.value = 3
            pos = JointPositions()
            pos.positions = (joint_1, joint_2, joint_3, joint_4, joint_5)
            self.armPub.publish(pos)
            self.state = 'grap'

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

    def pose2(self):
        print "pose2"

class TimeoutException(Exception):
    pass

def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) # this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
        key = sys.stdin.read(1)
       # print "Read key"
    except TimeoutException:
        # print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)
    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    controller = ArmController()
    while(1):
        settings = termios.tcgetattr(sys.stdin)
        key = getKey()
        if key == '1':
            controller.poseStart()
        elif key == '2':
            controller.poseHome()
        elif key == '3':
            controller.poseGrap1()
        elif key == '4':
            controller.poseGrap2()
        elif key == '9':
            controller.openGripper()
        elif key == '0':
            controller.closeGripper()
        elif key == 'q':
            break
