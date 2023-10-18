#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from moveit_commander import MoveGroupCommander, RobotCommander
import numpy as np

#axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

class JoystickFullControl:    
    def __init__(self):
        rospy.init_node('joystick_full_control', anonymous=True)
        self.arm = MoveGroupCommander('arm_group')
        self.arm.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.arm.set_planner_id("LIN")
        #self.arm.set_planning_pipeline_id("ompl")
        #self.arm.set_planner_id("RRT")
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
    def joy_callback(self, joy_msg):
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons

    def homming(self):
        joint_goal = self.arm.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0

        self.arm.go(joint_goal, wait=True)
        self.arm.stop()

    def bon_dia(self):
        i = 0
        j = 0
        self.homming()
        joint_goal = self.arm.get_current_joint_values()

        while i < 2:
            joint_goal[0] = -0.785
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = 0
            joint_goal[4] = 0

            self.arm.go(joint_goal, wait=True)
            self.arm.stop() 

            while j < 2:
                joint_goal[0] = -0.785
                joint_goal[1] = 0.279
                joint_goal[2] = 0.524
                joint_goal[3] = -0.473
                joint_goal[4] = 0

                self.arm.go(joint_goal, wait=True)
                self.arm.stop() 

                joint_goal[0] = -0.785
                joint_goal[1] = 0.279
                joint_goal[2] = 0.524
                joint_goal[3] = 0
                joint_goal[4] = 0
                self.arm.go(joint_goal, wait=True)
                self.arm.stop() 

                j += 1             

            joint_goal[0] = 0.0
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = 0
            joint_goal[4] = 0

            self.arm.go(joint_goal, wait=True)
            self.arm.stop() 

            joint_goal[0] = 0.0
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = 0
            joint_goal[4] = 0
            self.arm.go(joint_goal, wait=True)
            self.arm.stop()  

            j = 0

            while j < 2:
                joint_goal[0] = 0.785
                joint_goal[1] = 0.279
                joint_goal[2] = 0.524
                joint_goal[3] = -0.473
                joint_goal[4] = 0

                self.arm.go(joint_goal, wait=True)
                self.arm.stop() 

                joint_goal[0] = 0.785
                joint_goal[1] = 0.279
                joint_goal[2] = 0.524
                joint_goal[3] = 0
                joint_goal[4] = 0
                self.arm.go(joint_goal, wait=True)
                self.arm.stop() 

                j += 1    

            joint_goal[0] = 0.0
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = 0
            joint_goal[4] = 0
            self.arm.go(joint_goal, wait=True)
            self.arm.stop()

            i += 1
            j = 0

        self.homming()

    def full_control(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            if self.buttons[2] == 1:
                self.homming()
            
            
            elif self.buttons[0] == 1:
                self.bon_dia()

            rate.sleep()

if __name__ == '__main__':
    try:
        joystick_full_control = JoystickFullControl()
        joystick_full_control.full_control()
    except rospy.ROSInterruptException:
        pass