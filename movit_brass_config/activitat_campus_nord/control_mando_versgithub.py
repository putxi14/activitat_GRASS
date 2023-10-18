#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from moveit_commander import MoveGroupCommander, RobotCommander



class JoystickFullControl:
    def __init__(self):
        rospy.init_node('joystick_full_control', anonymous=True)
        self.arm = MoveGroupCommander('arm_group')
        #self.arm.set_planning_pipeline_id("pilz_industrial_motion_planner")
        #self.arm.set_planner_id("LIN")
        self.arm.set_planning_pipeline_id("ompl")
        self.arm.set_planner_id("RRT")
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Asegúrate de ajustar esto según los ejes de tu joystick
        self.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #self.arm = RobotCommander()
        self.scale = 0.1  # Ajusta este valor según tu preferencia


    def set_pose_target_arm(self, current_pose):
        self.arm.set_pose_target(current_pose)
        self.arm.go(wait=True)
        self.arm.stop()
    
    def set_joint_value_target(self, current_joint_positions):
        self.arm.set_joint_value_target(current_joint_positions)
        self.arm.go(wait=True)
        self.arm.stop()

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
            

    def joy_callback(self, joy_msg):
        self.axes = joy_msg.axes
        self.buttons = joy_msg.buttons
          
    
    def full_control(self):
        rate = rospy.Rate(5)  # Frecuencia de actualización (Hz)

        while not rospy.is_shutdown():
        
            

            if self.buttons[0] == 1:
                self.scale = 0.01

            elif self.buttons[2] == 1:
                self.scale = 0.1

            elif self.buttons[1] == 1:
                self.bon_dia()

            elif self.buttons[3] == 1:
                self.homming()

            elif self.buttons[4] == 1:
                current_joint_positions = self.arm.get_current_joint_values()
                joint_angle_increment = 0.1          
                current_joint_positions[3] += joint_angle_increment

                self.set_joint_value_target(current_joint_positions)
            
            elif self.buttons[5] == 1:
                current_joint_positions = self.arm.get_current_joint_values()
                joint_angle_increment = 0.1          
                current_joint_positions[3] -= joint_angle_increment

                self.set_joint_value_target(current_joint_positions)

            #x = self.axes[1] * self.scale
            #z = self.axes[3] * self.scale
            x = self.scale
            z = self.scale          
        
            if self.axes[1] > 0.8: #Mou el brass en l'eix x
                current_pose = self.arm.get_current_pose().pose
                current_pose.position.x += x
                self.set_pose_target_arm(current_pose)

            elif self.axes[1] < -0.8: #Mou el brass en l'eix x
                current_pose = self.arm.get_current_pose().pose
                current_pose.position.x -= x
                self.set_pose_target_arm(current_pose)
            
            elif self.axes[3] > 0.8: #Mou el brass en l'eix z
                current_pose = self.arm.get_current_pose().pose
                current_pose.position.z += z
                self.set_pose_target_arm(current_pose)

            elif self.axes[3] < -0.8: #Mou el brass en l'eix z
                current_pose = self.arm.get_current_pose().pose
                current_pose.position.z -= z
                self.set_pose_target_arm(current_pose)
            
            elif abs(self.axes[2]) > 0.8: #Només rota l'end effector del brass
                current_joint_positions = self.arm.get_current_joint_values()
                joint_angle_increment = 0.08  
            
                if self.axes[2] < -0.8:
                    current_joint_positions[4] += joint_angle_increment
                elif self.axes[5] < -0.8:
                    current_joint_positions[4] -= joint_angle_increment

                self.set_joint_value_target(current_joint_positions)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        joystick_full_control = JoystickFullControl()
        joystick_full_control.full_control()
    except rospy.ROSInterruptException:
        pass