#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from moveit_commander import MoveGroupCommander, RobotCommander
   
axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Asegúrate de ajustar esto según los ejes de tu joystick
buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#arm = RobotCommander()
scale = 0.1  # Ajusta este valor según tu preferencia

def set_pose_target_arm(current_pose):
    arm.set_pose_target(current_pose)
    arm.go(wait=True)
    arm.stop()

def set_joint_value_target(current_joint_positions):
    arm.set_joint_value_target(current_joint_positions)
    arm.go(wait=True)
    arm.stop()

def homming():
    joint_goal = arm.get_current_joint_values()

    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0

    arm.go(joint_goal, wait=True)
    arm.stop()

def bon_dia():
    i = 0
    j = 0
    homming()
    joint_goal = arm.get_current_joint_values()
    
    while i < 2:
        joint_goal[0] = -0.785
        joint_goal[1] = 0.279
        joint_goal[2] = 0.524
        joint_goal[3] = 0
        joint_goal[4] = 0

        arm.go(joint_goal, wait=True)
        arm.stop() 

        while j < 2:
            joint_goal[0] = -0.785
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = -0.473
            joint_goal[4] = 0

            arm.go(joint_goal, wait=True)
            arm.stop() 

            joint_goal[0] = -0.785
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = 0
            joint_goal[4] = 0
            arm.go(joint_goal, wait=True)
            arm.stop() 

            j += 1             

        joint_goal[0] = 0.0
        joint_goal[1] = 0.279
        joint_goal[2] = 0.524
        joint_goal[3] = 0
        joint_goal[4] = 0

        arm.go(joint_goal, wait=True)
        arm.stop() 

        joint_goal[0] = 0.0
        joint_goal[1] = 0.279
        joint_goal[2] = 0.524
        joint_goal[3] = 0
        joint_goal[4] = 0
        arm.go(joint_goal, wait=True)
        arm.stop()  

        j = 0

        while j < 2:
            joint_goal[0] = 0.785
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = -0.473
            joint_goal[4] = 0

            arm.go(joint_goal, wait=True)
            arm.stop() 

            joint_goal[0] = 0.785
            joint_goal[1] = 0.279
            joint_goal[2] = 0.524
            joint_goal[3] = 0
            joint_goal[4] = 0
            arm.go(joint_goal, wait=True)
            arm.stop() 

            j += 1    

        joint_goal[0] = 0.0
        joint_goal[1] = 0.279
        joint_goal[2] = 0.524
        joint_goal[3] = 0
        joint_goal[4] = 0
        arm.go(joint_goal, wait=True)
        arm.stop()

        i += 1
        j = 0

    homming()          
        

def joy_callback(joy_msg):
    global axes
    global buttons
    axes = joy_msg.axes
    buttons = joy_msg.buttons
        

def full_control():
    rate = rospy.Rate(5)  # Frecuencia de actualización (Hz)
    global scale

    while not rospy.is_shutdown():
    
        #x = axes[1] * scale
        #z = axes[3] * scale
        x = scale
        z = scale  

        if buttons[0] == 1:
            scale = 0.01

        elif buttons[2] == 1:
            scale = 0.1

        elif buttons[1] == 1:
            bon_dia()

        elif buttons[3] == 1:
            homming()

        elif buttons[4] == 1:
            current_joint_positions = arm.get_current_joint_values()
            joint_angle_increment = 0.1          
            current_joint_positions[3] += joint_angle_increment

            set_joint_value_target(current_joint_positions)
        
        elif buttons[5] == 1:
            current_joint_positions = arm.get_current_joint_values()
            joint_angle_increment = 0.1          
            current_joint_positions[3] -= joint_angle_increment

            set_joint_value_target(current_joint_positions)
        

                
    
        if axes[1] > 0.8: #Mou el brass en l'eix x
            current_pose = arm.get_current_pose().pose
            current_pose.position.x += x
            set_pose_target_arm(current_pose)

        elif axes[1] < -0.8: #Mou el brass en l'eix x
            current_pose = arm.get_current_pose().pose
            current_pose.position.x -= x
            set_pose_target_arm(current_pose)
        
        elif axes[3] > 0.8: #Mou el brass en l'eix z
            current_pose = arm.get_current_pose().pose
            current_pose.position.z += z
            set_pose_target_arm(current_pose)

        elif axes[3] < -0.8: #Mou el brass en l'eix z
            current_pose = arm.get_current_pose().pose
            current_pose.position.z -= z
            set_pose_target_arm(current_pose)
        
        elif abs(axes[2]) > 0.8: #Només rota l'end effector del brass
            current_joint_positions = arm.get_current_joint_values()
            joint_angle_increment = 0.08  
        
            if axes[2] < -0.8:
                current_joint_positions[4] += joint_angle_increment
            elif axes[5] < -0.8:
                current_joint_positions[4] -= joint_angle_increment

            set_joint_value_target(current_joint_positions)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joystick_full_control', anonymous=True)
        arm = MoveGroupCommander('arm_group')
        #arm.set_planning_pipeline_id("pilz_industrial_motion_planner")
        #arm.set_planner_id("LIN")
        arm.set_planning_pipeline_id("ompl")
        arm.set_planner_id("RRT")
        joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
        full_control()

    except rospy.ROSInterruptException:
        pass