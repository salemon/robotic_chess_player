#!/usr/bin/env python
import sys
import numpy as np
import rospy
import rospkg
import geometry_msgs.msg
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg
'''
rospack = rospkg.RosPack()
path = rospack.get_path('execution_node')
gripper_path = path+'/include/robotiq_hande_ros_driver'
sys.path.append(gripper_path)
from robotiq_hande_ros_driver.srv import gripper_service
'''
from robotic_chess_player.srv import MotionCommand, MotionCommandResponse

class MotionPlanner():
    def __init__(self):
        ## initialize service
        self.server = rospy.Service('command_for_robot',MotionCommand,self.service_handler)
        self.manipulator = RobotManipulator()

    def service_handler(self,req):
        action,info = req.order,req.detail
        rospy.loginfo("Robot needs to do: {}".format('action'))
        if action == "go to joint state":
            feedback = self.manipulator.go_to_joint_state(info)
            return MotionCommandResponse(feedback)
        

    def moveStraightLine(self, start_pose, end_pose):
        pass

class RobotManipulator(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        group_name = "arm"#manipulator ur5e.srdf<group name="manipulator">
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        #self.gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)
    
    def go_to_joint_state(self,info):
        state = [float(i)/180*np.pi for i in info.split(',')]
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        for i in range(len(state)):
            joint_goal[i] = state[i]
        move_group.go(joint_goal,wait = True)
        return 'Done'

if __name__ == "__main__":
    rospy.init_node('robot_execute_command')
    robot = MotionPlanner()
    rospy.spin()
