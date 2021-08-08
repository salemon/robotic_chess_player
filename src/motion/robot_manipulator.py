#!/usr/bin/env python
import sys
import numpy as np
import rospy

import geometry_msgs.msg
from std_msgs.msg import String
from transformation import Trans3D
import moveit_commander
import moveit_msgs.msg

import rospkg
rospack = rospkg.RosPack()
path = rospack.get_path('robotic_chess_player')
gripper_path = path+'/include/robotiq_hande_ros_driver'
sys.path.append(gripper_path)
from robotiq_hande_ros_driver.srv import gripper_service

class RobotManipulator(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        self.gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)
       
    def robotCurrentPose(self):
        return Trans3D.from_PoseStamped(self.move_group.get_current_pose())

    def goToJointState(self,state):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        for i in range(len(state)):
            joint_goal[i] = float(state[i])/180*np.pi
        move_group.go(joint_goal,wait = True)
        pose_msg = move_group.get_current_pose()
        pose = Trans3D.from_PoseStamped(pose_msg)
        return pose
    
    def goToPose(self,pose):
        self.move_group.set_pose_target(pose.to_Pose())
        self.move_group.go(wait = True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return None

    def goStraightToPose(self,pose):
        waypoints = [pose.to_Pose()]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,0.001,0.0)#(waypoints,eef_stpe,jump_threshold)
        self.move_group.execute(plan, wait=True)
        return None
    
    def executePlan(self,waypoints):
        for pose in waypoints:
            if type(pose)!= int:
                self.goStraightToPose(pose)
            else:
                if pose == 0: self.gripper_srv(position=255, speed=255, force=1)
                else: self.gripper_srv(position=100, speed=255, force=1)

                
if __name__ == "__main__":
    rospy.init_node('robot_execute_command')
    robot = RobotManipulator()
    pose = robot.robotCurrentPose()
    print(pose.to_string())
    rospy.spin()
