import sys
import numpy as np
import rospy
import rospkg
import geometry_msgs.msg
from std_msgs.msg import String
from transformation import Trans3D
import moveit_commander
import moveit_msgs.msg
'''
rospack = rospkg.RosPack()
path = rospack.get_path('execution_node')
gripper_path = path+'/include/robotiq_hande_ros_driver'
sys.path.append(gripper_path)
from robotiq_hande_ros_driver.srv import gripper_service
'''

class RobotManipulator(object):
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        group_name = "arm"#manipulator ur5e.srdf<group name="manipulator">
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        #self.gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)

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

    def goStraightToPose(self,pose):
        move_group = self.move_group
        waypoints = [pose.to_Pose()]
        (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)#(waypoints,eef_stpe,jump_threshold)
        move_group.execute(plan, wait=True)
        return None
        '''
    
        (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)#(waypoints,eef_stpe,jump_threshold)
        move_group.execute(plan, wait=True)
        pose_msg = move_group.get_current_pose()
        pose = Trans3D.from_PoseStamped(pose_msg)
        return pose
        '''
    
if __name__ == "__main__":
    rospy.init_node('robot_execute_command')
    robot = MotionPlanner()
    rospy.spin()
