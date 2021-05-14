#!/usr/bin/env python
# include the ikfast lib
import rospy
from sensor_msgs.msg import JointState
import rospkg
r = rospkg.RosPack()
path = r.get_path('robotic_chess_player') + '/src/include/ikfastpy/'
print(path)
import sys
sys.path.append(path)
import ikfastpy
import numpy as np
import tf2_ros
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from transformation import Trans3D 

def max_joint_diff(wp1, wp2):
    '''
    find the lagest joint angle difference for two waypoint
    '''
    diff_max = 0
    for i in range(len(wp1)):
        diff = abs(wp1[i] - wp2[i])
        if diff > diff_max:
            diff_max = diff
    return diff_max

class MotionPlanner:
    '''
    simple motion planner
    '''
    def __init__(self):
        self.kin = ikfastpy.PyKinematics()
        
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        self.lastest_joint_state = [0.0] * 6
        self.name_map = {'shoulder_pan_joint': 0, 
                       'shoulder_lift_joint': 1, 
                       'elbow_joint': 2,
                       'wrist_1_joint': 3,
                       'wrist_2_joint': 4,
                       'wrist_3_joint': 5}

        # connect to robot controller
        rospy.loginfo("Attempting connection to robot action server.....")
        self.robot_client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for robot server...")
        self.robot_client.wait_for_server()
        rospy.loginfo("Connected to robot server")

        # tf buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # speed configuration 
        # TODO: read speed config from file
        self.joint_speed = 100.0/180.0*np.pi     # 100 degree/second 
        self.ee_speed = 0.4
        self.ee_acc = 0.2
        self.acc_time = self.ee_speed / self.ee_acc
        self.ee_rot_speed = np.pi

    def joint_state_callback(self, msg):
        joint_idx = []
        for idx, name in enumerate(msg.name):
            if name in self.name_map:
                joint_idx.append(self.name_map[name]) 
            else:
                joint_idx.append(-1)

        for idx, ang in enumerate(msg.position):
            if joint_idx[idx] != -1:
                self.lastest_joint_state[joint_idx[idx]] = ang
    
    def forward_kin(self, joint_angles):
        pose = self.kin.forward(joint_angles)
        pose = np.asarray(pose).reshape(3,4)
        return Trans3D.from_tfmatrix(pose)

    def pose_diff(self, waypoint1, waypoint2):
        '''
        compute the relative pose from waypoint1 (joint angles) to waypoint2 (joint angles).

        Args:
            waypoint1 (np array): first joint angles
            waypoint2 (np array): second joint angles
        Returns:
            distant (double): distance of end-effector in meters between two waypoint
            d_angle (double): rotation angles between two waypoints
        '''
        wp1_pose = self.forward_kin(waypoint1)
        wp2_pose = self.forward_kin(waypoint2)
        diff_tfmatrix = np.linalg.inv(wp1_pose.to_tfmatrix()).dot(wp2_pose.to_tfmatrix())
        d_tvec = diff_tfmatrix[0:3,3]
        distant = np.linalg.norm(d_tvec)
        d_rot = Trans3D.from_tfmatrix(diff_tfmatrix).to_angaxis()
        d_angle = np.linalg.norm(d_rot)
        return distant, d_angle

    def create_trajectories(self, trajectory_list, speed_scal=1, unit='degree'):

        if unit == 'degree':
            # Convert angle from degrees to radians
            for i in range(trajectory_list.__len__()):
                for j in range(trajectory_list[0].__len__()):
                    trajectory_list[i][j] *= np.pi / 180.0 
        elif unit != 'rad':
            rospy.logerr("invalid unit: {}".format(unit))

        # Initialize trajectory object
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Add each point in trajectory_list to the trajectory object
        g.trajectory.points = []
        current_joint_pos = self.lastest_joint_state
        counter = 0
        current_t = 0
        for joint_angles in trajectory_list:
            ang_diff = max_joint_diff(current_joint_pos, joint_angles)
            dist, rot_dist = self.pose_diff(current_joint_pos, joint_angles)
            # maximum joint speed
            t1 = ang_diff / self.joint_speed 
            # acc then maximum EE speed then dacc
            test = self.ee_acc * self.acc_time * self.acc_time
            if dist < test:
                t2 = np.sqrt(dist / self.ee_acc)
            else:
                t2 = (dist - test)/self.ee_speed + (2 * self.acc_time)
            # maximum EE rot speed
            t3 = rot_dist / self.ee_rot_speed 
            t = max((t1,t2,t3))
            current_t += t / speed_scal
            g.trajectory.points.append(JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(current_t)))
            current_joint_pos = joint_angles
        return(g)

    def ikSolve(self, pose, example_solution, debug=False):
        '''
        Find the ik solution that is closest to the example solution 

        Args:
            pose (Trans3D): end-effector pose 
            example_solution (np 1D array): example solution as a joint angle list
        
        Returns:
            solution (np 1D array): ik solution
        '''
        # assemble transformation
        tf_matrix = pose.to_tfmatrix()
        transformation_matrix = tf_matrix[0:3, :]
        ik_solutions = self.kin.inverse(transformation_matrix.reshape(-1).tolist())
        n_solutions = int(len(ik_solutions)/6)
        ik_solutions = np.asarray(ik_solutions).reshape(n_solutions, 6)
        
        if debug:
            print("{} solutions found".format(n_solutions))
            for ik_sol in ik_solutions:
                print(ik_sol)

        min_dist = 1000
        closest_sol = None
        def match_solution(sol, example_sol):
            '''
            extend the angle from [-pi, pi] to [-2pi, 2pi]
            '''
            for i in range(6):
                ang1 = sol[i]
                if ang1 < 0:
                    ang2 = ang1 + 2*np.pi
                else:
                    ang2 = ang1 - 2*np.pi
                if abs(ang1 - example_sol[i]) > abs(ang2 - example_sol[i]):
                    sol[i] = ang2
            return sol

        for ik_sol in ik_solutions:
            ik_sol = match_solution(ik_sol, example_solution)
            diff = ik_sol - example_solution
            dist = np.linalg.norm(diff)
            if dist < min_dist:
                closest_sol = ik_sol
                min_dist = dist

        if debug:
            print("found cloest solution: {}".format(closest_sol))

        return closest_sol
    
    def jog(self, tvec, speed_scale=1):
        '''
        jog the robot by tvec vector
        '''
        current_pose = self.currentRobotPose()
        move_pose = Trans3D.from_tvec(tvec)
        next_pose = current_pose * move_pose
        self.moveRobot([next_pose], speed_scale)
    
    def moveRobot(self, pose_list, speed_scale=1, start_joint_pos=None):
        if start_joint_pos is None:
            start_joint_pos = self.lastest_joint_state

        # ik solution for each pose 
        trajectory_list = []
        current_joint_pos = start_joint_pos
        for pose in pose_list:
            current_joint_pos = self.ikSolve(pose, current_joint_pos)
            trajectory_list.append(current_joint_pos)
        
        # generate joint trajectory list
        msg = self.create_trajectories(trajectory_list, speed_scale, 'rad')

        # send to robot controler
        self.robot_client.send_goal(msg)
        try:
            self.robot_client.wait_for_result()
        except KeyboardInterrupt:
            self.robot_client.cancel_goal() 
            raise
    
    def moveRobotJoint(self, joint_pos_list, speed_scale=1, unit='degree'):
        msg = self.create_trajectories(joint_pos_list, speed_scale, unit)
        # send to robot controler
        self.robot_client.send_goal(msg)
        try:
            self.robot_client.wait_for_result()
        except KeyboardInterrupt:
            self.robot_client.cancel_goal() 
            raise
        
    def currentRobotPose(self):
        try:
            trans = self.tfBuffer.lookup_transform('base', 'tool0_controller', rospy.Time.now(), rospy.Duration(0.5))
        except:
            rospy.logerr("Failed to get the current robot pose.")
        return Trans3D.from_TransformStamped(trans) 

if __name__ == "__main__":
    rospy.init_node('motion_planning_test')
    mp = MotionPlanner()
    rospy.sleep(1)
    mp.jog(np.array([0,0,0.05]))
    rospy.sleep(1)
    mp.jog(np.array([0,0,-0.05]))
    rospy.sleep(1)
    mp.jog(np.array([0,0.05,0.05]))
    rospy.sleep(1)
    mp.jog(np.array([0,-0.05,-0.05]))