#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np 
import numpy.linalg as LA
from manipulate import *
from detect import *
from transformation import Trans3D
from robotic_chess_player.srv import TaskPlanning, TaskPlanningResponse

class RobotService:

    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.robot_manipulator = MotionManipulator()
        self.vision_detector = VisionDetector()

    def serviceHandler(self, req):
        rospy.loginfo("received player service: {}".format(req.request))
        if req.request == "detect chessboard":
            feedback = self.detectChessboard()
            return TaskPlanningResponse(feedback)

    def detectChessboard(self):
        general_standby = [0.0,-45,-90,-70,90,0.0]
        manipulator = self.robot_manipulator
        robot_pose = manipulator.go_to_joint_state(general_standby)
        image = self.takeImage()
        base2chessboard_pose = self.vision_detector.chessboardPose(image,robot_pose)
        camera_matrix = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        z = (camera_matrix[0][0]*(-0.18))/(400-camera_matrix[0][2])
        above_mtx = base2chessboard_pose.to_tfmatrix().copy()
        above_mtx[:,-1] = np.matmul(above_mtx,np.array([0.18,0.18,-z,1]))
        above_pose = Trans3D.from_tfmatrix(above_mtx)
        robot_pose = manipulator.go_cartesian(above_pose)
        #calcuate the pose right above the chessboard 
        #move toward the that position return robot pose record
        #robot_pose = manipulator.go_to_pose(mtx)
        #take image
        #image = self.takeImage()
        #detecting chessboard position save square_dict, return square's pose 
        #new function to calculate each square's position from return square's pose
        #self.square_pose = 
        #self.vision_detector.chessboarbSquare
        #back to new standby position
        
        return 'Accomplished'
    def moveChess(self, request):
        ## decode the request
        chess, stat_pos, end_pos = self.decodeRequest(request)

        ## call movechess

    def __removeChess(self, chess, pos):
        '''
        remove a chess from the chessboard
        '''
        table_pos = None
        self.moveChess(chess, pos, tale_pos)

    def __moveChess(self, chess, start_pos, end_pos):
        '''
        move chess from point A to point B
        '''

    def takeImage(self):
        img = cv2.imread('standby.jpg')
        '''       
        msg = rospy.wait_for_message('avt_camera_img', Image)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        '''
        return img

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()