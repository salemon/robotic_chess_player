#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv,norm
from robot_manipulator import *
from vision_detector import *
from transformation import Trans3D

class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.manipulator = RobotManipulator()
        self.detector = VisionDetector()

        self.camera_matrix = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        cam_rot = np.array([-0.00285051, -0.000809386, 0.00617178, 0.999977])
        cam_trans = np.array([0.001514679603077936, -0.08438965970995699, 0.09423193500454446])
        self.TCP2camera_pose = Trans3D.from_quaternion(cam_rot, cam_trans)

    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request))
        if msg.request == "to general standby":
            self.manipulator.goToJointState([90,-135,90,-70,-90,0.0])
            return TaskPlanningResponse('Robot arrive general standby position') 
        
        elif msg.request == "detect chessboard":
            feedback = self.detectChessboard()
            return TaskPlanningResponse(feedback)

    def detectChessboard(self):
        self.manipulator.goToJointState([90,-135,90,-70,-90,0.0])
        current = self.manipulator.robotCurrentPose()
        image = self.takeImage('standby.jpg')
        base2TCP_pose = self.manipulator.robotCurrentPose()
        base2chessboard_pose = self.detector.chessboardPose(image,base2TCP_pose)
        self.takeImagePose(base2chessboard_pose)
        self.manipulator.goStraightToPose(self.camera_pose)
        image = self.takeImage('217.jpg')
        self.base2chessboard_pose = self.detector.chessboardSquare(image, base2TCP_pose)
        self.game_standby()
        self.manipulator.goToJointState(self.standby)
        return "Detection accomplished"

    def game_standby(self):
        tvect = self.camera_pose.to_tvec()
        x, y = tvect[0] + 0.1333025, tvect[1]
        angle = np.arccos(x / norm(np.array([x,y]))) / np.pi * 180
        self.standby = [angle,-135,90,-70,-90,0.0]
        return None

    def takeImagePose(self,base2chessboard_pose):
        z = (self.camera_matrix[0][0] * (-0.18)) / (400 - self.camera_matrix[0][2])
        tfmatrix = base2chessboard_pose.to_tfmatrix()
        tfmatrix[:,-1] = np.matmul(tfmatrix,np.array([0.18,0.18,-z,1]))
        tfmatrix = np.matmul(tfmatrix,inv(self.TCP2camera_pose.to_tfmatrix()))
        self.camera_pose = Trans3D.from_tfmatrix(tfmatrix)
        return self.camera_matrix

    def takeImage(self,file_name):
        img = cv2.imread(file_name)
        return img

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)