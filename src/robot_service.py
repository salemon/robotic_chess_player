#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv
from robot_manipulator import *
from vision_detector import *
from transformation import Trans3D

class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.manipulator = RobotManipulator()
        self.detector = VisionDetector()
    
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
        image = self.takeImage('standby.jpg')
        base2TCP_pose = self.manipulator.robotCurrentPose()
        base2chessboard_pose = self.detector.chessboardPose(image,base2TCP_pose)
        print(base2chessboard_pose.to_string())
        '''
        above_mtx = base2TCP_pose.to_tfmatrix().copy()
        above_mtx[:,-1] = np.matmul(above_mtx,np.array([0.0,0.0,0.0,1]))
        above_pose = Trans3D.from_tfmatrix(above_mtx)
        self.manipulator.goStraightToPose(above_pose)
        above_mtx = base2TCP_pose.to_tfmatrix().copy()
        above_mtx[:,-1] = np.matmul(above_mtx,np.array([0.0,0.1,0.0,1]))
        above_pose = Trans3D.from_tfmatrix(above_mtx)
        self.manipulator.goStraightToPose(above_pose)
        '''
        self.manipulator.goStraightToPose(base2chessboard_pose)
        return "arrive"


    def takeImage(self,file_name):
        img = cv2.imread(file_name)
        return img

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)