#!/usr/bin/env python
import rospy
from robotic_chess_player.srv import RobotService,RobotServiceResponse
import numpy as np
from numpy.linalg import norm
from motion_planning import *
from visual_detector import *
from transformation import Trans3D
from avt_camera import *

class RobotServer:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',RobotService,self.serviceHandler)
        self.manipulator = MotionPlanner()
        self.board = None
        self.standby = (90,-135,90,-70,-90,0.0)
        self.EDGE = 0.043
        self.detector = VisualDetector(self.EDGE)
        self.camera = AvtCamera()

    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request))
        if msg.request == "to standby":
            self.manipulator.moveRobotJoint([[i for i in self.standby]])
            return RobotServiceResponse('Robot arrive standby position') 
        
        elif msg.request == 'locate chessboard':
            feedback = self.locate_chessboard()
            return RobotServiceResponse(feedback)
        
        elif msg.request[:2] == 'to':
            detail = msg.request.split(':')
            self.toSquare(detail[1])
            return RobotServiceResponse('Done')

        elif msg.request == 'detect chessboard state':
            self.manipulator.moveRobot([self.base2TCP_pose])
            return RobotServiceResponse('Robot arrive taking image position')

    def locate_chessboard(self):
        try:
            self.manipulator.moveRobotJoint([[i for i in self.standby]])
            rospy.sleep(1)
            base2TCP_pose = self.manipulator.currentRobotPose()
            closer_pose = self.detector.closer_view_pose(base2TCP_pose)
            self.manipulator.moveRobot([closer_pose])
            rospy.sleep(1)
            base2TCP_pose = self.manipulator.currentRobotPose()
            self.base2chessboard_pose, self.base2TCP_pose, self.spot = self.detector.position_pose(base2TCP_pose)
            self.manipulator.moveRobot([self.base2TCP_pose])
            rospy.sleep(1)
            str_square_dict = self.detector.generate_square_dict()
            self.standby = self.__gameStandby()
            self.manipulator.moveRobotJoint([[i for i in self.standby]])
            self.capture_piece = 0
            return 'Done;'+ str_square_dict
        except:
            return 'Fail'

    def __gameStandby(self):
        tvect = self.manipulator.currentRobotPose().to_tvec()
        x = np.array([tvect[0] - 0.132, tvect[1]])
        angle = np.arccos(-(x / norm(x))[0]) / np.pi * 180
        return (float(angle), -135, 90, -70, -90, 0.0)   

    def toSquare(self,square):
        square = square.split(',')
        square_pose = self.__squarePose(square[0])
        print('square translation: ',square_pose.to_tvec())
        if len(square) == 1: 
            self.manipulator.moveRobot([square_pose])
        elif len(square) ==  2:
            pickup_height = float(square[1])
            pickup_pose = square_pose * Trans3D.from_tvec(np.array([0,0,-pickup_height]))
            self.manipulator.moveRobot([pickup_pose])
        return 'Done'

    def __squarePose(self,square):
        "x : alf , y : num"
        if square == 'spot':
            return self.spot
        else:
            unit_length = self.EDGE/2
            alf_dict = {'h':1,'g':2,'f':3,'e':4,'d':5,'c':6,'b':7,'a':8}
            x, y = (2*alf_dict[square[0]]-1) * unit_length, (2*(int(square[1:]))-1) * unit_length
            point = Trans3D.from_tvec(np.array([x, y, 0]))
            square_pose = (self.base2chessboard_pose * point).to_tfmatrix()
            return Trans3D.from_tfmatrix(square_pose)

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotServer()
    rospy.spin()
