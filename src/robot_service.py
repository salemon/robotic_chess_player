#!/usr/bin/env python
import os
import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv,norm
from motion_planning import *
from vision_detector import *
from transformation import Trans3D

class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.manipulator = MotionPlanner()

        K = rospy.get_param('/camera_calibration/K')
        self.camera_matrix = np.array(K).reshape((3,3))
        D = rospy.get_param('/camera_calibration/D')
        self.dist_coeff = np.array(D)
        self.TCP2camera_pose = Trans3D.from_ROSParameterServer("/hand_eye_position")
        self.detector = VisionDetector(self.camera_matrix,self.dist_coeff,self.TCP2camera_pose)

        self.board = None
        self.trigger = rospy.Publisher('/trigger', String, queue_size=1)
        self.img_sub = rospy.Subscriber('avt_camera_img', Image, self.image_callback)
        self.lastest_img = None
        self.img_received = False 
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.lastest_img = img.copy()
        self.img_received = True 

    def trigger_image(self):
        self.trigger.publish(String('Hi!'))
        while(not self.img_received):
            continue
        self.img_received = False

    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request))
        if msg.request == "to standby":
            self.manipulator.moveRobotJoint([[90,-135,90,-70,-90,0.0]])
            return TaskPlanningResponse('Robot arrive general standby position') 
        
        elif msg.request == 'detect chessboard':
            feedback = self.detectChessboard()
            return TaskPlanningResponse(feedback)
        
        elif msg.request == 'chessboard state':
            fen_string,self.chessboard = self.chessboardState()
            return TaskPlanningResponse(fen_string)

        elif msg.request[:4] == 'step':
            detail = msg.request.split(':')
            self.carryOutOrder(detail[1])
            return TaskPlanningResponse('Done')
        
        elif msg.request[:4] == 'auto':
            detail = msg.request.split(':')
            self.collectData(detail[1])
            return TaskPlanningResponse('Done')
        
        elif msg.request[:4] == 'move':
            detail = msg.request.split(':')
            self.toSquare(detail[1])
            return TaskPlanningResponse('Done')

    def detectChessboard(self):
        self.manipulator.moveRobotJoint([[90,-135,90,-70,-90,0.0]])
        self.trigger_image()
        #self.lastest_img = self.takeImage("standby.jpg")
        # sleep 1 seconds. Wait robot to stablize.
        rospy.sleep(1)
        base2TCP_pose = self.manipulator.currentRobotPose()
        base2chessboard_pose = self.detector.chessboardPose(self.lastest_img,base2TCP_pose)
        self.manipulator.moveRobot(self.__takeImagePose(base2chessboard_pose))
        self.base2TCP_pose = self.manipulator.currentRobotPose()
        self.trigger_image()
        #self.lastest_img = self.takeImage("217.jpg")
        self.base2chessboard_pose = self.detector.chessboardSquare(self.lastest_img, self.base2TCP_pose)
        print(self.base2chessboard_pose.to_string())
        self.__gameStandby()
        self.manipulator.moveRobotJoint([self.standby])
        return "Detection accomplished"

    def __gameStandby(self):
        tvect = self.base2TCP_pose.to_tvec()
        x, y = tvect[0] - 0.132, tvect[1]
        angle = np.arccos(x / norm(np.array([x,y]))) / np.pi * 180
        self.standby = [angle,-135,90,-70,-90,0.0]
        return None

    def __takeImagePose(self,base2chessboard_pose):
        # calculate the pose for camera to take image
        z = (self.camera_matrix[0][0] * (-0.18)) / (400 - self.camera_matrix[0][2])
        point_pose = Trans3D.from_tvec(np.array([0.18,0.18,-z+0.1338]))
        inv_TCP2camera_pose = Trans3D.from_tfmatrix(inv(self.TCP2camera_pose.to_tfmatrix()))
        return [base2chessboard_pose * point_pose * inv_TCP2camera_pose]
 
    def chessboardState(self):
        image = self.takeImage(file_name)
        board = self.detector.chessboardState(image)
        if self.board == None:
            self.board = self.__humanCheck(board)
        else:
            self.board = self.__systemCheck(board)
        return self.detector.chessboardTOFen(self.board)

    def __humanCheck(self,board):
        pass
    def __systemCheck(self,board):
        pass       

    def squareToIndex(self,square):
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        return (int(square[1])-1, alf_dict[square[0]])

    def carryOutOrder(self,detail):
        square,capturing,castling = detail.split(',')
        start, end = self.squareToIndex(square[:2]), self.squareToIndex(square[2:])
        pickup_dict = {'k':0.065,'q':0.065,'b':0.04,'n':0.037,'r':0.037,'p':0.03}
        if capturing == 'yes':
            pass
        elif castling == 'yes':
            pass
        else:
            #piece = (self.chessboard[start[0]][start[1]]).lower()
            piece = 'k'
            pickup_height = pickup_dict[piece]
            if piece != 'n':
                raiseup_height = pickup_height + 0.007
            if piece == 'n':
                raiseup_height = self.raiseUpKnight(start,end) + pickup_height
            waypoints = self.pickAndPlaceWaypoints(start,end,pickup_height,raiseup_height)
            self.manipulator.goStraightToPose(waypoints)
        return 'Finished'
    
    def __takingImage(self):
        self.manipulator.moveRobot([self.base2TCP_pose])
        rospy.sleep(1)
        self.trigger_image()
        #self.lastest_img = self.takeImage('217.jpg')
        return None

    def collectData(self,piece):
        parent_dir = os.getcwd()
        alf_list = ['h','g','f','e','d','c','b','a']
        path = os.path.join(parent_dir, piece)
        os.mkdir(path)
        if piece.lower() != 'k':
            for word in alf_list:
                for num in range(1,9,2):
                    if word == 'h' and num == 1:
                        self.__takingImage()
                        sq1, sq2 = word + str(num), word + str(num+1)
                        self.detector.crop_image(self.lastest_img,[sq1,sq2],path)
                    elif num == 1:
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pickAndPlace(piece,pre_sq1,sq1)
                        sq2, pre_sq2 = word+str(num+1), sq2
                        self.pickAndPlace(piece,pre_sq2,sq2)
                        self.__takingImage()
                        self.detector.crop_image(self.lastest_img,[sq1,sq2],path)
                    else:
                        sq2, pre_sq2 = word+str(num+1), sq2
                        self.pickAndPlace(piece,pre_sq2,sq2)
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pickAndPlace(piece,pre_sq1,sq1)
                        self.__takingImage()
                        self.detector.crop_image(self.lastest_img,[sq1,sq2],path)
        else:
            self.__takingImage()
            sq1 = 'h1'
            self.detector.crop_image(self.lastest_img,[sq1],path)
            for word in alf_list:
                for num in range(1,9):
                    if word == 'h' and num == 1:continue
                    else:
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pickAndPlace(piece,pre_sq1,sq1)
                        self.__takingImage()
                        self.detector.crop_image(self.lastest_img,[sq1],path)

    
        return 'Finished'
    def toSquare(self,square):
        square_pose = self.__squarePose(square)
        self.manipulator.moveRobot([square_pose])
        return 'Done'

    def __squarePose(self,square):
        "x : alf , y : num"
        unit_length = 0.045/2
        alf_dict = {'h':1,'g':2,'f':3,'e':4,'d':5,'c':6,'b':7,'a':8}
        x, y = (2*alf_dict[square[0]]-1) * unit_length, (2*(int(square[1]))-1) * unit_length
        point = Trans3D.from_tvec(np.array([x, y, 0]))
        return self.base2chessboard_pose * point

    def __aboveSquarePose(self, square_pose):
        point = Trans3D.from_tvec(np.array([0,0,-0.1]))
        return square_pose * point

    def __pickDropPose(self,piece,start_pose,end_pose):
        pickup_dict = {'k':0.062,'q':0.059,'b':0.0475,'n':0.0358,'r':0.0327,'p':0.025}
        pickup_height = pickup_dict[piece.lower()]
        pickup_pose = start_pose * Trans3D.from_tvec(np.array([0,0,-pickup_height]))
        dropoff_pose = end_pose * Trans3D.from_tvec(np.array([0,0,-pickup_height + 0.0006]))
        return pickup_pose, dropoff_pose, pickup_height

    def __raiseUpKight(self,start,end):
        piece_height = {'k':0.105,'q':0.095,'b':0.08,'n':0.06,'r':0.06,'p':0.05,'_':0.01}
        row = [start[0], end[0]]
        col = [start[1], end[1]]
        chessboard = self.chessboard.copy()
        chessboard[start[0]][start[1]] = '_'
        passing_area = chessboard[min(row):max(row)+1,min(col):max(col)+1]
        return max([piece_height[i.lower()] for i in passing_area.reshape((passing_area.size,))])
    
    def __raiseUpPose(self,raiseup_height,start_pose,end_pose):
        ra_s_pose = start_pose * Trans3D.from_tvec(np.array([0,0,-raiseup_height]))
        ra_e_pose = end_pose * Trans3D.from_tvec(np.array([0,0,-raiseup_height]))
        return ra_s_pose,ra_e_pose

    def pickAndPlace(self, piece, start, end):
        '''pick and place piece from one square to another
        which piece, start square and end square
        1. need to calculate the gripper above the piece(heigher than during moving)
        2. need the pick up height at start square
        3. raise up height for moving at square square
            3.1 if raise up is not knight raiseup certain amout
            3.2 else: check the 
        4. raise up height for moving at end square
        5. drop off height at end square end square
        6. gripper above the piece at end square
         '''
        s_pose, e_pose = self.__squarePose(start), self.__squarePose(end)
        ab_s_pose, ab_e_pose = self.__aboveSquarePose(s_pose), self.__aboveSquarePose(e_pose)
        pickup_pose, dropoff_pose, pickup_height = self.__pickDropPose(piece,s_pose,e_pose)
        '''
        if piece != 'n':
            raiseup_height = pickup_height + 0.01
        else:
            raiseup_height = self.__raiseUpKight(start,end)
        '''
        raiseup_height = pickup_height + 0.01
        ra_s_pose,ra_e_pose = self.__raiseUpPose(raiseup_height,s_pose,e_pose)
        waypoints = [[ab_s_pose, pickup_pose], 0,[ra_s_pose, ra_e_pose,dropoff_pose], 1, [ab_e_pose]]
        self.manipulator.moveRobotWaypoints(waypoints)
        return None

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)