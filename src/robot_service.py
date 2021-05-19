#!/usr/bin/env python
from io import BytesIO as StringIO
import os
import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv,norm
from avt_camera import *
from motion_planning import *
from vision_detector import *
from transformation import Trans3D

class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.manipulator = MotionPlanner()
        self.detector = VisionDetector()
        self.board = None

    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request))
        if msg.request == "to standby":
            self.manipulator.moveRobotJoint([[90,-135,90,-70,-90,0.0]])
            return TaskPlanningResponse('Robot arrive general standby position') 
        
        elif msg.request == 'detect chessboard':
            feedback = self.detectChessboard()
            return TaskPlanningResponse(feedback)
        
        elif msg.request == 'chessboard state':
            fen = self.chessboardState()
            return TaskPlanningResponse(fen)

        elif msg.request[:4] == 'move:':
            detail = msg.request.split(':')
            self.carryOutOrder(detail[1])
            return TaskPlanningResponse('Done')
        
        elif msg.request[:4] == 'auto':
            detail = msg.request.split(':')
            self.collectData(detail[1])
            return TaskPlanningResponse('Done')
        
        elif msg.request[:2] == 'to':
            detail = msg.request.split(':')
            self.toSquare(detail[1])
            return TaskPlanningResponse('Done')

    def detectChessboard(self):
        self.manipulator.moveRobotJoint([[90,-135,90,-70,-90,0.0]])
        rospy.sleep(1)
        base2TCP_pose = self.manipulator.currentRobotPose()
        base2chessboard_pose = self.detector.takeImagePose(base2TCP_pose)
        self.manipulator.moveRobot(base2chessboard_pose)
        rospy.sleep(1)
        self.base2TCP_pose = self.manipulator.currentRobotPose()
        self.base2chessboard_pose = self.detector.poseAndSquare(self.base2TCP_pose)
        self.spot = self.__dropPieceSpot()
        self.standby = self.__gameStandby()
        self.manipulator.moveRobotJoint([self.standby])
        return "Detection accomplished"

    def chessboardState(self):
        self.manipulator.moveRobot([self.base2TCP_pose])
        chessboard = self.detector.chessboardState()
        if type(self.board) == type(None):
            chessboard = self.__humanRevise(chessboard)
        else:
            chessboard = self.__systemRevise(chessboard)
            chessboard = self.__humanRevise(chessboard)
        self.board = chessboard
        return self.__board2fen(self.board)

    def __board2fen(self,board):
        board = np.rot90(board,2)
        with StringIO() as s:
            for row in board:
                empty = 0
                for cell in row:
                    if cell != '_':
                        if empty > 0:
                            s.write(str(empty))
                            empty = 0
                        s.write(cell)
                    else:
                        empty += 1
                if empty > 0:
                    s.write(str(empty))
                s.write('/')
            # Move one position back to overwrite last '/'
            s.seek(s.tell() - 1)
            # If you do not have the additional information choose what to put
            s.write(' b KQkq - 0 1')
            return s.getvalue()

    def __dropPieceSpot(self):
        unit_length = 0.045/2
        x, y = (2*10-1) * unit_length, (2*8-1) * unit_length
        point = Trans3D.from_tvec(np.array([x, y, 0]))
        return self.base2chessboard_pose * point

    def __gameStandby(self):
        tvect = self.base2TCP_pose.to_tvec()
        x = np.array([tvect[0] - 0.132, tvect[1]])
        angle = np.arccos(-(x / norm(x))[0]) / np.pi * 180
        return [angle,-135,90,-70,-90,0.0]      

    def __humanRevise(self,chessboard):
        col = {'a':7,'b':6,'c':5,'d':4,'e':3,'f':2,'g':1,'h':0}
        print(chessboard)
        while True:
            ready = raw_input('Are there any piece needs to revise? ')
            try:
                if ready != '':
                    square,piece = ready.split(' ')
                    chessboard[int(square[1])-1,col[square[0]]] = piece
                    print(chessboard)
                else:break
            except:
                continue
                
        return chessboard
    
    def __systemRevise(self,chessboard):
        print('here')
        try:
            for row in range(8):
                for col in range(8):
                    if not chessboard[row,col].isupper() and self.board[row,col].islower() and chessboard[row,col] != self.board[row,col]: 
                        chessboard[row,col] = self.board[row,col]
                else:continue 
        except TypeError:
            pass
        return chessboard

    def squareToIndex(self,square):
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        return (int(square[1])-1, alf_dict[square[0]])

    def carryOutOrder(self,order):
        #decode order
        detail = order.split(',')
        if len(detail) == 4:
            #change piece 
            pass
        else:
            step,capturing,castling = detail
            start,end = step[:2],step[2:]
            if capturing == 'yes':
                #b3c4,yes,no
                piece, raiseup_height = self.__raiseUpHeight(end,'spot','capturing')
                print('here')
                self.pickAndPlace('R',end,'spot',raiseup_height)
                piece,raiseup_height = self.__raiseUpHeight(start,end,'move')
                self.pickAndPlace('q',start,end,raiseup_height)
                self.manipulator.moveRobotJoint([self.standby])
                pass
            elif castling == 'yes':
                'e1g1,no,yes'
                pass
            else:
                'e2e4,no,no'
                pass
        return 'Finished'
    
    def __takingImage(self):
        self.manipulator.moveRobot([self.base2TCP_pose])
        rospy.sleep(1)
        self.camera.trigger_image()
        #self.camera.lastest_img = self.takeImage('217.jpg')
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
                        self.detector.crop_image(self.camera.lastest_img,[sq1,sq2],path)
                    elif num == 1:
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pickAndPlace(piece,pre_sq1,sq1)
                        sq2, pre_sq2 = word+str(num+1), sq2
                        self.pickAndPlace(piece,pre_sq2,sq2)
                        self.__takingImage()
                        self.detector.crop_image(self.camera.lastest_img,[sq1,sq2],path)
                    else:
                        sq2, pre_sq2 = word+str(num+1), sq2
                        self.pickAndPlace(piece,pre_sq2,sq2)
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pickAndPlace(piece,pre_sq1,sq1)
                        self.__takingImage()
                        self.detector.crop_image(self.camera.lastest_img,[sq1,sq2],path)
        else:
            self.__takingImage()
            sq1 = 'h1'
            self.detector.crop_image(self.camera.lastest_img,[sq1],path)
            for word in alf_list:
                for num in range(1,9):
                    if word == 'h' and num == 1:continue
                    else:
                        sq1, pre_sq1 = word+str(num), sq1
                        self.pickAndPlace(piece,pre_sq1,sq1)
                        self.__takingImage()
                        self.detector.crop_image(self.camera.lastest_img,[sq1],path)
        return 'Finished'

    def toSquare(self,square):
        square_pose = self.__squarePose(square)
        self.manipulator.moveRobot([square_pose])
        return 'Done'

    def __squarePose(self,square):
        "x : alf , y : num"
        if square == 'spot':
            return self.spot
        else:
            unit_length = 0.045/2
            alf_dict = {'h':1,'g':2,'f':3,'e':4,'d':5,'c':6,'b':7,'a':8}
            x, y = (2*alf_dict[square[0]]-1) * unit_length, (2*(int(square[1:]))-1) * unit_length
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

    def __raiseUpHeight(self,start,end,action):
        '''
        row = [start[0], end[0]]
        col = [start[1], end[1]]
        chessboard = self.chessboard.copy()
        piece = chessboard[start[0]][start[1]]
        '''
        piece = 'q'
        if action == 'capturing':
            return piece, 0.125
        if action == 'castling':
            return piece, 0.065
        else:
            piece_height = {'k':0.105,'q':0.095,'b':0.08,'n':0.06,'r':0.06,'p':0.05,'_':0.01}
            if piece.lower() == 'n':
                chessboard[start[0]][start[1]] = '_'
                passing_area = chessboard[min(row):max(row)+1,min(col):max(col)+1]
                return piece, max([piece_height[i.lower()] for i in passing_area.reshape((passing_area.size,))])
            else: return piece, 0.01
    
    def __raiseUpPose(self,raiseup_height,start_pose,end_pose):
        ra_s_pose = start_pose * Trans3D.from_tvec(np.array([0,0,-raiseup_height]))
        ra_e_pose = end_pose * Trans3D.from_tvec(np.array([0,0,-raiseup_height]))
        return ra_s_pose,ra_e_pose

    def pickAndPlace(self,piece, start, end, raiseup_height):
        s_pose, e_pose = self.__squarePose(start), self.__squarePose(end)
        ab_s_pose, ab_e_pose = self.__aboveSquarePose(s_pose), self.__aboveSquarePose(e_pose)
        pickup_pose, dropoff_pose, pickup_height = self.__pickDropPose(piece,s_pose,e_pose)
        raiseup_height = pickup_height + raiseup_height
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