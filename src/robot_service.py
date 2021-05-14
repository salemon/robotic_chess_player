#!/usr/bin/env python
import os
import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv,norm
from motion_planning import MotionPlanner
from vision_detector import *
from transformation import Trans3D

class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('robot_service',TaskPlanning,self.serviceHandler)
        self.manipulator = MotionPlanner(simulation=True)
        self.detector = VisionDetector()

        self.camera_matrix = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        cam_rot = np.array([-0.00285051, -0.000809386, 0.00617178, 0.999977])
        cam_trans = np.array([0.001514679603077936, -0.08438965970995699, 0.09423193500454446])
        self.TCP2camera_pose = Trans3D.from_quaternion(cam_rot, cam_trans)

        self.board = None
        #self.trigger = rospy.Publisher('/trigger', String, queue_size=1)
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
        if msg.request == "to general standby":
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

    def detectChessboard(self):
        self.manipulator.moveRobotJoint([[90,-135,90,-70,-90,0.0]])
        #self.trigger_image()
        self.lastest_img = self.takeImage('standby.jpg')
        # sleep 1 seconds. Wait robot to stablize.
        rospy.sleep(1)
        base2TCP_pose = self.manipulator.currentRobotPose()
        self.base2chessboard_pose = self.detector.chessboardPose(self.lastest_img,base2TCP_pose)
        print("base to chessboard: ",self.base2chessboard_pose.to_string())
        self.manipulator.moveRobot(self.__takeImagePose(self.base2chessboard_pose))
        self.base2TCP_pose = self.manipulator.currentRobotPose()
        #self.trigger_image()
        #base2chessboard_pose = self.detector.chessboardSquare(self.lastest_img, self.base2TCP_pose)
        #self.__gameStandby()
        #self.manipulator.moveRobotJoint([self.standby])
        return "Detection accomplished"

    def __gameStandby(self):
        tvect = self.base2TCP_pose.to_tvec()
        x, y = tvect[0] + 0.1333025, tvect[1]
        angle = np.arccos(x / norm(np.array([x,y]))) / np.pi * 180
        self.standby = [angle,-135,90,-70,-90,0.0]
        return None

    def __takeImagePose(self,base2chessboard_pose):
        z = (self.camera_matrix[0][0] * (-0.18)) / (400 - self.camera_matrix[0][2])
        tfmatrix[:,-1] = np.matmul(tfmatrix,np.array([0.18,0.18,-z,1]))
        tfmatrix = np.matmul(tfmatrix,inv(self.TCP2camera_pose.to_tfmatrix()))
        return [Trans3D.from_tfmatrix(tfmatrix)]
    
    
    def takeImage(self,file_name):
        return cv2.imread(file_name)
 
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

    def collectData(self,piece):
        #alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        parent_dir = os.getcwd()
        path = os.path.join(parent_dir, piece)
        os.mkdir(path)
        alf_dict = {'h':0}
        pickup_height = self.__pickupHeight(piece)
        raiseup_height = pickup_height + 0.01
        for word,alf in alf_dict.items():
            if alf != 0:
                sq1, start1, end1 = word+str(1), end2, (0,alf)
                self.pickAndPlace(start1,end1,pickup_height,raiseup_height)
                sq2, start2, end2 = word+str(2), end1, (1,alf)
                self.pickAndPlace(start2,end2,pickup_height,raiseup_height)
                self.manipulator.goToPose(self.base2TCP_pose)
                rospy.sleep(1)
                self.trigger_image()
                square = [sq1,sq2]
                self.detector.crop_image(self.lastest_img,square,path)
            else:
                self.manipulator.goToPose(self.base2TCP_pose)
                sq1,sq2 = word+str(1),word+str(2)
                rospy.sleep(1)
                self.trigger_image()
                square = [sq1,sq2]
                self.detector.crop_image(self.lastest_img,square,path)

            for num in range(0,3):
                value1,value2 = (2*num+4),(2*num+3)
                sq1, start1, end1 = word+str(value1), (2*num+1,alf), (2*num+3,alf)
                self.pickAndPlace(start1,end1,pickup_height,raiseup_height)
                sq2, start2, end2 = word+str(value2), (2*num,alf), (2*num+2,alf)
                self.pickAndPlace(start2,end2,pickup_height,raiseup_height)
                pose = self.manipulator.robotCurrentPose()
                print("moveit: ",pose.to_string())
                #self.manipulator.goToPose(self.base2TCP_pose)
                rospy.sleep(1)
                self.trigger_image()
                square = [sq1,sq2]
                self.detector.crop_image(self.lastest_img,square,path)
                break
        return 'Finished'

    def squareToIndex(self,square):
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        return (int(square[1])-1, alf_dict[square[0]])

    def __pickupHeight(self,piece):
        #pickup_dict = {'k':0.09,'q':0.09,'b':0.06,'n':0.05,'r':0.05,'p':0.04}
        #pickup_height = pickup_dict[piece.lower()] + 0.135
        pickup_height = 0.20
        return pickup_height

    def pickAndPlace(self,start,end,pickup_height,raiseup_height):
        waypoints = self.pickAndPlaceWaypoints(start,end,pickup_height,raiseup_height)
        self.manipulator.executePlan(waypoints)

    def raiseUpKnight(self,start,end):
        piece_height = {'k':0.105,'q':0.095,'b':0.08,'n':0.06,'r':0.06,'p':0.05,'_':0.007}
        row = [start[0], end[0]]
        col = [start[1], end[1]]
        chessboard = self.chessboard.copy()
        chessboard[start[0]][start[1]] = '_'
        passing_area = chessboard[min(row):max(row)+1,min(col):max(col)+1]
        return max([piece_height[i.lower()] for i in passing_area.reshape((passing_area.size,))])
    
    def __waypointGenerator(self,square,height):
        unit_length = 0
        tfmatrix = self.base2chessboard_pose.to_tfmatrix().copy()
        tfmatrix[:,-1] = np.matmul(tfmatrix, np.array([(2*square[1]+1)*unit_length,(2*square[0]+1)*unit_length,-height,1]))
        tfmatrix_pose = Trans3D.from_tfmatrix(tfmatrix)
        return tfmatrix_pose

    def pickAndPlaceWaypoints(self,start,end,pickup_height,raiseup_height):
        pickup = self.__waypointGenerator(start, pickup_height)
        start_raiseup = self.__waypointGenerator(start, raiseup_height)
        end_raiseup = self.__waypointGenerator(end, raiseup_height)
        dropoff = self.__waypointGenerator(end, pickup_height-0.003)
        waypoints = [pickup]
        print("calculated: ", pickup.to_string())
        #waypoints = [pickup, 0, start_raiseup, end_raiseup, dropoff, 1]
        return waypoints

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)