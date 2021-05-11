#!/usr/bin/env python
import os
import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
import numpy as np
from numpy.linalg import inv,norm
from robot_manipulator import *
from vision_detector import *
from transformation import Trans3D

#gripper height gripper service import os
class RobotService:
    
    def __init__(self):

        self.server = rospy.Service('auto_service',TaskPlanning,self.serviceHandler)
        self.manipulator = RobotManipulator()
        self.detector = VisionDetector()

        self.camera_matrix = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        cam_rot = np.array([-0.00321568, -0.0135531, -0.0116504, 0.999835])
        cam_trans = np.array([-0.001561951275522637, -0.08589372872499101, 0.09399786754775818])
        self.TCP2camera_pose = Trans3D.from_quaternion(cam_rot, cam_trans)

        self.trigger = rospy.Publisher('/trigger', String, queue_size=1)
        self.img_sub = rospy.Subscriber('avt_camera_img', Image, self.image_callback)
        self.lastest_img = None
        self.img_received = False 
        self.bridge = CvBridge()

    def serviceHandler(self,msg):
        rospy.loginfo("Request: {}".format(msg.request)) 
        
        if msg.request == 'detect chessboard':
            feedback = self.detectChessboard()
            return TaskPlanningResponse(feedback)

        elif msg.request[:4] == 'auto':
            detail = msg.request.split(':')
            self.carryOutOrder(detail[1])
            return TaskPlanningResponse('Done')

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

    def detectChessboard(self):
        self.manipulator.goToJointState([90,-135,90,-70,-90,0.0])
        self.trigger_image()
        base2TCP_pose = self.manipulator.robotCurrentPose()
        base2chessboard_pose = self.detector.chessboardPose(self.lastest_img,base2TCP_pose)
        self.__takeImagePose(base2chessboard_pose)
        self.manipulator.goStraightToPose(self.camera_pose)
        base2TCP_pose = self.manipulator.robotCurrentPose()
        self.trigger_image()
        self.base2chessboard_pose,self.square_dict = self.detector.chessboardSquare(self.lastest_img, base2TCP_pose)
        self.__gameStandby()
        self.manipulator.goToJointState(self.standby)
        return "Detection accomplished"

    def __gameStandby(self):
        tvect = self.camera_pose.to_tvec()
        x, y = tvect[0] + 0.1333025, tvect[1]
        angle = np.arccos(x / norm(np.array([x,y]))) / np.pi * 180
        self.standby = [angle,-135,90,-70,-90,0.0]
        return None

    def __takeImagePose(self,base2chessboard_pose):
        z = (self.camera_matrix[0][0] * (-0.18)) / (400 - self.camera_matrix[0][2])
        tfmatrix = base2chessboard_pose.to_tfmatrix()
        tfmatrix[:,-1] = np.matmul(tfmatrix,np.array([0.18,0.18,-z,1]))
        tfmatrix = np.matmul(tfmatrix,inv(self.TCP2camera_pose.to_tfmatrix()))
        self.camera_pose = Trans3D.from_tfmatrix(tfmatrix)
        return self.camera_pose
    
    def carryOutOrder(self,piece):
        #alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        parent_dir = os.getcwd()
        path = os.path.join(parent_dir, piece)
        os.mkdir(path)
        alf_dict = {'h':0,'g':1,}
        pickup_dict = {'k':0.065,'q':0.065,'b':0.04,'n':0.037,'r':0.037,'p':0.03}
        pickup_height = pickup_dict[piece.lower()] + 0.1338
        raiseup_height = pickup_height + 0.007
        for word,alf in alf_dict.items():
            if alf != 0:
                sq1, start1, end1 = word+str(1), end2, (0,alf)
                self.pickAndPlace(start1,end1,pickup_height,raiseup_height)
                sq2, start2, end2 = word+str(2), end1, (1,alf)
                self.pickAndPlace(start2,end2,pickup_height,raiseup_height)
                self.manipulator.goStraightToPose(self.camera_pose)
                self.crop_image([sq1,sq2],path)
            else:
                self.manipulator.goStraightToPose(self.camera_pose)
                sq1,sq2 = word+str(1),word+str(2)
                self.crop_image([sq1,sq2],path)
            for num in range(0,3):
                value1,value2 = (2*num+4),(2*num+3)
                sq1, start1, end1 = word+str(value1), (2*num+1,alf), (2*num+3,alf)
                self.pickAndPlace(start1,end1,pickup_height,raiseup_height)
                sq2, start2, end2 = word+str(value2), (2*num,alf), (2*num+2,alf)
                self.pickAndPlace(start2,end2,pickup_height,raiseup_height)
                self.manipulator.goStraightToPose(self.camera_pose)
                self.crop_image([sq1,sq2],path)
        return 'Finished'

    def pickAndPlace(self,start,end,pickup_height,raiseup_height):
        waypoints = self.pickAndPlaceWaypoints(start,end,pickup_height,raiseup_height)
        self.manipulator.executePlan(waypoints)

    def crop_image(self,square,path):
        self.trigger_image()
        for sq in square:
            p = self.square_dict[sq]
            square_img = self.lastest_img[p[0]:p[1],p[2]:p[3]]
            name = os.path.join(path, '{}.jpg'.format(sq))
            cv2.imwrite(name,square_img)
        
    def squareToIndex(self,square):
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        return (int(square[1])-1, alf_dict[square[0]])

    def waypoint_generator(self,square,height):
        unit_length = 0.045/2
        tfmatrix = self.base2chessboard_pose.to_tfmatrix().copy()
        tfmatrix[:,-1] = np.matmul(tfmatrix,np.array([(2*square[1]+1)*unit_length,(2*square[0]+1)*unit_length,-height,1]))
        tfmatrix_pose = Trans3D.from_tfmatrix(tfmatrix)
        return tfmatrix_pose

    def pickAndPlaceWaypoints(self,start,end,pickup_height,raiseup_height):
        pickup = self.waypoint_generator(start, pickup_height)
        start_raiseup = self.waypoint_generator(start, raiseup_height)
        end_raiseup = self.waypoint_generator(end, raiseup_height)
        dropoff = self.waypoint_generator(end, pickup_height-0.003)
        waypoints = [pickup, start_raiseup, 0, end_raiseup, dropoff, 1]
        return waypoints

if __name__ == "__main__":
    rospy.init_node('robot_system')
    robot = RobotService()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)