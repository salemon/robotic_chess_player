#!/usr/bin/env python3
import rospy
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
from avt_camera import *
import os 
import cv2
import numpy as np

class ChessboardStateDetection:
    #ros service, request: string, response: string

    def __init__(self):
        self.server = rospy.Service('board_state',TaskPlanning,self.serviceHandler)
        self.camera = AvtCamera()
        
        K = rospy.get_param('/camera_calibration/K')
        self.camera_matrix = np.array(K).reshape((3,3))
        D = rospy.get_param('/camera_calibration/D')
        self.dist_coeff = np.array(D)
        
    def serviceHandler(self,msg):
        if msg.request ==  'state':
            self.state()
        else:
            self.square_dict = eval(msg.request)
            return TaskPlanningResponse('Receive square image dictionary') 
    
    def state(self):
     
        parent_dir = os.getcwd()
        path = os.path.join(parent_dir, '_')
        os.mkdir(path)
        
        self.camera.trigger_image()
        image = self.__undistortImage(self.camera.lastest_img)
        for word, value in self.square_dict.items():
            y1,y2,x1,x2 = value
            img = image[y1:y2,x1:x2,:]
            name = os.path.join(path, '{}.jpg'.format(word))
            cv2.imwrite(name,img)

    def __undistortImage(self,image):
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.camera_matrix,self.dist_coeff,(w,h),1,(w,h))
        dst = cv2.undistort(image, self.camera_matrix, self.dist_coeff, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst  
if __name__ == "__main__":
    rospy.init_node('neural_network')
    robot = ChessboardStateDetection()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)