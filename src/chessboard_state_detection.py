#!/usr/bin/env python3
import rospy
from robotic_chess_player.srv import TaskPlanning,TaskPlanningResponse
from avt_camera import *
import os 
import cv2
import numpy as np
import torch
from torchvision import transforms

class NNvision:
    #ros service, request: string, response: string
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    processing = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224,224)),
        transforms.ToTensor(),
        transforms.Normalize([0.4129, 0.3452, 0.2850], [0.2895, 0.2620, 0.2181])])

    def __init__(self):
        self.server = rospy.Service('board_state',TaskPlanning,self.serviceHandler)
        self.camera = AvtCamera()
        
        K = rospy.get_param('/camera_calibration/K')
        self.camera_matrix = np.array(K).reshape((3,3))
        D = rospy.get_param('/camera_calibration/D')
        self.dist_coeff = np.array(D)
        
        self.model = self.load_model(nn_path)
        self.board = None
    def serviceHandler(self,msg):
        if msg.request ==  'state':
            state = self.detectingState()
            return TaskPlanningResponse(state) 
        else:
            self.square_dict = eval(msg.request)
            return TaskPlanningResponse('Receive square image dictionary') 
    
    @staticmethod
    def load_model(path):
        model = torch.load(path)
        model.eval() 
        model = model.to(NNVision.device)
        return model

    def __undistortImage(self):
        self.camera.trigger_image()
        h,  w = self.camera.latest_img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.camera_matrix,self.dist_coeff,(w,h),1,(w,h))
        dst = cv2.undistort(self.camera.latest_img, self.camera_matrix, self.dist_coeff, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

    def __board2msg(self,chessboard):
        msg = ''
        for row in chessboard:
            for entry in row:
                msg += entry
            msg += ','
        return msg[:-1]

    def detectingState(self):
        image = self.__undistortImage(self.camera.lastest_img)
        col = {'A':7,'B':6,'C':5,'D':4,'E':3,'F':2,'G':1,'H':0}
        row = {'1':0,'2':1,'3':2,'4':3,'5':4,'6':5,'7':6,'8':7}
        class_names = ['B', 'K', 'N', 'P', 'Q', 'R', '_', 'b', 'k', 'n', 'p', 'q', 'r']
        chessboard = np.zeros((8,8),dtype=str)
        img_list,key_list,count = [],[],0
        for key,value in self.square.items():
            y1,y2,x1,x2 = value
            img = image[y1:y2,x1:x2,:]
            img_tt = NNVision.processing(img)
            img_list.append(img_tt)
            key_list.append(key)
            count += 1
            if count == 8:
                inputs = torch.stack(img_list,0)
                with torch.no_grad():
                    inputs = inputs.to(NNVision.device)
                    outputs = self.model(inputs)
                    _, preds = torch.max(outputs,1)
                    for k in key_list:
                        chessboard[row[k[1]],col[k[0]]] = class_names[preds[key_list.index(k)]]
                    img_list,key_list,count = [],[],0
                    del inputs
                    torch.cuda.empty_cache()
        chessboard_msg = self.__board2msg(chessboard)
        return chessboard_msg

if __name__ == "__main__":
    rospy.init_node('neural_network')
    robot = ChessboardStateDetection()
    rospy.spin()
    #cv2.imshow("img",img)
    #cv2.waitKey(0)