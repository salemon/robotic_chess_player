#!/usr/bin/env python3
import rospy
from robotic_chess_player.srv import RobotService,RobotServiceResponse
from avt_camera import *
import cv2
import numpy as np
import torch
from torchvision import transforms
import rospkg
from cv_bridge import CvBridge,CvBridgeError

class NNVision:
    #ros service, request: string, response: string
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    processing = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224,224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

    def __init__(self):
        self.server = rospy.Service('board_state',RobotService,self.serviceHandler)
        self.camera = AvtCamera()
        K = rospy.get_param('/camera_calibration/K')
        self.camera_matrix = np.array(K).reshape((3,3))
        D = rospy.get_param('/camera_calibration/D')
        self.dist_coeff = np.array(D)
        rospack = rospkg.RosPack()
        nn_path = rospack.get_path('robotic_chess_player')+'/src/best.pth'
        self.model = self.load_model(nn_path)
        rospy.loginfo('neural network is ready to go!')

    def serviceHandler(self,msg):
        if msg.request ==  'state':
            state = self.detectingState()
            return RobotServiceResponse(state) 
        else:
            self.square = eval(msg.request)
            return RobotServiceResponse('neural network Received square info') 
    
    @staticmethod
    def load_model(path):
        model = torch.load(path)
        model.eval() 
        model = model.to(NNVision.device)
        return model

    def __undistortImage(self):
        self.camera.trigger_image()
        h,  w = self.camera.lastest_img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.camera_matrix,self.dist_coeff,(w,h),1,(w,h))
        dst = cv2.undistort(self.camera.lastest_img, self.camera_matrix, self.dist_coeff, None, newcameramtx)
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
        image = self.__undistortImage()
        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        col = {'a':7,'b':6,'c':5,'d':4,'e':3,'f':2,'g':1,'h':0}
        class_names = ['_', 'b', 'k', 'n', 'p', 'q', 'r', 'B', 'K', 'N', 'P', 'Q', 'R']
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
                        chessboard[int(k[1])-1,col[k[0]]] = class_names[preds[key_list.index(k)]]
                    img_list,key_list,count = [],[],0
                    del inputs
                    torch.cuda.empty_cache()
        chessboard_msg = self.__board2msg(chessboard)
        return chessboard_msg

if __name__ == "__main__":
    rospy.init_node('neural_network')
    robot = NNVision()
    rospy.spin()
