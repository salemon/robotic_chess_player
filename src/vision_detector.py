import os
import numpy as np
from transformation import Trans3D
from chessboard_pose_estimation import *
#from chessboard_state_detection import *

class VisionDetector:

    def __init__(self,cam_mtx,dist,TCP2camera_pose):
        self.cam_mtx = cam_mtx
        self.dist = dist
        self.pose_estimator = ChessboardPoseEstimation(self.cam_mtx, self.dist)
        self.TCP2camera_pose = TCP2camera_pose

    def __adjustError(self,camera2chessbaord_pose,base2TCP_pose):
        pose = base2TCP_pose * self.TCP2camera_pose * camera2chessbaord_pose
        pose_tfmatrix = pose.to_tfmatrix()
        pose_tfmatrix[2,3] += 0.0185
        return Trans3D.from_tfmatrix(pose_tfmatrix)

    def chessboardPose(self, image, base2TCP_pose):
        camera2chessbaord_pose = self.pose_estimator.estimatePose(image)
        return self.__adjustError(camera2chessbaord_pose,base2TCP_pose)
        
    def chessboardSquare(self,image,base2TCP_pose):
        self.square_dict,camera2chessbaord_pose = self.pose_estimator.estimateSquare(image)
        return self.__adjustError(camera2chessbaord_pose,base2TCP_pose)
        
    def chessboardState(self,image):
        board = self.state_detector.detecting(image)
        return board

    def crop_image(self,image, square, path):
        image = self.__undistortImage(image)
        for sq in square:
            p = self.square_dict[sq]
            square_img = image[p[0]:p[1],p[2]:p[3]]
            name = os.path.join(path, '{}.jpg'.format(sq))
            cv2.imwrite(name,square_img)

    def __undistortImage(self,image):
        h,  w = image.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.cam_mtx,self.dist,(w,h),1,(w,h))
        dst = cv2.undistort(image, self.cam_mtx, self.dist, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst
    
    def chessboardTOFen(self,board):
        fen = self.state_detector.boardTOFen(board)
        return fen

