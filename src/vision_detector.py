import os
import rospy
import numpy as np
from numpy.linalg import inv, norm
from transformation import Trans3D
from chessboard_pose_estimation import *
from avt_camera import *
from robotic_chess_player.srv import *
import cv2

class VisionDetector:

    def __init__(self):
        K = rospy.get_param('/camera_calibration/K')
        self.cam_mtx = np.array(K).reshape((3,3))
        D = rospy.get_param('/camera_calibration/D')
        self.dist = np.array(D)
        self.TCP2camera_pose = Trans3D.from_ROSParameterServer("/hand_eye_position")
        self.pose_estimator = ChessboardPoseEstimation(self.cam_mtx, self.dist)
        self.camera = AvtCamera()
        #add chessboard parameter to config
    def takeImagePose(self, base2TCP_pose):
        self.camera.trigger_image()
        camera2chessbaord_pose = self.pose_estimator.camera_chessboard(self.camera.lastest_img)
        base2chessboard_pose = base2TCP_pose * self.TCP2camera_pose * camera2chessbaord_pose
        z = (self.cam_mtx[0][0] * (-4*0.43)) / (100 - self.cam_mtx[0][2])
        point_pose = Trans3D.from_tvec(np.array([4*0.43, 4*0.43, -z+0.1338]))
        inv_TCP2camera_pose = Trans3D.from_tfmatrix(inv(self.TCP2camera_pose.to_tfmatrix()))
        return [base2chessboard_pose * point_pose * inv_TCP2camera_pose]
      
    def poseAndSquare(self,base2TCP_pose):
        self.camera.trigger_image()
        square_dict,camera2chessbaord_pose,square_draw = self.pose_estimator.estimateSquare(self.camera.lastest_img)
        self.square_dict = square_dict
        return str(square_dict), base2TCP_pose * self.TCP2camera_pose * camera2chessbaord_pose, str(square_draw)
        
    def chessboardState(self):
        try:
            state_msg = self.nn_client('state').feedback
            board = np.array([list(i) for i in state_msg.split(',')])
            return board
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def crop_image(self,image, square, path,count):
        image = self.__undistortImage(image)
        for sq in square:
            p = self.square_dict[sq]
            square_img = image[p[0]:p[1],p[2]:p[3]]
            name = os.path.join(path, '{}.jpg'.format(count))
            cv2.imwrite(name,square_img)
            count += 1
        return count

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

    def takeImage(self):
        self.camera.trigger_image()
        cv2.imshow('img',self.camera.lastest_img)
        cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()

