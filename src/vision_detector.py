import os
import numpy as np
from transformation import Trans3D
from chessboard_pose_estimation import *
#from chessboard_state_detection import *



class VisionDetector:
    

    def __init__(self):
        self.cam_mtx = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        self.dist = np.array([-0.0588151813531173, 0.05245363676337366, 0.000101400909359754, -0.001346375977094263, 0.02585377839443043])
        self.pose_estimator = ChessboardPoseEstimation(self.cam_mtx, self.dist)

        #self.state_detector = ChessboardStateDetection(nn_path)
        #TODO: obtain form ROS parameter
        cam_rot = np.array([-0.00285051, -0.000809386, 0.00617178, 0.999977])
        cam_trans = np.array([0.001514679603077936, -0.08438965970995699, 0.09423193500454446])
        self.TCP2camera_pose = Trans3D.from_quaternion(cam_rot, cam_trans)
        pass
        
    def chessboardPose(self, image, base2TCP_pose):
        camera2chessbaord_pose = self.pose_estimator.estimatePose(image)
        return base2TCP_pose * self.TCP2camera_pose * camera2chessboard_pose
        
    def chessboardSquare(self,image,base2TCP_pose):
        self.square_dict,camera2chessbaord_pose = self.pose_estimator.estimateSquare(image)
        return base2TCP_pose * self.TCP2camera_pose * camera2chessboard_pose
        
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

