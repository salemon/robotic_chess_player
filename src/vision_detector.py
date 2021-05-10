import numpy as np
import geometry_msgs.msg
from chessboard_pose_estimation import *
from tf.transformations import euler_matrix,quaternion_matrix
from transformation import Trans3D
from numpy.linalg import inv


class VisionDetector:
    

    def __init__(self):
        cam_mtx = np.array([1353.131570942828, 0, 758.6928458558336, 0, 1353.743967167117, 557.9749908957598, 0, 0, 1]).reshape((3,3))
        dist = np.array([-0.0588151813531173, 0.05245363676337366, 0.000101400909359754, -0.001346375977094263, 0.02585377839443043])
        self.pose_estimator = ChessboardPoseEstimation(cam_mtx, dist)

        #TODO: obtain form ROS parameter
        cam_rot = np.array([-0.00285051, -0.000809386, 0.00617178, 0.999977])
        cam_trans = np.array([0.001514679603077936, -0.08438965970995699, 0.09423193500454446])
        self.TCP2camera_pose = Trans3D.from_quaternion(cam_rot, cam_trans)
        pass
    
    def __baseToChessboard(self,camera2chessbaord_pose,base2TCP_pose):
        camera2chessboard_tfmatrix = camera2chessbaord_pose.to_tfmatrix()
        TCP2camera_tfmatrix = self.TCP2camera_pose.to_tfmatrix()
        base2TCP_tfmatrix = base2TCP_pose.to_tfmatrix()
        #base2chessboard_tfmatrix = np.matmul(base2TCP_tfmatrix,TCP2camera_tfmatrix)
        base2chessboard_tfmatrix = np.matmul(base2TCP_tfmatrix,np.matmul(TCP2camera_tfmatrix,camera2chessboard_tfmatrix))
        return Trans3D.from_tfmatrix(base2chessboard_tfmatrix)

    def chessboardPose(self, image, base2TCP_pose):
        camera2chessbaord_pose = self.pose_estimator.estimatePose(image)
        base2chessboard_pose = self.__baseToChessboard(camera2chessbaord_pose,base2TCP_pose)
        return base2chessboard_pose
        
    
    def chessboardSquare(self,image,base2TCP_pose):
        square_dict,camera2chessbaord_pose = self.pose_estimator.estimateSquare(image)
        base2chessboard_pose = self.generatePose(camera2chessboard_tfmatrix)
        self.square_dict = square_dict
        return base2chessboard_pose
        

