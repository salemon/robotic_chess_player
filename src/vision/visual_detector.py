import os
import rospy
import numpy as np
from pose_estimation import *
from feature_extraction import *
from numpy.linalg import inv
import rospkg
r = rospkg.RosPack()
path_1 = r.get_path('robotic_chess_player') + '/src/include'
import sys
sys.path.append(path_1)
from transformation import Trans3D
from avt_camera import *

class VisualDetector:

    def __init__(self, edge):
        self.CAM_MTX = np.array(rospy.get_param('/camera_calibration/K')).reshape((3,3))
        self.DIST = np.array(rospy.get_param('/camera_calibration/D'))
        self.EDGE = edge
        self.pose_estimator = PoseEstimation(self.CAM_MTX, self.DIST, self.EDGE)
        self.camera = AvtCamera()
        self.feature_extractor = FeatureExtraction()
        self.TCP2camera_pose = Trans3D.from_ROSParameterServer("/hand_eye_position")
        self.HEIGHT = -0.156

    def closer_view_pose(self, base2TCP_pose):
        self.camera.trigger_image()
        corner, image_points = self.feature_extractor.general_pose_points(self.camera.lastest_img)
        rotation_vector, translation_vector = self.pose_estimator.general_position(corner, image_points)
        camera2chessboard_pose = Trans3D.from_angaxis(rotation_vector, tvec=translation_vector)
        z = (self.CAM_MTX[1][1] / (1 - self.CAM_MTX[1][2]) * (-4 * self.EDGE) + 
        self.CAM_MTX[0][0] / (200 - self.CAM_MTX[0][2]) * (-4 * self.EDGE)) / 2
        inv_TCP2camera_pose = Trans3D.from_tfmatrix(inv(self.TCP2camera_pose.to_tfmatrix()))
        return base2TCP_pose * self.TCP2camera_pose * camera2chessboard_pose * Trans3D.from_tvec(np.array([4*self.EDGE, 4*self.EDGE, -z])) * inv_TCP2camera_pose
        
    def position_pose(self,base2TCP_pose):
        self.camera.trigger_image()
        image_points = self.feature_extractor.closer_view_estimation_points(self.camera.lastest_img)
        rotation_vector, translation_vector = self.pose_estimator.locate_chessboard(image_points)
        camera2chessboard_pose = Trans3D.from_angaxis(rotation_vector, tvec=translation_vector)
        pose = base2TCP_pose * self.TCP2camera_pose * camera2chessboard_pose
        base2chessboard_pose = pose * Trans3D.from_tvec(np.array([0,0,self.HEIGHT]))
        z = (self.CAM_MTX[1][1] / (150 - self.CAM_MTX[1][2]) * (-4 * self.EDGE) + 
        self.CAM_MTX[0][0] / (200 - self.CAM_MTX[0][2]) * (-4 * self.EDGE)) / 2
        inv_TCP2camera_pose = Trans3D.from_tfmatrix(inv(self.TCP2camera_pose.to_tfmatrix()))
        square_detection_pose = pose * Trans3D.from_tvec(np.array([4*self.EDGE, 4*self.EDGE, -z])) * inv_TCP2camera_pose
        x, y = (2*11-1) * self.EDGE / 2, (2*8-1) * self.EDGE / 2
        drop_piece_spot = base2chessboard_pose * Trans3D.from_tvec(np.array([x, y, 0]))
        return base2chessboard_pose, square_detection_pose, drop_piece_spot

    def generate_square_dict(self):
        self.camera.trigger_image()
        image_points = self.feature_extractor.closer_view_estimation_points(self.camera.lastest_img)
        rotation_vector, translation_vector = self.pose_estimator.locate_chessboard(image_points)
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        self.square_dict = {}
        def constant(value):
            return 4.082*(value)**2 - 36.735*(value) + 95
        for world,alf in alf_dict.items():
            for num in range(8):
                key = world + str(num+1)
                axis = np.float32([[alf*self.EDGE, num*self.EDGE, 0],[(alf+1)*self.EDGE, (num+1)*self.EDGE, 0]]).reshape(-1,3)
                imgpts,_ = cv2.projectPoints(axis, rotation_vector, translation_vector, self.CAM_MTX, np.zeros((5,)))
                imgpts = np.int64(imgpts)
                square_ul, square_lr = tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel())
                alf_constant,num_constant = constant(alf+1),constant(num+1)
                self.square_dict[key] = [square_ul[1]+int(num_constant*(num/7-8/7)),square_lr[1]+int(num_constant*(num/7-1/7)),
                square_ul[0]+int(alf_constant*((alf+1)/7-8/7)),square_lr[0]+int(alf_constant*((alf+1)/7-1/7))]
        return str(self.square_dict)

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
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.CAM_MTX,self.DIST,(w,h),1,(w,h))
        dst = cv2.undistort(image, self.CAM_MTX, self.DIST, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

if __name__ == "__main__":
    vision = VisualDetector()
    img = cv2.imread('frame0001.jpg')
    #points = vision.closer_view_estimation_points(img)
    #count = 1
    image = vision.square_dict(img)

    cv2.imshow('img',image)
    cv2.waitKey(0) & 0xFF
    cv2.destroyAllWindows()