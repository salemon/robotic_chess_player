#!/usr/bin/env python
import cv2
import numpy as np
from transformation import Trans3D
from feature_extractor import *
class ChessboardPoseEstimation():

    def __init__(self,cam_mtx,dist):
        self.camera_matrix = cam_mtx
        self.dist_coeff = dist
        self.feature_extractor = FeatureExtractor()
        
    @staticmethod
    def object_coordinate(corner,l = 4.3):
        #each row is a point col is the world frame coordinate define upper_left as origin
        if corner == 'upper_left':
            return np.array([[l, l, 0],[2*l, l, 0],[3*l, l, 0],[l, 2*l, 0],
            [l, 3*l, 0],[2*l, 2*l, 0],[2*l, 3*l , 0],[3*l, 2*l, 0],[3*l, 3*l, 0]])/100
        elif corner == 'upper_right':
            return np.array([[7*l, l, 0],[6*l, l, 0],[5*l, l, 0],[7*l, 2*l, 0],
            [7*l, 3*l, 0],[6*l, 2*l, 0],[6*l, 3*l, 0],[5*l, 2*l, 0],[5*l, 3*l, 0]])/100
        elif corner == 'lower_left':
            return np.array([[l, 7*l, 0],[2*l, 7*l, 0],[3*l, 7*l, 0],[l, 6*l, 0],
            [l, 5*l, 0],[2*l, 6*l, 0],[2*l, 5*l, 0],[3*l, 6*l, 0],[3*l, 5*l, 0]])/100
        else:
            return np.array([[7*l, 7*l, 0],[6*l, 7*l,0 ],[5*l, 7*l, 0],[7*l, 6*l, 0],
            [7*l, 5*l, 0],[6*l, 6*l, 0],[6*l, 5*l, 0],[5*l, 6*l, 0],[5*l, 5*l, 0]])/100
    
    def locate_chessboard(self,image):
        img = cv2.GaussianBlur(image, (5,5),0)
        corner,image_points = self.feature_extractor.pose_estimation_points(img)
        object_points = self.object_coordinate(corner)
        _, rotation_vector, translation_vector = cv2.solvePnP(
                object_points, image_points, self.camera_matrix,self.dist_coeff)

        def draw(img, imgpts):
            corner = tuple(imgpts[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (255,0,0), 5)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,255,0), 5)
            img = cv2.line(img, corner, tuple(imgpts[3].ravel()), (0,0,255), 5)
            return img
        axis = np.float32([[0,0,0],[0.045,0,0], [0,0.045,0], [0.0,0.0,-0.045]]).reshape(-1,3)
        imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, self.camera_matrix, self.dist_coeff)
        img = draw(img,imgpts)
        cv2.imshow('img',img)
        cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        cv2.imwrite('2.jpg',img)

        return rotation_vector,translation_vector

    def camera_chessboard(self,image):
        rotation_vector,translation_vector = self.locate_chessboard(image)
        camera2chessboard_pose = Trans3D.from_angaxis(rotation_vector, tvec=translation_vector)
        camera2chessboard_pose = camera2chessboard_pose * Trans3D.from_tvec(np.array([0,0,-0.1338]))
        return camera2chessboard_pose
        
    @staticmethod
    def squarePixel(alf,num,rot_vect,trans_vect,camera_matrix):
        def constant(value):
            return 4.082*(value)**2 - 36.735*(value) + 82.653
        unit_length = 0.045
        square_ul,square_lr  = [alf*unit_length, (num-1)*unit_length, 0],[(alf+1)*unit_length, (num)*unit_length, 0]
        square_p = np.float32([square_ul,square_lr]).reshape(-1,3)
        imgpts,_ = cv2.projectPoints(square_p, rot_vect, trans_vect, camera_matrix, np.zeros((5,)))
        imgpts = np.int64(imgpts)
        square_ul,square_lr = tuple(imgpts[0].ravel()),tuple(imgpts[1].ravel())
        alf_constant,num_constant = constant(alf+1),constant(num)
        square = [square_ul[1]+int(num_constant*(num/7-8/7)),square_lr[1]+int(num_constant*(num/7-1/7)),
                square_ul[0]+int(alf_constant*((alf+1)/7-8/7)),square_lr[0]+int(alf_constant*((alf+1)/7-1/7))]
        square_draw = [square_ul[1],square_lr[1],square_ul[0],square_lr[0]]
        return square, square_draw

    def estimateSquare(self,image):
        rot_vect,trans_vect = self.estimate(image)
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        square_dict, square_draw = {}, {}
        for world,alf in alf_dict.items():
            for num in range(1,9):
                key = world + str(num)
                square, square_d = self.squarePixel(alf,num,rot_vect,trans_vect,self.camera_matrix)
                square_dict[key] = square
                square_draw[key] =  square_d
        camera2chessboard_pose = Trans3D.from_angaxis(rot_vect, tvec=trans_vect)
        camera2chessboard_pose = camera2chessboard_pose * Trans3D.from_tvec(np.array([0,0,-0.1338]))
        return square_dict,camera2chessboard_pose, square_draw

if __name__ == "__main__":
    K = np.array([1368.275572152077, 0, 791.1267138494296, 0, 1367.886598551055, 584.9421667597671, 0, 0, 1]).reshape((3,3))
    D = np.array([-0.06734559871307284, 0.1048859527883741, 0.001097241557656975, 0.002257080113766957, -0.0278104745648757])
    robot = ChessboardPoseEstimation(K,D)
    image = cv2.imread('frame0000.jpg')
    _,_,sq_dict = robot.estimate(image)
    print(sq_dict)
