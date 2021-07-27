#!/usr/bin/env python
import cv2
import numpy as np

class PoseEstimation():

    def __init__(self, cam_mtx, dist, square_length):
        self.CAM_MTX = cam_mtx
        self.DIST = dist
        self.L = square_length

    @staticmethod
    def __corner_coordinate(corner, l):
        #each row is a point col is the world frame coordinate define upper_left as origin
        if corner == 'upper_left':
            return np.array([[l, l, 0],[2*l, l, 0],[3*l, l, 0],[l, 2*l, 0],
            [l, 3*l, 0],[2*l, 2*l, 0],[2*l, 3*l , 0],[3*l, 2*l, 0],[3*l, 3*l, 0]])
        elif corner == 'upper_right':
            return np.array([[7*l, l, 0],[6*l, l, 0],[5*l, l, 0],[7*l, 2*l, 0],
            [7*l, 3*l, 0],[6*l, 2*l, 0],[6*l, 3*l, 0],[5*l, 2*l, 0],[5*l, 3*l, 0]])
        elif corner == 'lower_left':
            return np.array([[l, 7*l, 0],[2*l, 7*l, 0],[3*l, 7*l, 0],[l, 6*l, 0],
            [l, 5*l, 0],[2*l, 6*l, 0],[2*l, 5*l, 0],[3*l, 6*l, 0],[3*l, 5*l, 0]])
        else:
            return np.array([[7*l, 7*l, 0],[6*l, 7*l,0 ],[5*l, 7*l, 0],[7*l, 6*l, 0],
            [7*l, 5*l, 0],[6*l, 6*l, 0],[6*l, 5*l, 0],[5*l, 6*l, 0],[5*l, 5*l, 0]])
    
    def general_position(self, corner, image_points):
        points_coordinate = self.__corner_coordinate(corner, self.L)
        _, rotation_vector, translation_vector = cv2.solvePnP(
                points_coordinate, image_points, self.CAM_MTX,self.DIST)        
        return rotation_vector, translation_vector

    def __board_coordinate(self):
        coordinate = []
        for y in range(1,8):
            for x in range(1,8):
                coordinate.append([x * self.L, y * self.L, 0])
        return np.array(coordinate)

    def locate_chessboard(self, image_points):
        points_coordinate = self.__board_coordinate()
        _, rotation_vector, translation_vector = cv2.solvePnP(
            points_coordinate, image_points, self.CAM_MTX, self.DIST)
        return rotation_vector, translation_vector

if __name__ == "__main__":
    K = np.array([1368.275572152077, 0, 791.1267138494296, 0, 1367.886598551055, 584.9421667597671, 0, 0, 1]).reshape((3,3))
    D = np.array([-0.06734559871307284, 0.1048859527883741, 0.001097241557656975, 0.002257080113766957, -0.0278104745648757])
    L = 0.043
    robot = PoseEstimation(K,D,L)
    print(robot.board_coordinate())
    
