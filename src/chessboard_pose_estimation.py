#!/usr/bin/env python
import cv2
import numpy as np
from numpy import linalg as LA
from scipy.spatial import ConvexHull
from scipy.spatial.distance import cdist
from transformation import Trans3D

class ChessboardPoseEstimation():

    def __init__(self,cam_mtx,dist):
        self.camera_matrix = cam_mtx
        self.dist_coeff = dist

    @staticmethod
    def crop_img(img,col,row,r = 30):
        return img[row-r:row+r,col-r:col+r]

    @staticmethod
    def point_detection(gray_img):
        points = cv2.goodFeaturesToTrack(gray_img,150,0.01,30,blockSize = 5,useHarrisDetector = True, k = 0.04)
        winSize = (5, 5)
        zeroZone = (-1, -1)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
        points = cv2.cornerSubPix(gray_img, points, winSize, zeroZone, criteria)
        return np.int0(points)

    @staticmethod
    def color_clustering(color_img):
        try:
            vectorized = color_img.reshape((-1,3))
            vectorized = np.float32(vectorized)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            K, attempts = 2, 10
            _,label,center=cv2.kmeans(vectorized,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
            center = np.uint8(center)
            center[0] = np.array([255,255,255])
            center[1] = np.array([0,0,0])
            res = center[label.flatten()]
            result_image = res.reshape((color_img.shape))
            gray_image = cv2.cvtColor(result_image,cv2.COLOR_BGR2GRAY)
            return True, gray_image
        except:
            return False, None

    @staticmethod
    def region_condition(img):
        row,col = img.shape
        upper_left = np.bincount(img[5:12,5:12].reshape(1,49)[0]).argmax()
        lower_left = np.bincount(img[row-12:row-5,5:12].reshape(1,49)[0]).argmax()
        upper_right = np.bincount(img[5:12,col-12:col-5].reshape(1,49)[0]).argmax()
        lower_right = np.bincount(img[row-12:row-5,col-12:col-5].reshape(1,49)[0]).argmax()
        result = upper_left == lower_right and upper_right == lower_left and upper_left != upper_right
        return result
    
    def chessboard_inner_points(self):
        points = self.point_detection(self.gray)
        inner_points = []
        for p in points:
            col,row = p.ravel()
            p_color_img = self.crop_img(self.img,col,row)
            ret_1, p_clustering_gray_img = self.color_clustering(p_color_img)
            if ret_1:
                ret_2 = self.region_condition(p_clustering_gray_img)
                if ret_2:
                    inner_points.append([col,row])
        return inner_points

    @staticmethod
    def histogram_mean(img):
        w,h = img.shape
        pixels_num = w*h
        histg = cv2.calcHist([img],[0],None,[256],[0,256])
        histg_sum = 0
        for i in range(len(histg)):
            histg_sum += histg[i]*(i+1)
        return histg_sum/pixels_num

    @staticmethod
    def pClosest(points,origin): 
        x, y = origin
        points.sort(key = lambda P: (P[0]-x)**2 + (P[1]-y)**2)
        points = np.array(points)
        _,point0,point1 = points[:3]
        dir_0 = (point0 - origin)/LA.norm(point0 - origin)
        dir_1 = (point1 - origin)/LA.norm(point1 - origin)
        direction = {abs(dir_0).argmax():(dir_0,point0),abs(dir_1).argmax():(dir_1,point1)}
        x_dir,x_point = direction[0]
        y_dir,y_point = direction[1]
        def get_closet_point(pin_point,points,direction,num):
            emp = []
            for p in points:
                if num > 0:
                    if not np.array_equal(p,pin_point):
                        check = (p - pin_point)/LA.norm(p - pin_point)
                        dot_product = np.dot(check, direction)
                        if dot_product > 1:
                            dot_product = 1
                        angle = np.arccos(dot_product)/3.14*180
                        if angle < 5:
                            emp.append(p)
                            num -= 1
                        else:continue
            return emp

        x_point_2 = get_closet_point(x_point,points,x_dir,1)[0]
        imageP = [origin,x_point,x_point_2]
        followup = []
        for cp in imageP:
            candid = get_closet_point(cp,points,y_dir,2)
            followup += candid
        imageP += followup
        if x_dir[0] < 0: 
            if y_dir[1] < 0:
                corner =  'lower_right'
            else:corner = 'upper_right'
        else:
            if y_dir[1] < 0:
                corner = 'lower_left'
            else:corner = 'upper_left'
        return corner,np.array(imageP, dtype='float64')
    
    def img_points(self,inner_points):
        points = np.array(inner_points)
        hull = ConvexHull(points)
        hullpoints = points[hull.vertices,:]
        hdist = cdist(hullpoints, hullpoints, metric='euclidean')
        bestpair = np.unravel_index(hdist.argmax(), hdist.shape)
        pair = [hullpoints[bestpair[0]],hullpoints[bestpair[1]]]
        brightness = 255
        for i in pair:
            col,row = i.ravel()
            candi_gray = cv2.cvtColor(self.crop_img(self.img,col,row,r = 150),cv2.COLOR_BGR2GRAY)
            if type(candi_gray) == type(None):continue 
            else:
                mean_intensity = self.histogram_mean(candi_gray)
                if mean_intensity < brightness:
                    brightness = mean_intensity
                    origin = np.array([col,row])
        corner, imageP = self.pClosest(inner_points,origin)
        return corner,imageP
        
    @staticmethod
    def object_coordinate(corner):
        #each row is a point col is the world frame coordinate define upper_left as origin
        if corner == 'upper_left':
            return np.array([[4.5,4.5,0],[9.0,4.5,0],[13.5,4.5,0],[4.5,9.0,0],
            [4.5,13.5,0],[9.0,9.0,0],[9.0,13.5,0],[13.5,9.0,0],[13.5,13.5,0]])/100
        elif corner == 'upper_right':
            return np.array([[31.5,4.5,0],[27.0,4.5,0],[22.5,4.5,0],[31.5,9.0,0],
            [31.5,13.5,0],[27.0,9.0,0],[27.0,13.5,0],[22.5,9.0,0],[22.5,13.5,0]])/100
        elif corner == 'lower_left':
            return np.array([[4.5,31.5,0],[9.0,31.5,0],[13.5,31.5,0],[4.5,27.0,0],
            [4.5,22.5,0],[9.0,27.0,0],[9.0,22.5,0],[13.5,27.0,0],[13.5,22.5,0]])/100
        else:
            return np.array([[31.5,31.5,0],[27.0,31.5,0],[22.5,31.5,0],[31.5,27.0,0],
            [31.5,22.5,0],[27.0,27.0,0],[27.0,22.5,0],[22.5,27.0,0],[22.5,22.5,0]])/100
    
    def estimate(self,image):
        self.img = cv2.GaussianBlur(image, (5,5),0)
        self.gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        inner_points = self.chessboard_inner_points()
        corner,image_points = self.img_points(inner_points)
        object_points = self.object_coordinate(corner)

        camera_matrix,dist_coeff = self.camera_matrix,self.dist_coeff
        _, rotation_vector, translation_vector = cv2.solvePnP(
                object_points, image_points, camera_matrix, dist_coeff)
        '''
        def draw(img, imgpts):
            corner = tuple(imgpts[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (255,0,0), 5)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,255,0), 5)
            img = cv2.line(img, corner, tuple(imgpts[3].ravel()), (0,0,255), 5)
            return img
        axis = np.float32([[0,0,0],[0.045,0,0], [0,0.045,0], [0.045*7.5,0.045*0.5,0]]).reshape(-1,3)
        imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, self.camera_matrix, self.dist_coeff)
        img = image.copy()
        img = draw(img,imgpts)
        for p in image_points:
            x,y = p
            cv2.circle(img,(int(x),int(y)),3,(255,0,0),-1)
        cv2.imshow('img',img)
        cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        '''
        return rotation_vector,translation_vector

    def estimatePose(self,image):
        rotation_vector,translation_vector = self.estimate(image)
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
        return square

    def estimateSquare(self,image):
        rot_vect,trans_vect = self.estimate(image)
        alf_dict = {'h':0,'g':1,'f':2,'e':3,'d':4,'c':5,'b':6,'a':7}
        square_dict, unit_length = {}, 0.045
        for world,alf in alf_dict.items():
            for num in range(1,9):
                key = world + str(num)
                square = self.squarePixel(alf,num,rot_vect,trans_vect,self.camera_matrix)
                square_dict[key] = square
        camera2chessboard_pose = Trans3D.from_angaxis(rot_vect, tvec=trans_vect)
        camera2chessboard_pose = camera2chessboard_pose * Trans3D.from_tvec(np.array([0,0,-0.1338]))
        return square_dict,camera2chessboard_pose

