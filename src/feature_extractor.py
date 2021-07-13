import cv2
import numpy as np
from numpy import linalg as LA
from scipy.spatial import ConvexHull
from scipy.spatial.distance import cdist

class FeatureExtractor():

    @staticmethod
    def point_detection(gray_img):
        points = cv2.goodFeaturesToTrack(gray_img,150,0.01,30,blockSize = 5,useHarrisDetector = True, k = 0.04)
        winSize = (5, 5)
        zeroZone = (-1, -1)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
        points = cv2.cornerSubPix(gray_img, points, winSize, zeroZone, criteria)
        return np.int0(points)

    @staticmethod
    def crop_img(img,col,row,r = 30):
        return img[row-r:row+r,col-r:col+r]

    @staticmethod
    def color_clustering(color_img):
        try:
            vectorized = np.float32(color_img.reshape((-1,3)))
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            _,label,center=cv2.kmeans(vectorized, 2, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
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
    def feature_filter(img):
        row,col = img.shape
        upper_left = np.bincount(img[5:12,5:12].reshape(1,49)[0]).argmax()
        lower_left = np.bincount(img[row-12:row-5,5:12].reshape(1,49)[0]).argmax()
        upper_right = np.bincount(img[5:12,col-12:col-5].reshape(1,49)[0]).argmax()
        lower_right = np.bincount(img[row-12:row-5,col-12:col-5].reshape(1,49)[0]).argmax()
        result = upper_left == lower_right and upper_right == lower_left and upper_left != upper_right
        return result
    
    def chessboard_features(self, image):
        gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        points = self.point_detection(gray_image)
        features = []
        for p in points:
            col,row = p.ravel()
            p_color_img = self.crop_img(image,col,row)
            ret_1, p_clustering_gray_img = self.color_clustering(p_color_img)
            if ret_1:
                ret_2 = self.feature_filter(p_clustering_gray_img)
                if ret_2:
                    features.append([col,row])           
        return features

    @staticmethod
    def histogram_mean(img):
        w, h = img.shape
        pixels_num = w * h
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

    def pose_estimation_points(self,image):
        features = self.chessboard_features(image)
        np_features = np.array(features)
        hull = ConvexHull(np_features)
        hullpoints = np_features[hull.vertices,:]
        hdist = cdist(hullpoints, hullpoints, metric='euclidean')
        bestpair = np.unravel_index(hdist.argmax(), hdist.shape)
        pair = [hullpoints[bestpair[0]],hullpoints[bestpair[1]]]
        brightness = 255
        for i in pair:
            col,row = i.ravel()
            candi_gray = cv2.cvtColor(self.crop_img(image,col,row,r = 150),cv2.COLOR_BGR2GRAY)
            if type(candi_gray) == type(None):
                continue 
            else:
                mean_intensity = self.histogram_mean(candi_gray)
                if mean_intensity < brightness:
                    brightness = mean_intensity
                    origin = np.array([col,row])      
        corner, imageP = self.pClosest(features,origin)
        return corner,imageP
    
if __name__ == "__main__":
    vision = FeatureExtractor()
    img = cv2.imread('frame0000.jpg')
    # 1
    blur=cv2.GaussianBlur(img,(0,0),3)
    image=cv2.addWeighted(img,1.5,blur,-0.5,0)
    # 2
    kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    image = cv2.filter2D(img, -1, kernel)
    # 3
    image=cv2.bilateralFilter(img,9,75,75)
    # 4
    sigma = 1; threshold = 5; amount = 1
    blurred=cv2.GaussianBlur(img,(0,0),1,None,1)
    lowContrastMask = abs(img - blurred) < threshold
    sharpened = img*(1+amount) + blurred*(-amount)
    image=cv2.bitwise_or(sharpened.astype(np.uint8),lowContrastMask.astype(np.uint8))
    _,gray_img = vision.color_clustering(image)
    '''
    for p in points:
        x,y = p
        cv2.circle(new_image,(int(x),int(y)),3,(255,0,0),-1)
    '''
    cv2.imshow('img',gray_img)
    cv2.waitKey(0) & 0xFF
    cv2.destroyAllWindows()





