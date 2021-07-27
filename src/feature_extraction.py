import cv2
import numpy as np
from numpy import linalg as LA
from scipy.spatial import ConvexHull
from scipy.spatial.distance import cdist

class FeatureExtraction():

    @staticmethod
    def point_detection(gray_img):
        points = cv2.goodFeaturesToTrack(gray_img,150,0.01,30,blockSize = 5,useHarrisDetector = True, k = 0.04)
        winSize = (5, 5)
        zeroZone = (-1, -1)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
        points = cv2.cornerSubPix(gray_img, points, winSize, zeroZone, criteria)
        return np.int0(points)

    @staticmethod
    def crop_image(img,col,row,r = 30):
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
    def __filter(image):
        row,col = image.shape
        upper_left = np.unique(image[0:10,0:10].reshape(1,100)[0])
        lower_left = np.unique(image[row-10:row,0:10].reshape(1,100)[0])
        upper_right = np.unique(image[0:10,col-10:col].reshape(1,100)[0])
        lower_right = np.unique(image[row-10:row,col-10:col].reshape(1,100)[0])
        result = (upper_left == lower_right).all() and (upper_right == lower_left).all() and (upper_left != upper_right).all()
        return result
    
    def chessboard_features(self, image):
        gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        points = self.point_detection(gray_image)
        features = []
        for p in points:
            col,row = p.ravel()
            p_color_img = self.crop_image(image,col,row)
            ret_1, p_clustering_gray_img = self.color_clustering(p_color_img)
            if ret_1:
                ret_2 = self.__filter(p_clustering_gray_img)
                if ret_2:
                    features.append((col,row))          
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

    def general_pose_points(self,image):
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
            candi_gray = cv2.cvtColor(self.crop_image(image,col,row,r = 150),cv2.COLOR_BGR2GRAY)
            if type(candi_gray) == type(None):
                continue 
            else:
                mean_intensity = self.histogram_mean(candi_gray)
                if mean_intensity < brightness:
                    brightness = mean_intensity
                    origin = np.array([col,row])      
        corner, imageP = self.pClosest(features,origin)
        return corner,imageP
    
    def closer_view_estimation_points(self,image):
        points = self.chessboard_features(image)
        points.sort(key = lambda x: x[1])
        sorted_points = []
        for n in range(0, len(points), 7):
            sorted_points.extend(sorted(points[n:n+7]))
        return np.array(sorted_points, dtype='float64')

if __name__ == "__main__":
    import ast
    vision = FeatureExtraction()
    img = cv2.imread('frame0000.jpg')
    #points = vision.closer_view_estimation_points(img)
    #count = 1
    font = cv2.FONT_HERSHEY_SIMPLEX
    square_dict = "Done {'f1': [83, 206, 538, 661], 'f2': [205, 327, 539, 662], 'f3': [327, 449, 539, 662], 'f4': [449, 571, 539, 662], 'd8': [937, 1060, 784, 908], 'f6': [693, 815, 540, 663], 'f7': [816, 938, 540, 663], 'f8': [938, 1061, 540, 664], 'h3': [328, 450, 296, 418], 'h1': [84, 206, 295, 418], 'h6': [693, 816, 296, 419], 'h7': [816, 939, 297, 420], 'h4': [449, 572, 296, 419], 'h5': [571, 694, 296, 419], 'b4': [447, 570, 1028, 1152], 'b5': [569, 692, 1029, 1152], 'b6': [691, 814, 1029, 1153], 'b7': [815, 938, 1029, 1153], 'b1': [81, 204, 1027, 1151], 'b2': [203, 326, 1027, 1151], 'b3': [325, 448, 1028, 1151], 'd6': [692, 815, 784, 907], 'd7': [815, 938, 784, 907], 'd4': [448, 571, 783, 906], 'd5': [570, 693, 783, 907], 'b8': [937, 1060, 1030, 1153], 'e7': [815, 938, 662, 785], 'd1': [82, 205, 782, 905], 'f5': [571, 693, 540, 663], 'e1': [83, 205, 660, 783], 'e5': [570, 693, 661, 785], 'd3': [326, 449, 783, 906], 'h2': [206, 328, 295, 418], 'e3': [326, 449, 661, 784], 'h8': [938, 1061, 297, 420], 'e2': [205, 327, 660, 784], 'g7': [816, 939, 418, 541], 'g6': [693, 816, 418, 541], 'g5': [571, 694, 418, 541], 'g4': [449, 572, 417, 541], 'g3': [327, 450, 417, 540], 'g2': [205, 328, 417, 540], 'g1': [84, 206, 417, 540], 'e4': [448, 571, 661, 784], 'g8': [938, 1061, 419, 542], 'a1': [81, 203, 1149, 1273], 'a3': [325, 447, 1150, 1274], 'c8': [937, 1060, 906, 1030], 'a5': [569, 692, 1151, 1274], 'a4': [447, 570, 1150, 1274], 'a7': [814, 937, 1152, 1275], 'a6': [691, 814, 1151, 1275], 'c3': [326, 448, 905, 1028], 'a8': [937, 1060, 1152, 1276], 'c1': [82, 204, 904, 1027], 'e6': [692, 815, 662, 785], 'c7': [815, 938, 906, 1030], 'c6': [692, 815, 906, 1029], 'c5': [570, 692, 905, 1029], 'c4': [448, 570, 905, 1029], 'd2': [204, 327, 782, 906], 'a2': [203, 325, 1150, 1273], 'c2': [204, 326, 904, 1028], 'e8': [937, 1060, 662, 786]}"
    # fontScale
    fontScale = 1
    
    # Blue color in BGR
    color = (255, 0, 0)
    
    # Line thickness of 2 px
    thickness = 2
    image = cv2.rectangle(img, (83, 206), (538, 661), color, thickness)
    # Using cv2.putText() method
    '''
    for key in square_dict:
        print(square_dict[key])
        
        #img = cv2.putText(img, str(count), (int(x),int(y)), font, 
        #           fontScale, color, thickness, cv2.LINE_AA)
        #count += 1
    '''
    cv2.imshow('img',image)
    cv2.waitKey(0) & 0xFF
    cv2.destroyAllWindows()


