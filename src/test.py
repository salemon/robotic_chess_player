#!/usr/bin/env python
import rospy
from motion_planning import MotionPlanner
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge,CvBridgeError
from transformation import Trans3D
import numpy as np
from std_msgs.msg import String

rospy.init_node('test_node')

class Test:
    def __init__(self):
        self.mp = MotionPlanner()

        K = rospy.get_param('/camera_calibration/K')
        self.camera_matrix = np.array(K).reshape((3,3))
        D = rospy.get_param('/camera_calibration/D')
        self.dist_coeff = np.array(D)

        # read hand eye position from parameter server
        self.TCP2camera_pose = Trans3D.from_ROSParameterServer("/hand_eye_position")

        self.trigger = rospy.Publisher('/trigger', String, queue_size=10)
        self.img_sub = rospy.Subscriber('/avt_camera_img', Image, self.image_callback)
        self.lastest_img = None
        self.img_received = False 
        self.bridge = CvBridge()
        rospy.sleep(1)
    
    def run(self):
        # move to initial position
        self.mp.moveRobotJoint([[90, -135, 90, -70, -90, 0]])

        # take an image
        self.trigger_image()
        cv2.imshow('test', self.lastest_img)
        cv2.waitKey(-1)
        cv2.destroyAllWindows()

        # detect chessboard pose
        camera2chessboard_pose = self.estimatePoseTest(self.lastest_img)

        base2TCP_pose = self.mp.currentRobotPose()

        base2chessboard_pose = base2TCP_pose * self.TCP2camera_pose * camera2chessboard_pose

        goal_pose = base2chessboard_pose * Trans3D.from_tvec(np.array([0, 0, -0.1338-0.05]))

        self.mp.moveRobot([goal_pose], 0.2)

    def image_callback(self, msg):
        rospy.loginfo("image received")
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.lastest_img = img.copy()
        self.img_received = True 

    def trigger_image(self):
        self.trigger.publish(String('Hi!'))
        count = 0
        while(not self.img_received):
            rospy.sleep(0.1)
            count += 1
            if count > 15: break
            continue
        self.img_received = False

    def estimatePoseTest(self, image):
        def draw(img, corners, imgpts):
            corner = tuple(corners[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
            return img
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:6,0:9].T.reshape(-1,2)
        objp *= 0.022
        axis = np.float32([[0.088,0,0], [0,0.088,0], [0,0,-0.088]]).reshape(-1,3)

        # detect feature points.
        ret, corners = cv2.findChessboardCorners(image, (6,9))
        # refine pixel location
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        if ret:
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners2 = cv2.cornerSubPix(image_gray, corners, (11,11), (-1,-1), criteria)
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, self.camera_matrix, self.dist_coeff)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.camera_matrix, self.dist_coeff)
        img = draw(image, corners2, imgpts)
        cv2.imshow('test',img)
        cv2.waitKey(-1)
        cv2.destroyAllWindows()

        camera2chessboard_pose =  Trans3D.from_angaxis(rvecs, tvec=tvecs)
        print("detected chessboard to camera pose: " + camera2chessboard_pose.to_string())
        return camera2chessboard_pose 

test_obj = Test()

test_obj.run()