import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera():
    def __init__(self):

    def takeImage(self):       
        msg = rospy.wait_for_message('avt_camera_img', Image)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        return img