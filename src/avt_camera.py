import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge,CvBridgeError

class AvtCamera():

    def __init__(self):
        self.trigger = rospy.Publisher('/trigger', String, queue_size=1)
        self.img_sub = rospy.Subscriber('avt_camera_img', Image, self.image_callback)
        self.lastest_img = None
        self.img_received = False 
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.lastest_img = img.copy()
        self.img_received = True 

    def trigger_image(self):
        self.trigger.publish(String('Hi!'))
        while(not self.img_received):
            continue
        self.img_received = False