import numpy as np 
import numpy.linalg as LA
from chessboard_state_detection import ChessboardStateDetection
#from robotic_chess_player.srv import PlayerService, PlayerServiceReponse

class RobotService:

    def __init__(self):
        self.chessboardStateDetection = ChessboardStateDetection()
        #self.chessboard_pose = 
        ## initialize gripper

        ## initialize camera topic subscriber

        ## setup PlayerService
        self.action_srv = rospy.ServiceProxy('action_service', Action)#send command to moveit and execute

        pass

    def serviceHandler(self, req):
        rospy.loginfo("received player service: {}".format(req.request))
        if req.request == "detect chessboard":
            rospy.loginfo("detecting chessboard")
            feedback = self.detectChessbaord()
            return PlayerServiceReponse(feedback)
        
        if req.request[:4] == "move":
            rospy.loginfo("move request received: " + req.request)
            feedback = self.moveChess(req.request)
            return PlayerServiceReponse(feedback)

        if req.request == "detect chessboard state":
            
    @staticmethod       
    def imageFromTopic():
        img_msg = rospy.wait_for_message('avt_camera_img', Image)
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def detectChessbaord(self):
        ## move robot to general standby position
        general_standby = []
        manipulator = self.action_srv
        pose_estimator = self.chessboardPoseEstimation
        #send to moveit the wait for it done
        manipulator('joint,[]')
        ## take one image
        image = self.imageFromTopic()
        ## do pose estimation
        rot_vec,trans_vec = pose_estimator.pose_estimation(image)
        ## adjust the standby position
        ori_vec = trans_vec/LA.norm(trans_vec)
        dot_product = np.dot(ori_vec, zero_direction)
        general_standby[0] = np.arccos(dot_product)
        ## send to moveit 
        manipulator('joint,[]')
        ## save the information

        self.chessboardStateDetection.setSquarePos(square_pos)
    
    def moveChess(self, request):
        ## decode the request
        chess, stat_pos, end_pos = self.decodeRequest(request)

        ## call movechess



    def __removeChess(self, chess, pos):
        '''
        remove a chess from the chessboard
        '''
        table_pos = None
        self.moveChess(chess, pos, tale_pos)

    def __moveChess(self, chess, start_pos, end_pos):
        '''
        move chess from point A to point B
        '''
