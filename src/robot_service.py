from chessboard_state_detection import ChessboardStateDetection
#from robotic_chess_player.srv import PlayerService, PlayerServiceReponse

class RobotService:

    def __init__(self):
        self.chessboardStateDetection = ChessboardStateDetection()
        #self.chessboard_pose = 
        ## initialize gripper

        ## initialize camera topic subscriber

        ## setup PlayerService

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
            

    
    def detectChessbaord(self):
        ## move robot to standby position

        ## take one image

        ## do pose estimation for the chessboard

        ## save information.
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
