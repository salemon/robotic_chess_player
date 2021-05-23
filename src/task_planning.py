#!/usr/bin/env python
from logging import info
from avt_camera import *
from vision_detector import *
class TaskPlanning:

    def __init__(self):
        #connect with robot service and chess ai 
        #get necessary information from chess ai and robot service and sent to gui
        rospy.init_node("task_planning_node")
        # connect to chess ai node
        rospy.wait_for_service('chess_ai_service')
        self.chess_ai = rospy.ServiceProxy('chess_ai_service', ChessAI)
        # connect to robot server node
        rospy.wait_for_service('chess_ai_service')
        self.robot_service = rospy.ServiceProxy('robot_service', RobotService)
        # connect to neural network
        rospy.wait_for_service('board_state')
        self.nn_service = rospy.ServiceProxy('board_state', TaskPlanning)
        # connect to GUI
        self.info_pub = rospy.Publisher('info', String, queue_size=5)
        self.gui_sub = rospy.Subscriber('gui_command', String, queue_size=4, callback=self.gui_callback)
        #self.result_pub = rospy.Publisher('robot_outcome', InspectionResultPart, queue_size=2)
        #self.reset_pub = rospy.Publisher('ai_move', Empty, queue_size=1)
        
        #self.result_part = InspectionResultPart()
        #self.part_id = 1
        self.locate_flag = False 
        self.detect_flag = False
        self.robot_flag = False
    
    def gui_callback(self, msg):
        rospy.loginfo("received gui command {}".format(msg.data))
        if msg.data == "locate chessboard":
            self.locate_flag = True 
        if msg.data == "detect chessboard":
            self.detect_flag = True 
        if msg.data == "human player finished a move":
            self.robot_flag = True

    def run(self):
        rospy.loginfo("task planning is running now")
        while(not rospy.is_shutdown()):
            if(self.locate_flag):
                rospy.loginfo("locating chessboard position")
                self.locating_chessboard()
                self.locate_flag = False
            if(self.detect_flag):
                rospy.loginfo("detecting chessboard state")
                self.detecting_chessboard()
                self.detect_flag = False
            if(self.robot_flag):
                rospy.loginfo("robot making move")
                self.robot_move()
                self.robot_flag = False
            rospy.sleep(0.1)
    
    def locating_chessboard(self):
        #let the robot_service to locate the chesboard
        #robot service return Done:square_dict if succefful
        #else: return a fail
        #and tell gui the locating is finished
        info = self.robot_service('locate chessboard').feedback
        if info[:4] ==  'Done':

        rospy.loginfo(msg)
    
    def detecting_chessboard(self):
        chessboard = self.robot_service('detect chessboard')
        return a detected stat_result
        prompt usr input and modify the stat_
        and return back to task planning and give it back to robot service
    
    def robot_move():
        self.detect Chessboard
        self.robot_service(generate fen string)
        self.chess_ai fen String
        command = 'move' + next move 
        self_robot_service(command)

        



if __name__ == "__main__":
    try:
        # init task planning ros node 
        rospy.init_node('task_planning', anonymous=True)
        taskPlanning()
    except rospy.ROSInterruptException:
        pass