#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class TaskPlanning():
    
    def __init__(self):
        '''
        receive order from GUI node
        prepare for a game
        detect the chessboard state
        send to chess ai to generate a move
        move the robot'''
        self.gui_sub = rospy.Subscriber('/button', String, queue_size=5, callback=self.gui_callback)
        #set up the flag for actions
        self.locate_flag = False 
        self.detect_flag = False
        self.robot_flag = False
        self.correct_flag = False
        # publishe task planning message to gui
        self.info_pub = rospy.Publisher('/info_msg', String, queue_size=5)
        #init ros node
        rospy.init_node("task_planning_node")
        #connect to chess ai service
        self.ai_service = self.initService('chess_ai_service',ChessAI)
        #connect to robot service
        self.robot_service = self.initService('robot_service',RobotService)
        #connect to neural network service
        self.nn_service = self.initService('board_state', RobotService)