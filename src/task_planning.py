#!/usr/bin/env python
import rospy
from robotic_chess_player.srv import *
from std_msgs.msg import String
import numpy as np
class TaskPlanning():

    def __init__(self):
        #init ros node
        rospy.init_node("task_planning_node")
        
        #connect to chess ai service
        #self.chess_ai_service = self.initService('chess_ai_service',ChessAI)
        #connect to robot service
        self.robot_service = self.initService('robot_service',RobotService)
        #connect to neural network service
        self.nn_service = self.initService('board_state', RobotService)
        #subscribe message from gui
        
        self.gui_sub = rospy.Subscriber('/button', String, queue_size=5, callback=self.gui_callback)
        #set up the flag for actions
        self.locate_flag = False 
        self.detect_flag = False
        self.robot_flag = False
        self.correct_flag = False
        # publishe task planning message to gui
        self.info_pub = rospy.Publisher('/info_msg', String, queue_size=5)

    def initService(self,service_name,service_message):
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, service_message)
        rospy.loginfo('Successfully connected to {}'.format(service_name))
        return service

    def gui_callback(self, command):
        rospy.loginfo("received gui command: {}".format(command.data))
        if command.data == "locate chessboard":
            self.locate_flag = True 
        if command.data == "detect chessboard":
            self.detect_flag = True
        if command.data[:5] == "confirm":
            self.last_state = command.data.split(';')
            self.robot_flag = True
      
    def run(self):
        rospy.loginfo("task planning is running now")
        while not rospy.is_shutdown():
            if self.locate_flag:
                rospy.loginfo("locating chessboard position")
                self.locating_chessboard()
                self.locate_flag = False
            if self.detect_flag:
                rospy.loginfo("detecting chessboard state")
                #state_msg = self.detecting_chessboard()
                #self.board_state_pub.publish(state_msg)
                #self.robot_service('to standby')
                self.detect_flag = False
            if self.robot_flag:
                #self.robot_move()
                self.robot_flag = False
            rospy.sleep(0.1)


    def locating_chessboard(self):
        info = self.robot_service('locate chessboard').feedback
        if info[:4] ==  'Done':
            str_square_dict = info.split(';')[1]
            try:
                resp = self.nn_service(str_square_dict)
                rospy.loginfo(resp.feedback)
                self.info_pub.publish('Location Accomplished')
            except rospy.ServiceException as e:
                self.info_pub.publish('Location Accomplished But Neural Network Node did not receive square dictionary')
        else:
            self.info_pub.publish('Location Failed, Please remove possible noise and locate again')

    def detecting_chessboard(self):
        self.robot_service('to take image')
        return self.nn_service('state').feedback

    def robot_move(self):
        #received previous correct chessboard state string
        #get the detection and revise 
        #send back to GUI to verify and get the confirmed state
        #transfrom to fen and sent to ai
        #get the next move and send to robot service
        '''this function needs to communicate with the gui
        revise finished published the new message if the message'''
        
        state_msg = self.detecting_chessboard()
        syscorr_state_msg = self.__systemRevise(state_msg)



    def __systemRevise(self,chessboard):
        try:
            for row in range(8):
                for col in range(8):
                    if not chessboard[row,col].isupper() and self.board[row,col].islower() and chessboard[row,col] != self.board[row,col]: 
                        chessboard[row,col] = self.board[row,col]
                else:continue 
        except TypeError:
            pass
        return chessboard
if __name__ == "__main__":
    try:
        obj = TaskPlanning()
        obj.run()
    except rospy.ROSInterruptException:
        pass