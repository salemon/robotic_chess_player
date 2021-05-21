#!/usr/bin/env python
import rospy
from robotic_chess_player.srv import *


def taskPlanning():
    #init robot service's client
    rospy.wait_for_service('robot_service')
    robot_service = rospy.ServiceProxy('robot_service', TaskPlanning)
    rospy.wait_for_service('chess_ai_service')
    ai_service = rospy.ServiceProxy('chess_ai_service', ChessAI)
    print('Place the empty chessboard on the workspace')
    board_ready = raw_input('''Hit Enter if the chessboard is in place\nHit Ctrl+C to exit the system anytime''')
    while not rospy.is_shutdown():
        if board_ready == '':
            while True:
                action = robot_service('detect chessboard').feedback
                if action == 'Detection accomplished':
                    print(action)
                    break
                else:
                    continue
        elif board_ready == 'no':
            pass
        else:
            raw_input("Ctrl+c to exit the system")
            continue
        
        while True:
            print('Please put the chess pieces in place')
            game_start = raw_input('Hit Enter if you want to start the game.')
            if game_start == '':
                human_move = raw_input('Hit Enter if you finish move')
                while True:
                    if human_move == '':
                        fen = robot_service('chessboard state').feedback
                        move = ai_service(fen).command
                        robot_move = robot_service('move:' + move).feedback
                        if robot_move == 'Done':
                            human_move = raw_input('Make a move and hit enter.')
                            continue
                    else:
                        human_intention = raw_input('Hit e if you want to leave this game, if not press any key: ')
                        if human_intention == 'e':
                            break
                        else:
                            human_move = raw_input('Make a move and hit enter ')

                new_game = raw_input('Hit enter if you want to start a new game or hit e to exit: ')
                if new_game ==  '':
                    board_ready = raw_input('Hit enter if board is moved. Type in no if board is not moved: ')
                else:
                    break
            else:
                raw_input("Ctrl+c to exit the system")
                break




if __name__ == "__main__":
    try:
        # init task planning ros node 
        rospy.init_node('task_planning', anonymous=True)
        taskPlanning()
    except rospy.ROSInterruptException:
        pass