#!/usr/bin/env python3
import chess
import chess.engine
import rospy
from robotic_chess_player.srv import ChessAI, ChessAIResponse
import rospkg

def open_chess_engine():
    r = rospkg.RosPack()
    path = r.get_path('robotic_chess_player')
    engine_path = path + '/src/chess_ai/stockfish/stockfish-11-linux/Linux/stockfish_20011801_x64'
    engine = chess.engine.SimpleEngine.popen_uci(engine_path)
    extreme_setup = {'Threads': 10, 'Hash': 5000, 'Skill Level': 20,}
    engine.configure(extreme_setup)
    return engine

def service_handle(msg):
    board = chess.Board(msg.chess_board_state)
    time_limit = chess.engine.Limit(time = 0.1)
    try:
        if not board.is_game_over():
            action = engine.play(board,time_limit)
            chess_move = str(action.move)
            if board.is_capture(action.move): result = chess_move +',yes'+',no'
            elif board.is_castling(action.move): result = chess_move +',no'+',yes'
            elif len(chess_move) == 5: result = chess_move[:-1] +',no'+',no' + ',' + chess_move[-1]
            else: result = chess_move +',no'+',no'
            rospy.loginfo(msg.chess_board_state)
            rospy.loginfo(result)
            return ChessAIResponse(result)
        else: return ChessAIResponse("Game is over")
    except:
        return ChessAIResponse("Game is over")

if __name__ =="__main__":
    try:
        rospy.init_node('Chess_AI_Node')
        engine = open_chess_engine()
        rospy.loginfo("starting service.....")
        s = rospy.Service('chess_ai_service', ChessAI, service_handle)
        rospy.loginfo("chess ai service started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass