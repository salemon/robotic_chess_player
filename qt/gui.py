#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
import numpy as np
from std_msgs.msg import String


class Ui_MainWindow(object):
    def __init__(self):
        rospy.init_node("gui_node")
        self.info_sub = rospy.Subscriber("/info_msg", String, queue_size=5, callback=self.info_callback)
        self.command_pub = rospy.Publisher("/button", String, queue_size=2)
        self.text_pub = rospy.Publisher("/correction_msg", String, queue_size=2)
        self.board = None
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(749, 536)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.locate_chessboard = QtWidgets.QPushButton(self.centralwidget)
        self.locate_chessboard.setGeometry(QtCore.QRect(20, 40, 151, 51))
        self.locate_chessboard.setObjectName("pushButton")
        self.info_text = QtWidgets.QTextBrowser(self.centralwidget)
        self.info_text.setGeometry(QtCore.QRect(60, 130, 291, 341))
        self.info_text.setObjectName("info_text")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setGeometry(QtCore.QRect(410, 130, 301, 91))
        self.plainTextEdit.setPlainText("")
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.detect_chessboard = QtWidgets.QPushButton(self.centralwidget)
        self.detect_chessboard.setGeometry(QtCore.QRect(190, 40, 151, 51))
        self.detect_chessboard.setObjectName("pushButton_2")
        self.confirm = QtWidgets.QPushButton(self.centralwidget)
        self.confirm.setGeometry(QtCore.QRect(540, 40, 151, 51))
        self.confirm.setObjectName("pushButton_3")
        self.correct_chessboard = QtWidgets.QPushButton(self.centralwidget)
        self.correct_chessboard.setGeometry(QtCore.QRect(370, 40, 151, 51))
        self.correct_chessboard.setObjectName("pushButton_4")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 749, 20))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.locate_chessboard.setText(_translate("MainWindow", "Locate Chessboard"))
        self.detect_chessboard.setText(_translate("MainWindow", "Detect Chessboard"))
        self.confirm.setText(_translate("MainWindow", "Confirm"))
        self.correct_chessboard.setText(_translate("MainWindow", "Correct Chessboard"))


    def info_callback(self, msg):
        ret, info = msg.data.split(';')
        if ret ==  'loc':
            self.info_text.append(info)
        if ret == 'det':
            board = np.array([list(i) for i in info.split(',')])
            self.board = self.__systemRevise(board)
            self.info_text.append(str(self.board)[1:-1])
        if ret == 'mov':
            board_str = self.__msgToBoard(info)
            self.info_text.append('Robot finished the move')
            self.info_text.append(board_str)
        scrollbar = self.info_text.verticalScrollBar()
        rospy.sleep(0.01)
        scrollbar.setValue(scrollbar.maximum())

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

    def __msgToBoard(self,state_msg):
        self.board = np.array([list(i) for i in state_msg.split(',')])
        return str(self.board)[1:-1]

    def connectButtom(self):
        self.locate_chessboard.clicked.connect(self.locate_chessboard_clicked) 
        self.detect_chessboard.clicked.connect(self.detect_chessboard_clicked)
        self.confirm.clicked.connect(self.confirm_clicked)
        self.correct_chessboard.clicked.connect(self.correct_chessboard_clicked)
    
    def locate_chessboard_clicked(self):
        msg = String("locate chessboard")
        self.command_pub.publish(msg)
    
    def detect_chessboard_clicked(self):
        msg = String("detect chessboard")
        self.command_pub.publish(msg)
    
    def correct_chessboard_clicked(self):
        # get correction string
        if type(self.board) == type(None):
            self.info_text.append("Please detect the chessboard first")
        else:
            corr_comm = self.plainTextEdit.toPlainText()
            outcome = self.__humanRevise(corr_comm)
            self.info_text.append(outcome)
    
    def __humanRevise(self,corr_comm):
        col = {'a':7,'b':6,'c':5,'d':4,'e':3,'f':2,'g':1,'h':0}
        try:
            square,piece = corr_comm.split(' ')
            self.board[int(square[1])-1,col[square[0]]] = piece
            return str(self.board)[1:-1]
        except:
            return 'Correction message formate is incorrect'

    def confirm_clicked(self):
        board_msg = self.__boardToMsg()
        self.command_pub.publish('confirm;'+board_msg)
    
    def __boardToMsg(self):
        board_msg = str()
        for row in self.board:
            row_msg = str()
            for i in row:
                row_msg += i
            board_msg += row_msg + ','
        return board_msg[:-1]

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.connectButtom()
    MainWindow.show()
    sys.exit(app.exec_())