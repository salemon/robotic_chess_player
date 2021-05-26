#!/usr/bin/env python


from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from std_msgs.msg import String


class Ui_MainWindow(object):
    def __init__(self):
        rospy.init_node("gui_node")
        self.info_sub = rospy.Subscriber("/info_msg", String, queue_size=5, callback=self.info_callback)
        self.command_pub = rospy.Publisher("/button", String, queue_size=2)
        self.text_pub = rospy.Publisher("/correction_msg", String, queue_size=2)

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
        self.confirm.setGeometry(QtCore.QRect(370, 40, 151, 51))
        self.confirm.setObjectName("pushButton_3")
        self.correct_chessboard = QtWidgets.QPushButton(self.centralwidget)
        self.correct_chessboard.setGeometry(QtCore.QRect(540, 40, 151, 51))
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
        self.info_text.append(msg.data)
        scrollbar = self.info_text.verticalScrollBar()
        rospy.sleep(0.01)
        scrollbar.setValue(scrollbar.maximum())

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
    
    def confirm_clicked(self):
        msg = String("confirm")
        self.command_pub.publish(msg)
    
    def correct_chessboard_clicked(self):
        # get correction string
        string = self.plainTextEdit.toPlainText()
        print(string)

        # add string to display
        self.info_text.append("HAhas")
        # send correction msg
        # TODO
    

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.connectButtom()
    MainWindow.show()
    sys.exit(app.exec_())